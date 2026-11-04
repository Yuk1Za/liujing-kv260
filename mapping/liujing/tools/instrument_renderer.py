"""Create a separate IP copy with passive counters and explicit read-only metadata.

Never modifies a running HLS project or the accepted IP. This extension still
needs control-register simulation, OOC synthesis, timing closure and board parity.
"""
import argparse,copy,hashlib,json,shutil,xml.etree.ElementTree as E
from pathlib import Path

NAMES=('signature','cycles','surfel_read_beats','surfel_read_wait','surfel_read_backpressure','surfel_address_wait','index_read_beats','index_read_wait','output_write_beats','output_write_backpressure','output_address_wait','index_address_wait')

def replace_once(text,old,new):
 if text.count(old)!=1:raise ValueError('Unsupported generated RTL marker: '+old[:80])
 return text.replace(old,new,1)

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--out',required=True);p.add_argument('--hwh',required=True);p.add_argument('--last-register',choices=['mode','debug'],default='mode');a=p.parse_args()
 src=Path(a.source);dest=Path(a.out);rtl=src/'hdl/verilog';counter=Path(__file__).resolve().parents[1]/'hardware/telemetry/renderer_metrics.v'
 top=(rtl/'lj_raster.v').read_text();control=(rtl/'lj_raster_control_s_axi.v').read_text()
 if 'renderer_metrics' in top:raise ValueError('Already instrumented')
 control=replace_once(control,'input  wire                          ap_idle\n','input  wire                          ap_idle,\n    input wire [383:0] metrics\n')
 marker=f'                ADDR_{a.last_register.upper()}_DATA_0: begin\n                    rdata <= int_{a.last_register}[31:0];\n                end'
 cases='\n'.join("                8'h%02x: rdata <= metrics[%d+:32];"%(0xb0+4*i,32*i) for i in range(len(NAMES)))
 control=replace_once(control,marker,marker+'\n'+cases)
 top=replace_once(top,'control_s_axi_U(\n','control_s_axi_U(\n    .metrics(lj_metrics_bus),\n')
 top=replace_once(top,'lj_raster_control_s_axi #(\n','wire [383:0] lj_metrics_bus;\nlj_raster_control_s_axi #(\n')
 ports={'clk':'ap_clk','reset':'!ap_rst_n','start':'ap_start','done':'ap_done'}
 for prefix,axi,signals in [('surf','surf',('arvalid','arready','arlen','rvalid','rready')),('index','index',('arvalid','arready','arlen','rvalid','rready')),('out','output_mem',('awvalid','awready','wvalid','wready'))]:
  ports.update({prefix+'_'+s:'m_axi_'+axi+'_'+s.upper() for s in signals})
 ports['metrics']='lj_metrics_bus'
 instance='renderer_metrics lj_metrics_u(\n'+',\n'.join(' .%s(%s)'%x for x in ports.items())+'\n);\n'
 top=replace_once(top,'endmodule',instance+'endmodule')+'\n// LiuJing passive instrumentation, requires separate control/counter validation.\n'+counter.read_text()
 # Keep namespace prefixes used by vendor extensions and type references.
 for _,ns in E.iterparse(src/'component.xml',events=['start-ns']):E.register_namespace(*ns)
 xml=E.parse(src/'component.xml');block=xml.getroot().find('.//{*}memoryMap/{*}addressBlock')
 namespace=block.tag.split('}')[0]+'}';reg_template=next(r for r in block.findall('{*}register') if r.findtext('{*}name')==a.last_register)
 original_registers=[E.tostring(x) for x in block.findall('{*}register')]
 if max(int(x.findtext('{*}addressOffset'),0) for x in block.findall('{*}register'))>=0xb0:raise ValueError('Counter range occupied')
 for i,name in enumerate(NAMES):
  r=copy.deepcopy(reg_template);full='metric_'+name
  for tag,value in [('name',full),('displayName',full),('description','LiuJing passive '+name),('addressOffset',str(0xb0+4*i)),('access','read-only')]:r.find(namespace+tag).text=value
  r.find('{*}reset/{*}value').text=str(0x4c4a0101 if i==0 else 0)
  field=r.find('{*}field');field.find('{*}name').text=full;field.find('{*}access').text='read-only';field.find('{*}description').text=name
  field.remove(field.find('{*}writeValueConstraint'));block.append(r)
 assert original_registers==[E.tostring(x) for x in block.findall('{*}register')[:len(original_registers)]]
 # Extend only the renderer's register metadata in the compatible static HWH.
 hwh=E.parse(a.hwh);module=next(m for m in hwh.getroot().iter('MODULE') if m.get('INSTANCE')=='lj_raster_0')
 parent=next(n for n in module.iter() if n.find('REGISTER') is not None)
 template=next(n for n in parent.findall('REGISTER') if n.get('NAME')==a.last_register)
 for i,name in enumerate(NAMES):
  r=copy.deepcopy(template);r.set('NAME','metric_'+name)
  for prop in r.findall('PROPERTY'):
   key=prop.get('NAME')
   if key=='ADDRESS_OFFSET':prop.set('VALUE',str(0xb0+4*i))
   elif key=='ACCESS':prop.set('VALUE','read-only')
   elif key=='DESCRIPTION':prop.set('VALUE','LiuJing passive '+name)
   elif key=='RESET_VALUE':prop.set('VALUE',str(0x4c4a0101 if i==0 else 0))
  field=r.find('FIELDS/FIELD');field.set('NAME','metric_'+name)
  for prop in field.findall('PROPERTY'):
   if prop.get('NAME')=='ACCESS':prop.set('VALUE','read-only')
   if prop.get('NAME')=='DESCRIPTION':prop.set('VALUE',name)
  parent.append(r)
 shutil.copytree(src,dest) # fails if output exists, preserving every prior experiment
 (dest/'hdl/verilog/lj_raster.v').write_text(top);(dest/'hdl/verilog/lj_raster_control_s_axi.v').write_text(control)
 xml.write(dest/'component.xml',encoding='utf-8',xml_declaration=True);hwh.write(dest/'instrumented.hwh',encoding='utf-8',xml_declaration=True)
 report=dict(contract='passive_axi_metrics_v1',signature=0x4c4a0101,registers={name:0xb0+4*i for i,name in enumerate(NAMES)},scope='counts only; no dataflow/handshake outputs changed; cache hits and fragment conflicts not included',sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (src/'component.xml',Path(a.hwh),counter,Path(__file__),dest/'component.xml',dest/'hdl/verilog/lj_raster.v',dest/'hdl/verilog/lj_raster_control_s_axi.v',dest/'instrumented.hwh')})
 (dest/'instrumentation.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(contract=report['contract'],registers=report['registers']),indent=2))
if __name__=='__main__':main()
