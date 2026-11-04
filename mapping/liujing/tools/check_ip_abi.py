"""Check interface metadata only, NOT optimized-netlist replacement safety.

An old opt DCP can have constant-propagated interface signals removed even when
every XML port matches. Rebuild the platform when behavior changes (e.g. burst
length), and never interpret this metadata comparison as a boundary-logic proof.
"""
import argparse,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--before',required=True);p.add_argument('--after',required=True);p.add_argument('--out',required=True);p.add_argument('--metrics-extension',action='store_true');a=p.parse_args()
def signature(path):
 root=ET.parse(path).getroot();ports={};registers={};buses={}
 for bus in root.findall('{*}busInterfaces/{*}busInterface'):
  kind=bus.find('{*}busType');name=bus.findtext('{*}name')
  if kind is None:raise ValueError('missing bus type '+str(name))
  attrs={k.split('}')[-1]:v for k,v in kind.attrib.items()}
  parameters={x.findtext('{*}name'):x.findtext('{*}value') for x in bus.findall('{*}parameters/{*}parameter')}
  contract={k:parameters[k] for k in ('PROTOCOL','READ_WRITE_MODE','DATA_WIDTH','ADDR_WIDTH','ID_WIDTH','NUM_READ_OUTSTANDING','NUM_WRITE_OUTSTANDING','MAX_READ_BURST_LENGTH','MAX_WRITE_BURST_LENGTH') if k in parameters}
  if attrs.get('name')=='aximm' and not all(k in contract for k in ('PROTOCOL','READ_WRITE_MODE')):raise ValueError('missing AXI behavior metadata '+str(name))
  buses[name]=dict(type=attrs,contract=contract,mapping=sorted((m.findtext('{*}logicalPort/{*}name'),m.findtext('{*}physicalPort/{*}name')) for m in bus.findall('{*}portMaps/{*}portMap')))
 for port in root.findall('.//{*}model/{*}ports/{*}port'):
  name=port.findtext('{*}name');wire=port.find('{*}wire')
  if wire is None:raise ValueError('non-wire port '+str(name))
  ports[name]={'direction':wire.findtext('{*}direction'),'left':wire.findtext('{*}vector/{*}left'),'right':wire.findtext('{*}vector/{*}right')}
 for reg in root.findall('.//{*}memoryMap/{*}addressBlock/{*}register'):
  name=reg.findtext('{*}name');registers[name]={k:reg.findtext('{*}'+k) for k in ('addressOffset','size','access')}
  registers[name]['fields']=[{k:f.findtext('{*}'+k) for k in ('name','bitOffset','bitWidth','access')} for f in reg.findall('{*}field')]
 if not ports or not registers:raise ValueError('empty interface extraction')
 return dict(ports=ports,registers=registers,buses=buses)
before,after=signature(a.before),signature(a.after);r=dict(exact=before==after,before=before,after=after,sha256={s:hashlib.sha256(Path(s).read_bytes()).hexdigest() for s in (a.before,a.after)})
r['scope']='interface metadata only; does not prove safety of substituting into an optimized checkpoint'
r['compatible_metrics_extension']=False
if a.metrics_extension:
 names=('signature','cycles','surfel_read_beats','surfel_read_wait','surfel_read_backpressure','surfel_address_wait','index_read_beats','index_read_wait','output_write_beats','output_write_backpressure','output_address_wait','index_address_wait')
 extra=set(after['registers'])-set(before['registers'])
 compatible=before['ports']==after['ports'] and before['buses']==after['buses'] and all(after['registers'].get(k)==v for k,v in before['registers'].items()) and extra=={'metric_'+n for n in names}
 for i,n in enumerate(names):
  v=after['registers'].get('metric_'+n,{})
  compatible &= v.get('addressOffset')==str(0xb0+4*i) and v.get('size')=='32' and v.get('access')=='read-only' and v.get('fields')==[dict(name='metric_'+n,bitOffset='0',bitWidth='32',access='read-only')]
 r['compatible_metrics_extension']=compatible;r['hwh_action']='must use separately generated instrumented.hwh; original HWH is incomplete'
Path(a.out).write_text(json.dumps(r,indent=2));print(json.dumps(dict(exact=r['exact'],compatible_metrics_extension=r['compatible_metrics_extension'],ports=len(after['ports']),registers=len(after['registers']))))
if not (r['exact'] or r['compatible_metrics_extension']):raise SystemExit('ABI differs: regenerate the platform and HWH; do not reuse')
