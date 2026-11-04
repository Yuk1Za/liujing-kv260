"""Archive one completed, independently named hardware experiment."""
import argparse,hashlib,json,shutil,tarfile,zipfile,xml.etree.ElementTree as E
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--build',required=True);p.add_argument('--hls',required=True);p.add_argument('--project',default='liujing_stream');p.add_argument('--lanes',type=int,choices=[1,2,4],default=2);p.add_argument('--dma-alignment',type=int,choices=[1,64],default=1);p.add_argument('--version',required=True);p.add_argument('--numeric-contract',choices=['mixed','qrcp_v1'],default='mixed');p.add_argument('--descriptor-layout',choices=['float_bounds','int_bounds_v1'],default='float_bounds');p.add_argument('--instrumented-ip');a=p.parse_args()
if a.descriptor_layout=='int_bounds_v1' and a.numeric_contract!='qrcp_v1':raise ValueError('packed bounds currently require qrcp_v1')
root=Path(__file__).resolve().parents[1];build=Path(a.build);hls=Path(a.hls)
if (build/'platform_input.json').exists():
 platform=json.loads((build/'platform_input.json').read_text())
 ipxml=E.parse(Path(platform['ip'])/'component.xml').getroot()
 output_bus=next(b for b in ipxml.findall('{*}busInterfaces/{*}busInterface') if b.findtext('{*}name')=='m_axi_output_mem')
 expected_burst=int(next(p.findtext('{*}value') for p in output_bus.findall('{*}parameters/{*}parameter') if p.findtext('{*}name')=='MAX_WRITE_BURST_LENGTH'))
 boundary=json.loads((build/'boundary_check.json').read_text())
 if not boundary['passed'] or boundary['maximum_burst']!=expected_burst or boundary['source_sha256']!=hashlib.sha256((build/'boundary.tsv').read_bytes()).hexdigest():raise ValueError('fresh platform lacks verified retained AXI burst boundary')
instrumented=Path(a.instrumented_ip) if a.instrumented_ip else None
if instrumented:
 proof=json.loads((instrumented/'instrumentation.json').read_text());fresh=(build/'platform_input.json').is_file();replacement=json.loads((build/('platform_input.json' if fresh else 'replacement_input.json')).read_text())
 if proof['contract']!='passive_axi_metrics_v1' or not replacement.get('metrics') or Path(replacement['ip']).resolve()!=instrumented.resolve():raise ValueError('instrumented IP was not the implemented input')
 for filename,digest in proof['sha256'].items():
  f=Path(filename)
  if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=digest:raise ValueError('instrumentation inputs changed: '+filename)
 if fresh:
  # Fresh BD has its own interconnect metadata. Compare all renderer register
  # semantics, not whole-file equality against an unrelated old platform HWH.
  def register_signature(path):
   module=next(m for m in E.parse(path).getroot().iter('MODULE') if m.get('INSTANCE')=='lj_raster_0')
   return {r.get('NAME'):{p.get('NAME'):p.get('VALUE') for p in r.findall('PROPERTY') if p.get('NAME') in ('ADDRESS_OFFSET','SIZE','ACCESS')} for r in module.iter('REGISTER')}
  if register_signature(instrumented/'instrumented.hwh')!=register_signature(build/'artifacts/liujing.hwh'):raise ValueError('fresh HWH renderer register contract differs')
 else:
  if (instrumented/'instrumented.hwh').read_bytes()!=(build/'artifacts/liujing.hwh').read_bytes():raise ValueError('implemented HWH differs from counter metadata')
 validation=json.loads((instrumented/'metrics_validation/report.json').read_text())
 if not validation['passed']:raise ValueError('counter/control RTL validation did not pass')
 for name,source in [('renderer_metrics.v',root/'hardware/telemetry/renderer_metrics.v'),('lj_raster_control_s_axi.v',instrumented/'hdl/verilog/lj_raster_control_s_axi.v')]:
  if hashlib.sha256(source.read_bytes()).hexdigest()!=validation['sha256'][name]:raise ValueError('tested counter/control RTL differs from packaged input')
if not a.version.replace('_','').isalnum():raise ValueError('simple version name required')
out=root/'artifacts'/a.version
if out.exists():raise FileExistsError(out)
required=['liujing.bit','liujing.hwh','timing.rpt','utilization.rpt','drc.rpt','route.rpt','check_timing.rpt']
for name in required:
 if not (build/'artifacts'/name).is_file():raise ValueError('incomplete implementation '+name)
log=(hls/'build.log').read_text(errors='replace')
if 'C/RTL co-simulation finished: PASS' not in log:raise ValueError('missing RTL parity')
if (hls/'stress.log').is_file() and 'C/RTL co-simulation finished: PASS' not in (hls/'stress.log').read_text(errors='replace'):
 raise ValueError('stress run exists but has not passed; do not package for deployment')
# The build script writes a bit only after non-negative setup/hold and no DRC errors.
out.mkdir(parents=True)
for name in required:shutil.copyfile(build/'artifacts'/name,out/name)
shutil.copyfile(build/'platform.tcl',out/'implemented_platform.tcl')
for name in ('route_checkpoint.tcl','replace_renderer.tcl','recovery_input.json','replacement_input.json','platform_input.json','boundary.tsv','boundary_check.json','boundary_audit.log'):
 if (build/name).is_file():shutil.copyfile(build/name,out/name)
for name in ('raster.cpp','raster.hpp','tb.cpp','core.cpp','core.hpp','build.tcl'):
 shutil.copyfile(hls/name,out/name)
for name in ('tb_initial.cpp','cosim_initial.rpt','stress_existing_hls.tcl','stress.log','check_lane_variants.tcl','synthesize_lane_variant.tcl','verify_lane_variant.tcl','lanes_edges.log','lanes4_synth.log'):
 if (hls/name).is_file():shutil.copyfile(hls/name,out/name)
for relative,name in [('solution/sim/report/lj_raster_cosim.rpt','hls_cosim.rpt'),('solution/syn/report/lj_raster_csynth.rpt','hls_synthesis.rpt')]:
 shutil.copyfile(hls/a.project/relative,out/name)
if instrumented:
 for source,name in [(instrumented/'instrumentation.json','instrumentation.json'),(root/'hardware/telemetry/renderer_metrics.v','renderer_metrics.v'),(instrumented/'metrics_validation/report.json','metrics_validation.json'),(instrumented/'metrics_validation/metrics_sim_simulation.log','metrics_unit_simulation.log'),(instrumented/'metrics_validation/control_metrics_sim_simulation.log','metrics_control_simulation.log')]:shutil.copyfile(source,out/name)
for label,ip in [('c16',Path('E:/tmp/losam_c16_clean_20261022/c16_hls/solution/impl/ip')),('raster',instrumented or hls/a.project/'solution/impl/ip')]:
 with zipfile.ZipFile(out/(label+'_ip.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for f in sorted(ip.rglob('*')):
   if f.is_file():z.write(f,f.relative_to(ip).as_posix())
manifest=dict(version=a.version,part='xck26-sfvc784-2LV-c',clock_target_mhz=100,tool='Vivado/Vitis 2026.1',scope='hardware implementation; not board acceptance',sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in out.iterdir() if f.is_file()})
manifest['numeric_contract']=a.numeric_contract
manifest['descriptor_layout']=a.descriptor_layout
manifest['lanes']=a.lanes
manifest['dma_alignment']=a.dma_alignment
if instrumented:
 manifest['metrics_contract']='passive_axi_metrics_v1'
 manifest['rtl_validation_scope']='HLS datapath passed full C/RTL tests before passive instrumentation; added counters and AXI-Lite extension passed independent RTL tests; actual instrumented board parity remains required'
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
archive=root/'evidence'/f'{a.version}_hardware.tgz'
with tarfile.open(archive,'w:gz') as tar:
 for name in ('liujing.bit','liujing.hwh','manifest.json'):tar.add(out/name,arcname=f'liujing/artifacts/{a.version}/{name}')
print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bit_sha256=manifest['sha256']['liujing.bit']),indent=2))
