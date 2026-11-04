"""Simulate the exact counter/control RTL packaged for one instrumented IP.

This does not replace full datapath C/RTL tests or subsequent board parity.
"""
import argparse, hashlib, json, shutil, subprocess
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--ip',required=True);a=p.parse_args()
ip=Path(a.ip).resolve();root=Path(__file__).resolve().parents[1]
out=ip/'metrics_validation';out.mkdir()
sources={name:root/'hardware/telemetry'/name for name in ('renderer_metrics.v','tb_metrics.sv','tb_control_metrics.sv')}
sources['lj_raster_control_s_axi.v']=ip/'hdl/verilog/lj_raster_control_s_axi.v'
# The separately simulated counter must be byte-identical to the appended RTL.
if not (ip/'hdl/verilog/lj_raster.v').read_text().endswith(sources['renderer_metrics.v'].read_text()):raise ValueError('counter RTL differs from simulated module')
for name,source in sources.items():shutil.copyfile(source,out/name)
bin=Path('E:/FPGA/2026.1/Vivado/bin')
def run(tool,args,log):
    with (out/log).open('x') as f:
        subprocess.run([str(bin/(tool+'.bat')),*args],cwd=out,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=120)
run('xvlog',['--sv',*sources],'compile.log')
for top,snapshot,expected in [('tb_metrics','metrics_sim','PASS metrics:'),('tb_control_metrics','control_metrics_sim','PASS control metrics:')]:
    run('xelab',[top,'-s',snapshot],snapshot+'_elaborate.log')
    log=snapshot+'_simulation.log';run('xsim',[snapshot,'-runall'],log)
    text=(out/log).read_text(errors='replace')
    if expected not in text or 'Fatal:' in text or 'ERROR:' in text:raise RuntimeError('RTL validation failed: '+log)
report=dict(passed=True,scope='exact packaged passive counter and control RTL; datapath C/RTL and board tests are separate',sha256={name:hashlib.sha256((out/name).read_bytes()).hexdigest() for name in sources})
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
