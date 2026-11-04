"""Exclusive structural probe of a qualified resident artifact; no speedup claim."""
import argparse,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path
from fusion_allocation import allocate_fusion
from hardware_fault import require_healthy,latch_timeout
from hardware_ownership import require_no_localization_clients
import hardware_ownership
p=argparse.ArgumentParser();p.add_argument('artifact',type=Path);p.add_argument('binary',type=Path);p.add_argument('trace',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
require_healthy()
locks=[open('/home/ubuntu/losam_live_20261021/pipeline.lock','a'),open('/home/ubuntu/liujing/live.lock','a')]
for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
require_no_localization_clients()
m=json.loads((a.artifact/'manifest.json').read_text())
if m['contract']!='fusion_resident_v4' or not m['board_validation'].startswith('PASSED_SAME_INPUT_'):raise RuntimeError('qualified resident artifact required')
for ext in ('bit','hwh','xclbin'):
    if sha(a.artifact/('liujing.'+ext))!=m[ext+'_sha256']:raise RuntimeError('artifact changed')
import pynq.pl_server.embedded_device as ed
ed.EmbeddedDevice._xrt_download=lambda self,data:None
from pynq import Overlay,Clocks
overlay=Overlay(str(a.artifact/'liujing.bit'))
if abs(Clocks.fclk0_mhz-100)>.1:raise RuntimeError('clock differs')
a.output.parent.mkdir(parents=True,exist_ok=True)
buffers,allocation=allocate_fusion(overlay,a.output.parent,a.artifact/'liujing.bit',m['contract'])
hardware_ownership._reconfiguration_lease.close();hardware_ownership._reconfiguration_lease=None
proof={'scope':'Structural diagnostic with a reconstructed resident index, not a pipeline benchmark; zero/one/eight/original queries per real group.','binary_sha256':sha(a.binary),'trace_sha256':sha(a.trace),'bit_sha256':m['bit_sha256'],'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'clock_mhz':Clocks.fclk0_mhz}
env=os.environ.copy();env['LIUJING_FUSION_PL']=str(allocation)
try:
    with Path(str(a.output)+'.log').open('w') as log:r=subprocess.run([str(a.binary),str(a.trace),str(a.output)+'.jsonl'],env=env,pass_fds=tuple(b.native_export_fd for b in buffers),stdout=log,stderr=subprocess.STDOUT,timeout=120)
finally:
    active=not(overlay.lj_associate_0.read(0)&4)
    if active:latch_timeout({'reason':'resident structural probe left DMA active'})
    if active or Path('/run/lock/liujing_raster_fault.json').exists():
        print('Hardware fault: retaining DMA until reboot',flush=True)
        while True:time.sleep(1)
proof['exit_code']=r.returncode;Path(str(a.output)+'_provenance.json').write_text(json.dumps(proof,indent=2));print(json.dumps(proof,indent=2));raise SystemExit(r.returncode)
