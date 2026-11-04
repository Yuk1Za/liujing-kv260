"""Exclusive, one-shot full-map hardware parity run; no sensors started."""
import argparse,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path
from fusion_allocation import allocate_fusion
from hardware_fault import require_healthy,latch_timeout
from hardware_ownership import require_no_localization_clients
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('artifact',type=Path);p.add_argument('capture',type=Path);p.add_argument('output',type=Path);p.add_argument('--verify',action='store_true');p.add_argument('--diagnostic-probe',action='store_true');a=p.parse_args()
if a.verify and a.diagnostic_probe:raise ValueError('probe and map validation are separate modes')
digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
require_healthy();lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);own=open(ROOT/'live.lock','a');fcntl.flock(own,fcntl.LOCK_EX|fcntl.LOCK_NB);require_no_localization_clients()
manifest=json.loads((a.artifact/'manifest.json').read_text());bit=a.artifact/'liujing.bit'
if str(manifest.get('board_validation','')).startswith('FAILED') and not a.diagnostic_probe:raise RuntimeError('failed artifact is restricted to explicit diagnostic probes')
if manifest.get('contract')!='fusion_association_v1' or not manifest.get('routed_timing_passed') or not manifest.get('rtl_cosim_passed'):raise RuntimeError('unverified association artifact')
for extension in ('bit','hwh','xclbin'):
    if digest(a.artifact/('liujing.'+extension))!=manifest[extension+'_sha256']:raise RuntimeError(extension+' hash mismatch')
import pynq.pl_server.embedded_device as ed
ed.EmbeddedDevice._xrt_download=lambda self,data:None
from pynq import Overlay,Clocks
overlay=Overlay(str(bit))
if abs(Clocks.fclk0_mhz-100)>.1 or 'lj_associate_0' not in overlay.ip_dict or 'fpga_feature_extract_0' not in overlay.ip_dict:raise RuntimeError('association platform mismatch')
a.output.parent.mkdir(parents=True,exist_ok=True);buffers,allocation=allocate_fusion(overlay,a.output.parent,bit)
import hardware_ownership
hardware_ownership._reconfiguration_lease.close();hardware_ownership._reconfiguration_lease=None
env=os.environ.copy();env['LIUJING_FUSION_PL']=str(allocation)
result=None
def snapshot_axi_watch(name):
    if 'debug_gpio' not in overlay.ip_dict:return
    from pynq import MMIO
    gpio=MMIO(overlay.ip_dict['debug_gpio']['phys_addr'],0x1000)
    gpio.write(4,0);gpio.write(12,0xffffffff)
    samples={}
    for selector in range(51):
        gpio.write(0,selector)
        samples[str(selector)]=gpio.read(8)
    (a.output.parent/name).write_text(json.dumps({'selectors':samples,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'artifact_bit_sha256':manifest['bit_sha256'],'output':str(a.output),'monotonic_ns':time.monotonic_ns()},indent=2))
try:
    cmd=([str(ROOT/'native_fusion_v3_build/fusion_probe'),str(a.capture),str(a.output)] if a.diagnostic_probe else [str(ROOT/'native_fusion_v3_build/fusion_replay'),'pl-check' if a.verify else 'pl',str(a.capture),str(a.output)])
    result=subprocess.run(cmd,env=env,pass_fds=tuple(b.native_export_fd for b in buffers),timeout=300)
finally:
    active=not (overlay.lj_associate_0.read(0)&4)
    if active:latch_timeout({'reason':'association replay exited with active DMA','output':str(a.output)})
    if active or Path('/run/lock/liujing_raster_fault.json').exists():
        # Diagnostic partial outputs are never published as a map. Preserve
        # them while the owning process still pins every DMA allocation.
        try:
            import numpy as np
            (a.output.parent/'fault_registers.json').write_text(json.dumps({'registers':{hex(i):hex(overlay.lj_associate_0.read(i)) for i in range(0,128,4)},'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()},indent=2))
            snapshot_axi_watch('fault_axi_watch.json')
            for name,b in zip(('table','groups','queries','matches','stats'),buffers):
                if name!='table':np.asarray(b).tofile(a.output.parent/('fault_'+name+'.bin'))
            (a.output.parent/'fault_latch.json').write_bytes(Path('/run/lock/liujing_raster_fault.json').read_bytes())
            kernel=subprocess.run(['dmesg','--ctime'],capture_output=True,timeout=5)
            (a.output.parent/'kernel_fault.log').write_bytes(kernel.stdout+kernel.stderr)
        except Exception as error:print('Fault snapshot failed; CMA remains pinned: '+str(error),flush=True)
        print('Association fault: retaining CMA until reboot',flush=True)
        while True:time.sleep(1)
if result is None or result.returncode:raise SystemExit(result.returncode if result else 1)
snapshot_axi_watch('completed_axi_watch.json')
if a.diagnostic_probe:print(json.dumps({'output':str(a.output)+'.jsonl','scope':'empty-table diagnostic probe only; no map or live acceptance'}))
else:print(json.dumps({'output':str(a.output)+'.bin','sha256':digest(Path(str(a.output)+'.bin')),'contract':'fusion_association_v1','verify_each_query':a.verify,'scope':'same-input on-board replay; no live acceptance'}))
