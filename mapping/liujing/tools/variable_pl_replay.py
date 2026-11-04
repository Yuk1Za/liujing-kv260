"""Exclusive CPU or variable-candidate PL replay, with faults pinned until reboot."""
import argparse,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path
from fusion_allocation import allocate_fusion
from hardware_fault import require_healthy,latch_timeout
from hardware_ownership import require_no_localization_clients
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('artifact',type=Path);p.add_argument('capture',type=Path);p.add_argument('output',type=Path);p.add_argument('--mode',choices=['cpu','pl','pl-check'],required=True);p.add_argument('--binary',type=Path);p.add_argument('--rgbd-calibration',type=Path);p.add_argument('--fast-prepare',action='store_true');p.add_argument('--schedule',choices=['serial','stream'],default='serial');a=p.parse_args()
if a.schedule=='stream' and not a.rgbd_calibration:raise ValueError('stream schedule requires raw RGB-D calibration')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
require_healthy();lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
own=open(ROOT/'live.lock','a');fcntl.flock(own,fcntl.LOCK_EX|fcntl.LOCK_NB);require_no_localization_clients()
a.output.parent.mkdir(parents=True,exist_ok=True)
manifest=json.loads((a.artifact/'manifest.json').read_text());bit=a.artifact/'liujing.bit'
if manifest.get('contract') not in ('fusion_variable_v1','fusion_cached_v2','fusion_resident_v4') or not manifest.get('routed_timing_passed') or not manifest.get('rtl_cosim_passed') or not manifest.get('axi_handshake_boundary_passed'):raise RuntimeError('unverified variable hardware artifact')
if str(manifest.get('board_validation','')).startswith('FAILED'):raise RuntimeError('failed variable artifact')
if a.mode=='pl' and not str(manifest.get('board_validation','')).startswith('PASSED_SAME_INPUT_'):raise RuntimeError('complete parity before timed PL runs')
if a.schedule=='stream' and a.mode!='cpu' and not str(manifest.get('board_validation','')).startswith('PASSED_SAME_INPUT_'):raise RuntimeError('complete serial query/map parity before stream qualification')
if a.schedule=='stream' and a.mode=='pl' and manifest.get('stream_validation')!='PASSED_ALL_COMMITTED_PACKETS':raise RuntimeError('complete stream packet parity before throughput measurements')
for ext in ('bit','hwh','xclbin'):
    if sha(a.artifact/('liujing.'+ext))!=manifest[ext+'_sha256']:raise RuntimeError(ext+' digest mismatch')
binary=a.binary or ROOT/('native_fusion_dataflow_build/pipeline_stream_replay' if a.schedule=='stream' else 'native_fusion_dataflow_build/pipeline_replay' if manifest['contract']=='fusion_resident_v4' and a.rgbd_calibration else 'native_pipeline_build/pipeline_replay' if a.rgbd_calibration else 'native_fusion_cached_build/variable_replay' if manifest['contract']=='fusion_cached_v2' else 'native_fusion_variable_build/variable_replay')
required='required_stream_binary_sha256' if a.schedule=='stream' else 'required_binary_sha256'
if manifest.get(required) and sha(binary)!=manifest[required]:raise RuntimeError('artifact requires the validated driver binary')
env=os.environ.copy();env['LIUJING_FUSION_FAST_PREPARE']='1' if a.fast_prepare else '0';env.update(OMP_NUM_THREADS='2',OMP_THREAD_LIMIT='2',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0',OPENBLAS_NUM_THREADS='1')
buffers=[];overlay=None
if a.mode!='cpu':
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    from pynq import Overlay,Clocks
    overlay=Overlay(str(bit))
    if abs(Clocks.fclk0_mhz-100)>.1 or 'lj_associate_0' not in overlay.ip_dict or 'fpga_feature_extract_0' not in overlay.ip_dict:raise RuntimeError('variable platform mismatch')
    buffers,allocation=allocate_fusion(overlay,a.output.parent,bit,manifest['contract']);env['LIUJING_FUSION_PL']=str(allocation)
import hardware_ownership
hardware_ownership._reconfiguration_lease.close();hardware_ownership._reconfiguration_lease=None
inputs={f.name:sha(f) for f in sorted(a.capture.glob('*.points'))}
if not inputs:raise RuntimeError('empty capture')
proof={'mode':a.mode,'capture':str(a.capture),'input_sha256':inputs,'binary_sha256':sha(binary),'bit_sha256':manifest['bit_sha256'],'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'thread_environment':{k:env[k] for k in ('OMP_NUM_THREADS','OMP_THREAD_LIMIT','OMP_WAIT_POLICY','GOMP_SPINCOUNT','LIUJING_FUSION_FAST_PREPARE')},'uname':os.uname().release,'start_mono':time.monotonic(),'timing_scope':'CPU: group and lookup + association; PL: grouping + packing/DMA + execute/scatter + incremental index synchronization; validation runs excluded from timing comparisons'}
proof['schedule']=a.schedule
if a.schedule=='stream':proof['timing_scope']='Memory-resident raw RGB-D through all committed delta encodings, with identical two-entry queues and pipeline fill/drain. No disk, sensors, online poses, network or browser. Stream check compares all canonical packets against CPU after serial query/map qualification.'
for name in ('scaling_governor','scaling_cur_freq'):
    f=Path('/sys/devices/system/cpu/cpu0/cpufreq')/name
    if f.exists():proof[name]=f.read_text().strip()
def thermal_snapshot():
    result={}
    for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
        try:result[zone.name]={'type':(zone/'type').read_text().strip(),'millidegrees_c':int((zone/'temp').read_text())}
        except (OSError,ValueError):pass
    for path in Path('/sys/class/hwmon').glob('hwmon*/temp*_input'):
        try:result[str(path)]={'millidegrees_c':int(path.read_text())}
        except (OSError,ValueError):pass
    return result
proof['thermal_start']=thermal_snapshot()
if a.rgbd_calibration:
    proof['raw_input_sha256']={f.with_suffix(ext).name:sha(f.with_suffix(ext)) for f in sorted(a.capture.glob('*.points')) for ext in ('.jpg','.depth','.json')}
    proof['calibration_sha256']=sha(a.rgbd_calibration)
    proof['pipeline_scope']='Raw saved JPEG/depth and recorded pose to committed delta encoding. File I/O, live acquisition/pose estimation, network and browser excluded.'
Path(str(a.output)+'_provenance.json').write_text(json.dumps(proof,indent=2))
result=None
try:
    with Path(str(a.output)+'.log').open('w') as log:
        run_mode='pl' if a.schedule=='stream' and a.mode=='pl-check' else a.mode
        result=subprocess.run([str(binary),run_mode,str(a.capture),str(a.output)]+([str(a.rgbd_calibration)] if a.rgbd_calibration else []),env=env,pass_fds=tuple(b.native_export_fd for b in buffers),stdout=log,stderr=subprocess.STDOUT,timeout=900)
finally:
    active=overlay is not None and not (overlay.lj_associate_0.read(0)&4)
    if active:latch_timeout({'reason':'variable association replay left DMA active','output':str(a.output)})
    if active or Path('/run/lock/liujing_raster_fault.json').exists():
        if overlay is not None:
            Path(str(a.output)+'_fault_registers.json').write_text(json.dumps({hex(i):hex(overlay.lj_associate_0.read(i)) for i in range(0,160,4)},indent=2))
        print('Variable association fault: retaining DMA buffers until reboot',flush=True)
        while True:time.sleep(1)
proof.update(end_mono=time.monotonic(),exit_code=result.returncode if result else None)
proof['thermal_end']=thermal_snapshot()
frequency=Path('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq')
if frequency.exists():proof['scaling_cur_freq_end']=frequency.read_text().strip()
if result is not None and result.returncode==0:proof['map_sha256']=sha(Path(str(a.output)+'.bin'))
if result is not None and result.returncode==0 and a.schedule=='stream':proof['committed_packets_sha256']=sha(Path(str(a.output)+'.packets'))
Path(str(a.output)+'_provenance.json').write_text(json.dumps(proof,indent=2));print(json.dumps({k:v for k,v in proof.items() if k!='input_sha256'},indent=2))
raise SystemExit(result.returncode if result else 1)
