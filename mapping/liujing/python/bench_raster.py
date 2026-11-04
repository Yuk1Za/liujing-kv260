"""Matched native PS / true PL rendering of one fixed map and view sequence."""
import argparse,json,time,hashlib,os,fcntl
from pathlib import Path
import numpy as np
from native import Map,Camera
def main():
 p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('--bit',required=True);p.add_argument('--out',required=True);p.add_argument('--repeats',type=int,default=5);p.add_argument('--views',type=int,default=10);p.add_argument('--restore-bit',required=True);p.add_argument('--height',type=int,default=480);a=p.parse_args()
 if a.repeats<1 or a.views<1:raise ValueError('positive repeats/views required')
 cpu_mode=int(os.environ.get('LIUJING_CPU_RENDER_MODE','0'))
 if cpu_mode not in (0,4,5):raise ValueError('CPU numeric mode must be FP32 (0), Q18/Q12 (4), or qrcp_v1 (5)')
 lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a')
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from hardware_ownership import require_no_localization_clients
 require_no_localization_clients()
 # Same installed-board compatibility path as the accepted C16 verifier:
 # fpga_manager still programs the bitstream; skip incompatible XRT registration.
 import pynq.pl_server.embedded_device as ed
 ed.EmbeddedDevice._xrt_download=lambda self,data:None
 from pynq import Overlay,Clocks
 from accelerator import Raster
 scene=Path(a.scene);out=Path(a.out);out.mkdir(parents=True,exist_ok=False);model=scene/'surfels.npy';m=Map();m.restore(np.load(model,allow_pickle=False));cam=Camera.from_dict(json.loads((scene/'camera.json').read_text()));rows=[];r=None
 cam.cy+=(a.height-cam.height)/2;cam.height=a.height
 def ps(T):
  start=time.perf_counter();result=m.render(cam,T,cpu_mode);return (*result,(time.perf_counter()-start)*1000)
 os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
 try:
  ov=Overlay(a.bit);r=Raster(ov,surfel_capacity=max(4096,m.count),numeric_contract='qrcp_v1' if cpu_mode==5 else 'mixed');
  runtime_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (Path(m.lib._name),Path(__file__),Path(__file__).with_name('accelerator.py'),Path(__file__).with_name('native.py'))}
  m.render(cam,np.eye(4),cpu_mode);r.render(m,cam,np.eye(4)) # Explicit excluded warmup of both paths.
  for rep in range(a.repeats):
   for view in range(a.views):
    T=np.eye(4,dtype=np.float32);T[0,3]=.1*np.sin(view*.7);T[2,3]=-.1*(view%2)
    if (rep+view)%2:pl,pd,pi=r.render(m,cam,T);cpu,cd,ci,cpu_ms=ps(T)
    else:cpu,cd,ci,cpu_ms=ps(T);pl,pd,pi=r.render(m,cam,T)
    valid=(ci>=0)&(pi>=0);union=(ci>=0)|(pi>=0)
    mse=float(np.mean((cpu[valid].astype(float)-pl[valid].astype(float))**2)) if valid.any() else None
    depth99=float(np.percentile(abs(cd[valid]-pd[valid]),99)) if valid.any() else None
    thermal={str(f):float(f.read_text())/1000 for f in list(Path('/sys/class/thermal').glob('thermal_zone*/temp'))+list(Path('/sys/class/hwmon').glob('hwmon*/temp*_input'))};temps=list(thermal.values())
    rows.append({'repeat':rep,'view':view,'cpu_first':(rep+view)%2==0,'cpu_total_ms':cpu_ms,**r.timing,'max_thermal_zone_c':max(temps) if temps else None,'coverage_iou':float(valid.sum()/max(1,union.sum())),'id_agreement':float(np.mean(ci==pi)),'color_psnr_db':None if mse is None else (120.0 if mse==0 else float(10*np.log10(255**2/mse))),'depth_diff_p99_m':depth99})
    rows[-1]['valid_id_agreement']=float(np.mean(ci[valid]==pi[valid])) if valid.any() else None
    rows[-1]['valid_pixels']=int(valid.sum());rows[-1]['cpu_frequencies_khz']=[int(f.read_text()) for f in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')]
    mismatch=valid&(ci!=pi);delta=abs(cd[mismatch]-pd[mismatch]);rows[-1].update(temperatures_c=thermal,id_mismatch_count=int(mismatch.sum()),id_mismatch_depth_max_m=float(delta.max()) if len(delta) else 0.,id_mismatch_depth_over_10um=int(np.sum(delta>1e-5)))
    if cpu_mode>=4:
     reference,reference_d,reference_i=m.render(cam,T,0);qvalid=(reference_i>=0)&(pi>=0);qerr=float(np.mean((reference[qvalid].astype(float)-pl[qvalid].astype(float))**2)) if qvalid.any() else None
     rows[-1]['fp32_quality']={'id_mismatch':int(np.sum(reference_i!=pi)),'psnr_db':120. if qerr==0 else (float(10*np.log10(255**2/qerr)) if qerr is not None else None),'depth_max_m':float(abs(reference_d[qvalid]-pd[qvalid]).max()) if qvalid.any() else None}
    if rep==0 and view==0:
     from PIL import Image
     Image.fromarray(cpu).save(out/'cpu.png');Image.fromarray(pl).save(out/'pl.png')
  accepted=all(x['coverage_iou']>=.99 and x['color_psnr_db'] is not None and x['color_psnr_db']>=40 and x['depth_diff_p99_m']<=.01 for x in rows)
  summary={'scope':'same-map raster including PS projection/binning, transfer, PL, readback; excludes mapping and display encoding','cpu_threads':m.threads,'pl_clock_mhz':float(Clocks.fclk0_mhz),'fpga_manager_state':Path('/sys/class/fpga_manager/fpga0/state').read_text().strip(),'compatibility':'existing C16 fpga_manager path, XRT registration bypass','correctness_passed':accepted,'speedup':float(np.mean([x['cpu_total_ms'] for x in rows])/np.mean([x['total_ms'] for x in rows])),'map_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'bit_sha256':hashlib.sha256(Path(a.bit).read_bytes()).hexdigest(),'rows':rows}
  summary.update(surfels=m.count,render_size=[cam.width,cam.height],warmup_frames_excluded_per_backend=1,matched_pairs=len(rows),timing={k:{'mean':float(np.mean([x[k] for x in rows])),'p99':float(np.percentile([x[k] for x in rows],99)),'max':float(max(x[k] for x in rows))} for k in ('cpu_total_ms','prepare_ms','transfer_submit_ms','pl_wait_ms','readback_convert_ms','total_ms')},process_memory=[line for line in Path('/proc/self/status').read_text().splitlines() if line.startswith(('VmRSS:','VmHWM:'))])
  summary['image_depth_correctness_passed']=accepted;summary['surfel_ids_exact']=all(x['id_agreement']==1 for x in rows);summary['correctness_passed']=accepted and summary['surfel_ids_exact']
  summary['precision']='FP32 geometry, Q18 weights/Q12 colors/uint64 sums in both paths' if cpu_mode==4 else 'FP32 in both paths; exact-ID mismatch is reported independently of image/depth thresholds'
  if cpu_mode==5:summary['precision']='qrcp_v1: exact FP32 owner path, reciprocal color weights/normalization, Q18/Q12 integer sums in both paths'
  summary['runtime_sha256']=runtime_hashes;summary['cpu_render_mode']=cpu_mode
  (out/'report.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='rows'}))
 finally:
  if a.restore_bit:
   Overlay(a.restore_bit)
   (out/'restored.json').write_text(json.dumps({'bit':a.restore_bit,'sha256':hashlib.sha256(Path(a.restore_bit).read_bytes()).hexdigest(),'manager_state':Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()}))
   if r:r.failed=False
  if r:r.close()
  m.close()
if __name__=='__main__':main()
