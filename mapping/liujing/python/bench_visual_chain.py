"""Matched board-only visual chain. Real RGB-D, synthetic stationary trajectory.

This isolates compute; it is NOT moving SLAM, capture latency or calibration acceptance.
Both backends consume every input and produce two independently encoded views.
"""
import argparse,fcntl,hashlib,io,json,os,time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from native import Map,Camera
from alignment import register_color,checked_pose
from viewer import encode_frame,compose_trajectory

def summary(x):return dict(mean=float(np.mean(x)),p99=float(np.percentile(x,99)),max=float(max(x)))
def encode_view(im,poses,view,viewcam,submitted):
 started=time.perf_counter()
 canvas=compose_trajectory(im,poses,view,viewcam,'Stationary replay / synthetic identity pose')
 jpeg=encode_frame(canvas);finished=time.perf_counter()
 return jpeg,(finished-started)*1000,(started-submitted)*1000,finished

def run(m,r,backend,raw,cal,cam,viewcam,frames,rep,out,views_per_frame=2,cpu_mode=4,encode_pipeline=False):
 if encode_pipeline:
  if views_per_frame!=2:raise ValueError('bounded encoding experiment requires exactly two views per fusion')
  # Only two jobs may be submitted and both finish before the next input.
  # Apply the identical scheduler to CPU and PL, retaining four CPU render threads.
  with ThreadPoolExecutor(max_workers=1) as encoder:
   encoder.submit(time.perf_counter).result()
   return _run(m,r,backend,raw,cal,cam,viewcam,frames,rep,out,views_per_frame,cpu_mode,encoder)
 return _run(m,r,backend,raw,cal,cam,viewcam,frames,rep,out,views_per_frame,cpu_mode,None)

def _run(m,r,backend,raw,cal,cam,viewcam,frames,rep,out,views_per_frame,cpu_mode,encoder):
 rows=[];last=None;run_started=time.perf_counter()
 for frame in range(frames):
  start=time.perf_counter();d,rgb=register_color(*raw[frame%len(raw)],cal,native_library=m.lib);registered=time.perf_counter();T=np.eye(4,dtype=np.float32);version=m.version
  _,ad,ai=r.render(m,cam,T,3) if backend=='pl' else m.render(cam,T,3);associated=time.perf_counter();association_stages=dict(r.timing) if backend=='pl' else None
  m.integrate(d,rgb,cam,T,1+frame*.1,association=(ad,ai,version));fused=time.perf_counter()
  render_ms=encode_ms=encode_queue_ms=0;views=[];pending=[]
  for browse in range(views_per_frame):
   index=frame*views_per_frame+browse;view=T.copy();view[0,3]=.1*np.sin(index*.7);view[2,3]=-.1*(index%2);vstart=time.perf_counter()
   im,vd,vi=r.render(m,viewcam,view) if backend=='pl' else m.render(viewcam,view,cpu_mode);rendered=time.perf_counter()
   # Same board-side trajectory compositor and encoder as the live path.
   args=(im,[T[:3,3].copy()]*(frame+1),view.copy(),viewcam,rendered)
   task=encoder.submit(encode_view,*args) if encoder else encode_view(*args)
   pending.append(task);render_ms+=(rendered-vstart)*1000
   views.append(dict(index=index,depth_sha256=hashlib.sha256(vd.tobytes()).hexdigest(),id_sha256=hashlib.sha256(vi.tobytes()).hexdigest(),stages=dict(r.timing) if backend=='pl' else None))
  encoded=fused
  for entry,task in zip(views,pending):
   jpeg,duration,queue_age,ready=task.result() if encoder else task
   encode_ms+=duration;encode_queue_ms+=queue_age;encoded=max(encoded,ready)
   entry.update(jpeg_bytes=len(jpeg),jpeg_sha256=hashlib.sha256(jpeg).hexdigest(),encode_queue_ms=queue_age,compose_encode_ms=duration)
  rows.append(dict(frame=frame,backend=backend,repeat=rep,registration_ms=(registered-start)*1000,association_ms=(associated-registered)*1000,fusion_ms=(fused-associated)*1000,render_ms=render_ms,compose_encode_ms=encode_ms,total_ms=(encoded-start)*1000,elapsed_run_s=encoded-run_started,surfels=m.count,map_version=m.version,views=views,map_sha256=hashlib.sha256(m.surfels().tobytes()).hexdigest()))
  rows[-1].update(encode_pipeline=encoder is not None,encode_queue_ms=encode_queue_ms,source_index=frame%len(raw),association_id_sha256=hashlib.sha256(ai.tobytes()).hexdigest(),association_depth_sha256=hashlib.sha256(ad.tobytes()).hexdigest(),association_stages=association_stages,temperatures_c={str(f):int(f.read_text())/1000 for f in Path('/sys/class/hwmon').glob('hwmon*/temp*_input')},memory=[line for line in Path('/proc/self/status').read_text().splitlines() if line.startswith(('VmRSS:','VmHWM:'))])
  if frame==frames-1:Image.fromarray(im).save(out/f'{backend}_{rep}.png');last=(vd.copy(),vi.copy(),im.copy(),m.surfels())
 return rows,last

def main():
 p=argparse.ArgumentParser();p.add_argument('--capture',default='runs/camera_02');p.add_argument('--map',default='runs/camera_preview_04/surfels.npy');p.add_argument('--out',required=True);p.add_argument('--bit',required=True);p.add_argument('--restore-bit',required=True);p.add_argument('--frames',type=int,default=20);p.add_argument('--repeats',type=int,default=5);p.add_argument('--cpu-only',action='store_true');p.add_argument('--encode-pipeline',action='store_true');p.add_argument('--numeric-contract',choices=['mixed','qrcp_v1'],default='mixed');a=p.parse_args();cpu_mode=5 if a.numeric_contract=='qrcp_v1' else 4
 if a.frames<1 or a.repeats<1:raise ValueError('positive frame/repeat counts')
 out=Path(a.out);out.mkdir(parents=True,exist_ok=False);root=Path(a.capture);cal=json.loads((root/'factory_calibration.json').read_text());T=np.eye(4);T[:3,:3]=np.array(cal['rotation_raw']).reshape(3,3);T[:3,3]=np.array(cal['translation_raw'])*.001;cal['T_color_depth']=checked_pose(T).tolist()
 pairs=sorted(root.glob('*_depth.png'),key=lambda p:int(p.stem.split('_')[0]));raw=[(np.asarray(Image.open(p),np.float32)*.001,np.asarray(Image.open(p.with_name(p.name.replace('_depth','_rgb'))).convert('RGB'))) for p in pairs]
 if not raw:raise ValueError('no paired capture')
 initial=np.load(a.map,allow_pickle=False);cam=Camera.from_dict(cal['depth']);viewcam=Camera(640,480,cam.fx,cam.fy,cam.cx,cam.cy+40,cam.near_z,cam.far_z)
 if a.cpu_only:
  m=Map();m.restore(initial)
  try:rows,_=run(m,None,'cpu',raw,cal,cam,viewcam,a.frames,0,out,cpu_mode=cpu_mode,encode_pipeline=a.encode_pipeline)
  finally:m.close()
  report=dict(scope='CPU profile only, repeated real RGB-D and synthetic stationary identity pose',rows=rows,timing={key:summary([x[key] for x in rows]) for key in ('registration_ms','association_ms','fusion_ms','render_ms','compose_encode_ms','total_ms')});(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['timing']));return
 lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from hardware_ownership import require_no_localization_clients
 require_no_localization_clients()
 import pynq.pl_server.embedded_device as ed
 ed.EmbeddedDevice._xrt_download=lambda self,data:None
 os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
 from pynq import Overlay,Clocks
 from accelerator import Raster
 r=None;rows=[];checks=[]
 try:
  r=Raster(Overlay(a.bit),surfel_capacity=1000000,numeric_contract=a.numeric_contract)
  for rep in range(a.repeats):
   final={}
   for backend in (('cpu','pl') if rep%2==0 else ('pl','cpu')):
    m=Map();m.restore(initial)
    try:
     # Warm kernels but do not mutate the starting map.
     register_color(*raw[0],cal,native_library=m.lib);m.render(viewcam,np.eye(4),cpu_mode);r.render(m,viewcam,np.eye(4))
     batch,last=run(m,r,backend,raw,cal,cam,viewcam,a.frames,rep,out,cpu_mode=cpu_mode,encode_pipeline=a.encode_pipeline);rows.extend(batch);final[backend]=last
     print(json.dumps(dict(repeat=rep,backend=backend,total_ms=summary([x['total_ms'] for x in batch]))),flush=True)
    finally:m.close()
   cd,ci,cr,cm=final['cpu'];pd,pi,pr,pm=final['pl'];valid=(ci>=0)&(pi>=0);mse=float(np.mean((cr[valid].astype(float)-pr[valid].astype(float))**2)) if valid.any() else None
   checks.append(dict(repeat=rep,id_mismatch=int(np.sum(ci!=pi)),depth_max_m=float(abs(cd-pd).max()),map_exact=bool(np.array_equal(cm,pm)),color_psnr_db=120. if mse==0 else (float(10*np.log10(255**2/mse)) if mse is not None else None)))
  timings={b:{key:summary([x[key] for x in rows if x['backend']==b]) for key in ('registration_ms','association_ms','fusion_ms','render_ms','compose_encode_ms','total_ms')} for b in ('cpu','pl')}
  report=dict(scope='paired input ready through registration, association, fusion, new-view render, composition and JPEG ready; excludes acquisition/pose wait/network',workload='repeated real RGB-D frame, synthetic stationary identity pose; NOT moving room acceptance',numeric='FP32 geometry, Q18 weight/Q12 color/uint64 accumulation, same CPU/PL',timing=timings,speedup=timings['cpu']['total_ms']['mean']/timings['pl']['total_ms']['mean'],checks=checks,rows=rows,bit_sha256=hashlib.sha256(Path(a.bit).read_bytes()).hexdigest(),initial_map_sha256=hashlib.sha256(Path(a.map).read_bytes()).hexdigest(),input_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ('0_depth.png','0_rgb.png','factory_calibration.json')})
  paired={(x['repeat'],x['frame'],x['backend']):x for x in rows}
  report.update(numeric_contract=a.numeric_contract,cpu_render_mode=cpu_mode,cpu_threads=m.threads,encode_pipeline=a.encode_pipeline,encode_scheduler="one worker, at most two jobs, drained per fusion frame" if a.encode_pipeline else "sequential",stage_timing_note="render and encoding wall times overlap when enabled; total_ms measures paired-input-ready to both encoded views ready")
  if cpu_mode==5:report['numeric']='qrcp_v1: exact FP32 owners, reciprocal color weights/normalization with Q18/Q12 integer sums in both backends'
  report['workload']='cyclic replay of all '+str(len(raw))+' real RGB-D pairs; synthetic stationary identity pose; NOT moving room acceptance'
  report['input_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'factory_calibration.json',*pairs,*[p.with_name(p.name.replace('_depth','_rgb')) for p in pairs]]}
  all_maps=all(paired[(rep,f,'cpu')]['map_sha256']==paired[(rep,f,'pl')]['map_sha256'] for rep in range(a.repeats) for f in range(a.frames))
  all_ids=all([v['id_sha256'] for v in paired[(rep,f,'cpu')]['views']]==[v['id_sha256'] for v in paired[(rep,f,'pl')]['views']] for rep in range(a.repeats) for f in range(a.frames))
  all_associations=all(paired[(rep,f,'cpu')]['association_id_sha256']==paired[(rep,f,'pl')]['association_id_sha256'] for rep in range(a.repeats) for f in range(a.frames))
  report.update(browse_frames_per_fusion=2,all_map_versions_exact=all_maps,all_browse_ids_exact=all_ids,wall_throughput={b:{'map_hz':a.frames*a.repeats/sum(x['elapsed_run_s'] for x in rows if x['backend']==b and x['frame']==a.frames-1),'browse_fps':2*a.frames*a.repeats/sum(x['elapsed_run_s'] for x in rows if x['backend']==b and x['frame']==a.frames-1)} for b in ('cpu','pl')})
  report['all_association_ids_exact']=all_associations
  report['all_association_depths_exact']=all(paired[(rep,f,'cpu')]['association_depth_sha256']==paired[(rep,f,'pl')]['association_depth_sha256'] for rep in range(a.repeats) for f in range(a.frames))
  for label,key in [('all_browse_depths_exact','depth_sha256'),('all_encoded_views_exact','jpeg_sha256')]:
   report[label]=all([v[key] for v in paired[(rep,f,'cpu')]['views']]==[v[key] for v in paired[(rep,f,'pl')]['views']] for rep in range(a.repeats) for f in range(a.frames))
  report['runtime_sha256']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (Path(os.environ.get('LIUJING_LIBRARY','build/libliujing.so')),Path(__file__),Path(__file__).with_name('accelerator.py'),Path(__file__).with_name('alignment.py'))}
  report['environment']=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),pl_clock_mhz=float(Clocks.fclk0_mhz),cpu_frequency_khz={str(p):int(p.read_text()) for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')},cpu_governor={str(p):p.read_text().strip() for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor')})
  report['correctness_passed']=all_maps and all_ids and all_associations and all(x['id_mismatch']==0 and x['map_exact'] and x['color_psnr_db'] is not None and x['color_psnr_db']>=40 for x in checks)
  report['correctness_passed'] &= report['all_association_depths_exact'] and report['all_browse_depths_exact'] and report['all_encoded_views_exact']
  (out/'report.json').write_text(json.dumps(report,indent=2));print('SUMMARY '+json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
 finally:
  Overlay(a.restore_bit);(out/'restored.json').write_text(json.dumps(dict(bit=a.restore_bit,sha256=hashlib.sha256(Path(a.restore_bit).read_bytes()).hexdigest())))
  if r:r.failed=False;r.close()
if __name__=='__main__':main()
