"""True concurrent C16/raster DDR stress, using recorded data, not live SLAM."""
import argparse,fcntl,hashlib,json,os,struct,time,threading
from pathlib import Path
import numpy as np
from native import Map,Camera

def digest(b):return hashlib.sha256(np.asarray(b).tobytes()).hexdigest()
def summary(values):return {'count':len(values),'mean':float(np.mean(values)),'p99':float(np.percentile(values,99)),'max':float(max(values))} if values else None
def main():
 p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('--bit',required=True);p.add_argument('--restore-bit',required=True);p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=30);p.add_argument('--lidar-only',action='store_true');p.add_argument('--numeric-contract',choices=['mixed','qrcp_v1'],default='mixed');a=p.parse_args()
 if not 1<=a.seconds<=3600:raise ValueError('duration')
 root=Path('/home/ubuntu/losam_live_20261021');lock=open(root/'pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from hardware_ownership import require_no_localization_clients
 require_no_localization_clients()
 out=Path(a.out);out.mkdir(parents=True,exist_ok=False);scene=Path(a.scene);m=Map();m.restore(np.load(scene/'surfels.npy'));cam=Camera.from_dict(json.loads((scene/'camera.json').read_text()));cam.cy+=(480-cam.height)/2;cam.height=480
 import pynq.pl_server.embedded_device as ed
 ed.EmbeddedDevice._xrt_download=lambda self,data:None
 from pynq import Overlay,allocate
 from accelerator import Raster
 os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
 buffers={};r=None;thread=None;stop=threading.Event();lidar=[];render=[];errors=[]
 try:
  ov=Overlay(a.bit);ip=ov.fpga_feature_extract_0;rm=ip.register_map;r=None if a.lidar_only else Raster(ov,surfel_capacity=max(m.count,4096),numeric_contract=a.numeric_contract)
  for key,n,dtype,dim in [('input',115200,'<f4',4),('point',28800,'<f4',4),('range',28800,'<f4',1),('lessflat',28800,'u1',1),('ground',28800,'u1',1),('flat',384,'<i2',2),('sharp',192,'<i2',2),('lesssharp',1920,'<i2',2)]:
   b=allocate(shape=(n,dim) if dim>1 else (n,),dtype=dtype);buffers[key]=b;reg='laserCloudInArray' if key=='input' else 'rangeimage_'+key;setattr(rm,reg+'_1',int(b.physical_address)&0xffffffff);setattr(rm,reg+'_2',int(b.physical_address)>>32)
  data=np.load(root/'captured_c16.npz');points=data['points'].astype('<f4');keep=np.isfinite(points).all(1)&(np.linalg.norm(points[:,:3],axis=1)>=.7)&(abs(points[:,:3]).max(1)<125);points=points[keep];points=points[np.argsort(np.arctan2(points[:,1],points[:,0]),kind='stable')]
  expected=json.loads((root/'c16_acceptance.json').read_text())['trials'][0]['hashes']
  def worker():
   deadline=time.monotonic();last=None
   try:
    while not stop.is_set():
     start=time.monotonic();buffers['input'][:len(points)]=points;buffers['input'][len(points)]=points[-1];buffers['input'].flush()
     for k,b in buffers.items():
      if k!='input':b[:]=0;b.flush()
     rm.cloudSize=len(points)+1;rm.start_point_ori=struct.unpack('<I',struct.pack('<f',-np.pi))[0];rm.end_point_ori=struct.unpack('<I',struct.pack('<f',np.pi))[0]
     launch=time.monotonic();ip.write(0,1)
     while not ip.read(0)&2:
      if time.monotonic()-launch>5:raise TimeoutError('C16 AP_DONE timeout')
      time.sleep(.00005)
     finish=time.monotonic()
     for k,b in buffers.items():
      if k!='input':b.invalidate()
     actual={k:digest(b) for k,b in buffers.items() if k!='input'}
     if actual!=expected:raise RuntimeError('Concurrent C16 output hash mismatch')
     lidar.append({'pl_ms':(finish-launch)*1000,'total_with_hash_ms':(time.monotonic()-start)*1000,'start_lateness_ms':max(0,start-deadline)*1000,'start_interval_ms':None if last is None else (start-last)*1000,'all_hashes_equal':True});last=start;deadline+=.2
     stop.wait(max(0,deadline-time.monotonic()))
   except Exception as exc:errors.append(repr(exc));stop.set()
  thread=threading.Thread(target=worker);thread.start();start=time.monotonic();i=0
  while time.monotonic()-start<a.seconds and not stop.is_set():
   if a.lidar_only:stop.wait(min(.1,max(0,a.seconds-(time.monotonic()-start))));continue
   T=np.eye(4);T[0,3]=.1*np.sin(i*.7);image,depth,ids=r.render(m,cam,T);render.append(dict(r.timing));i+=1
  stop.set();thread.join(timeout=6)
  if thread.is_alive():raise RuntimeError('C16 worker did not terminate')
  report={'scope':'recorded C16 at 5 Hz and PL raster concurrently; no ROS localization/fusion or real-time room acceptance','bit_sha256':digest(np.frombuffer(Path(a.bit).read_bytes(),np.uint8)),'seconds':time.monotonic()-start,'errors':errors,'lidar':lidar,'raster':render,'lidar_pl_ms':summary([x['pl_ms'] for x in lidar]),'lidar_start_interval_ms':summary([x['start_interval_ms'] for x in lidar if x['start_interval_ms'] is not None]),'raster_total_ms':summary([x['total_ms'] for x in render])};(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('lidar','raster')}))
  report.update(lidar_only=a.lidar_only,numeric_contract=a.numeric_contract)
  if a.lidar_only:report['scope']='recorded C16 at 5 Hz alone on the same integrated bitstream; not ROS localization'
  (out/'report.json').write_text(json.dumps(report,indent=2))
  if errors:raise RuntimeError(errors)
 finally:
  stop.set()
  if thread:thread.join(timeout=6)
  Overlay(a.restore_bit)
  if r:r.failed=False;r.close()
  for b in buffers.values():b.freebuffer()
  m.close()
  (out/'restored.json').write_text(json.dumps({'sha256':hashlib.sha256(Path(a.restore_bit).read_bytes()).hexdigest(),'manager_state':Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()}))
if __name__=='__main__':main()
