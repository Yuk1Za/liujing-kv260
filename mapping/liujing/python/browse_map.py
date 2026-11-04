"""Interactive board renderer for a saved map; viewer transports only 2-D JPEG."""
import argparse,json,time,threading,math
from pathlib import Path
import numpy as np
from native import Map,Camera
from viewer import FrameServer
def main():
 p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8090);p.add_argument('--seconds',type=float,default=0);a=p.parse_args()
 scene=Path(a.scene);m=Map();m.restore(np.load(scene/'surfels.npy',allow_pickle=False));camera=Camera.from_dict(json.loads((scene/'camera.json').read_text()));lock=threading.Lock();state={'T':np.eye(4),'mode':0,'revision':0}
 def control(v):
  with lock:
   if 'mode' in v:
    if v['mode']=='follow':state['T']=np.eye(4)
    elif v['mode']=='top':
     # For single-camera snapshots the world axes are optical, not ROS map axes.
     target=np.array([0.,0.,2.]);eye=np.array([0.,-2.,1.]);forward=target-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0.,-1.,0.]);right/=np.linalg.norm(right);down=np.cross(forward,right);T=np.eye(4);T[:3,:3]=np.stack([right,down,forward],1);T[:3,3]=eye;state['T']=T
    elif v['mode']!='free':raise ValueError('mode')
   for key,axis in [('dx',0),('dz',2)]:
    if key in v:
     x=float(v[key])
     if not math.isfinite(x) or abs(x)>.5:raise ValueError('step')
     state['T'][:3,3]+=state['T'][:3,axis]*x
   if 'yaw' in v:
    angle=float(v['yaw'])
    if not math.isfinite(angle) or abs(angle)>.5:raise ValueError('yaw')
    c,s=math.cos(angle),math.sin(angle);state['T'][:3,:3]=state['T'][:3,:3]@np.array([[c,0,s],[0,1,0],[-s,0,c]])
   if 'render_mode' in v:
    if v['render_mode'] not in (0,1,2):raise ValueError('render mode')
    state['mode']=v['render_mode']
   state['revision']+=1
 server=FrameServer(a.host,a.port,control);start=time.monotonic();last=-1
 try:
  while a.seconds<=0 or time.monotonic()-start<a.seconds:
   with lock:T=state['T'].copy();mode=state['mode'];rev=state['revision']
   if rev!=last:
    t=time.perf_counter();im,depth,ids=m.render(camera,T,mode);ms=(time.perf_counter()-t)*1000
    server.update(im,{'mode':'saved_map_interactive_browse','backend':'KV260 native PS if running on board','source':str(scene),'surfels':m.count,'render_ms':ms,'cpu_threads':m.threads,'coverage':float(np.mean(ids>=0)),'revision':rev,'not_live_mapping':True});last=rev
   time.sleep(.05)
 finally:server.close();m.close()
if __name__=='__main__':main()
