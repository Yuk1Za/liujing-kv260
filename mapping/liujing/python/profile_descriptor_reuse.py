"""Trace the real renderer's descriptor order; this is a cache model, not PL timing."""
import argparse,ctypes as C,hashlib,json
from pathlib import Path
import numpy as np
from native import Map,Camera,ptr

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();rows=[]
 for scene in ('camera_preview_04','analytic_100k'):
  root=Path('runs')/scene;model=root/'surfels.npy';m=Map();m.restore(np.load(model,allow_pickle=False));cam=Camera.from_dict(json.loads((root/'camera.json').read_text()));cam.cy+=(480-cam.height)/2;cam.height=480
  surf=np.empty((m.count,16),np.float32);off=np.empty(1201,np.uint32);idx=np.empty(4000000,np.uint32);tiles=((cam.width+15)//16)*((cam.height+15)//16)
  try:
   for view in range(10):
    T=np.eye(4,dtype=np.float32);T[0,3]=.1*np.sin(view*.7);T[2,3]=-.1*(view%2)
    refs=m.lib.lj_prepare_checked(m.handle,C.byref(cam),ptr(T),ptr(surf),ptr(off),ptr(idx),len(idx),m.count,m.version,1)
    if refs<0:raise RuntimeError(refs)
    order=[]
    for tile in range(tiles):
     ids=idx[off[tile]:off[tile+1]].tolist();order.extend(ids)
     if len(ids)>512:order.extend(ids) # no per-tile retention in the existing large path
    caches=[]
    for capacity in (1024,4096,8192,16384):
     tags=[-1]*capacity;hits=0
     for i in order:
      slot=i&(capacity-1)
      if tags[slot]==i:hits+=1
      else:tags[slot]=i
     caches.append(dict(capacity=capacity,hits=hits,misses=len(order)-hits,hit_fraction=hits/max(1,len(order))))
    rows.append(dict(scene=scene,view=view,surfels=m.count,map_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),references=refs,descriptor_reads=len(order),index_sha256=hashlib.sha256(idx[:refs].tobytes()).hexdigest(),cache_model=caches))
  finally:m.close()
 out=Path(a.out);out.parent.mkdir(parents=True,exist_ok=True)
 with out.open('x') as f:json.dump(dict(scope='PS trace, direct-mapped whole-frame descriptor cache MODEL; not hardware hits or timing',rows=rows),f,indent=2)
 print('descriptor trace saved',out,flush=True)
if __name__=='__main__':main()
