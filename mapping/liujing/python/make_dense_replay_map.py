"""Additional fixed workload from real depth samples, without fabricated geometry.

Uses synthetic stationary poses, so it is not a calibrated moving-room map.
The original fixed map and every existing benchmark remain unchanged.
"""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from PIL import Image
from native import Map,Camera
from alignment import register_color,checked_pose

p=argparse.ArgumentParser();p.add_argument('--capture',default='runs/camera_02');p.add_argument('--out',required=True);a=p.parse_args()
root=Path(a.capture);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
cal=json.loads((root/'factory_calibration.json').read_text());T=np.eye(4);T[:3,:3]=np.array(cal['rotation_raw']).reshape(3,3);T[:3,3]=np.array(cal['translation_raw'])*.001;cal['T_color_depth']=checked_pose(T).tolist()
cam=Camera.from_dict(cal['depth']);pose=np.eye(4,dtype=np.float32)
pairs=sorted(root.glob('*_depth.png'),key=lambda p:int(p.stem.split('_')[0]))
if not pairs:raise ValueError('no paired capture')
m=Map(capacity=100000,stride=1);rows=[]
try:
 for index,dpath in enumerate(pairs):
  rpath=dpath.with_name(dpath.name.replace('_depth','_rgb'));start=time.perf_counter()
  d,rgb=register_color(np.asarray(Image.open(dpath),np.float32)*.001,np.asarray(Image.open(rpath).convert('RGB')),cal,native_library=m.lib)
  m.integrate(d,rgb,cam,pose,1+index*.1)
  rows.append(dict(source_index=index,source_name=dpath.name,surfels=m.count,map_version=m.version,capacity_drops=m.capacity_drops,ms=(time.perf_counter()-start)*1000))
  print(json.dumps(rows[-1]),flush=True)
 np.save(out/'surfels.npy',m.surfels());(out/'camera.json').write_text(json.dumps(cam.as_dict(),indent=2))
 report=dict(scope='additional fixed real-depth workload; synthetic stationary identity poses, not moving-room acceptance',geometry='native stride-one integration of each captured RGB-D pair exactly once; no duplicated or fabricated surfels added to reach capacity',capacity=100000,surfels=m.count,capacity_reached=m.count==100000,rows=rows,sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(m.lib._name),Path(__file__),root/'factory_calibration.json',*pairs,*[p.with_name(p.name.replace('_depth','_rgb')) for p in pairs],out/'surfels.npy',out/'camera.json']})
 (out/'provenance.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('rows','sha256')}),flush=True)
finally:m.close()
