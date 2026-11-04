"""Compare sequential and bounded encoding on exactly the same CPU workload."""
import argparse,json,hashlib,os
from pathlib import Path
import numpy as np
from PIL import Image
from alignment import checked_pose
from native import Map,Camera
from bench_visual_chain import run

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
 root=Path('runs/camera_02');cal=json.loads((root/'factory_calibration.json').read_text());T=np.eye(4);T[:3,:3]=np.array(cal['rotation_raw']).reshape(3,3);T[:3,3]=np.array(cal['translation_raw'])*.001;cal['T_color_depth']=checked_pose(T).tolist()
 pairs=sorted(root.glob('*_depth.png'),key=lambda p:int(p.stem.split('_')[0]));raw=[(np.asarray(Image.open(p),np.float32)*.001,np.asarray(Image.open(p.with_name(p.name.replace('_depth','_rgb'))).convert('RGB'))) for p in pairs]
 initial=np.load('runs/camera_preview_04/surfels.npy',allow_pickle=False);cam=Camera.from_dict(cal['depth']);viewcam=Camera(640,480,cam.fx,cam.fy,cam.cx,cam.cy+40,cam.near_z,cam.far_z)
 results=[]
 for index,pipeline in enumerate((False,True)):
  m=Map();m.restore(initial)
  try:results.append(run(m,None,'cpu',raw,cal,cam,viewcam,5,index,out,cpu_mode=5,encode_pipeline=pipeline))
  finally:m.close()
 left,right=results;checks=[]
 for a,b in zip(left[0],right[0]):
  keys=('frame','source_index','surfels','map_version','map_sha256','association_id_sha256','association_depth_sha256')
  exact=all(a[k]==b[k] for k in keys)
  exact &= all(all(x[k]==y[k] for k in ('index','jpeg_bytes','jpeg_sha256','depth_sha256','id_sha256')) for x,y in zip(a['views'],b['views']))
  assert exact,'scheduler changed output frame '+str(a['frame'])
  checks.append(dict(frame=a['frame'],exact=exact,sequential_ms=a['total_ms'],pipeline_ms=b['total_ms']))
 assert all(np.array_equal(x,y) for x,y in zip(left[1],right[1]))
 report=dict(passed=True,scope='five real RGB-D inputs, stationary synthetic pose; CPU scheduling correctness, not speedup acceptance',checks=checks,runtime_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__).with_name('bench_visual_chain.py'),Path(os.environ['LIUJING_LIBRARY']))})
 (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if __name__=='__main__':main()
