"""Depth disagreement check for candidate rigid transforms on joint observations.

This is a diagnostic, not automatic calibration acceptance: occlusion, sensor
noise, timestamp error, and motion all contribute to the measured residuals.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from alignment import register_color

p=argparse.ArgumentParser();p.add_argument('capture');p.add_argument('--candidates',required=True);p.add_argument('--out',required=True);p.add_argument('--no-overlay',action='store_true');a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
files=sorted(Path(a.capture).glob('*.npz'));report={'scope':'same-observation depth residual diagnostic, not held-out multi-scene acceptance','frames':len(files),'candidates':{}}
for candidate in sorted(Path(a.candidates).glob('*.json')):
 cal=json.loads(candidate.read_text())
 if 'T_pose_depth' not in cal:continue
 T=np.linalg.inv(cal['T_pose_depth']);c=cal['depth'];rows=[];errors=[]
 for number,f in enumerate(files):
  with np.load(f,allow_pickle=False) as data:
   depth=data['depth'];rgb=data['rgb'];xyz=data['lidar_xyz'];stamps=data['stamps']
  pc=xyz@T[:3,:3].T+T[:3,3];pc=pc[np.isfinite(pc).all(1)&(pc[:,2]>.3)&(pc[:,2]<5)]
  u=np.rint(pc[:,0]/pc[:,2]*c['fx']+c['cx']).astype(int);v=np.rint(pc[:,1]/pc[:,2]*c['fy']+c['cy']).astype(int)
  inside=(u>=2)&(u<c['width']-2)&(v>=2)&(v<c['height']-2);u=u[inside];v=v[inside];pc=pc[inside];d=depth[v,u];valid=(d>.3)&(d<5);err=pc[valid,2]-d[valid];errors.extend(err.tolist())
  rows.append({'frame':f.name,'in_depth_fov':len(pc),'valid_depth_comparisons':len(err),'abs_error_median_m':float(np.median(abs(err))) if len(err) else None,'within_5cm_fraction':float(np.mean(abs(err)<.05)) if len(err) else None,'rgb_depth_stamp_difference_s':float(stamps[0]-stamps[1]),'depth_lidar_stamp_difference_s':float(stamps[1]-stamps[2])})
  if number==0 and not a.no_overlay:
   _,registered=register_color(depth,rgb,cal);im=Image.fromarray(registered);draw=ImageDraw.Draw(im)
   for x,y,e in zip(u[valid],v[valid],err):draw.ellipse((int(x)-1,int(y)-1,int(x)+1,int(y)+1),fill=(0,255,60) if abs(e)<.05 else (255,50,30))
   im.save(out/(candidate.stem+'_projection.jpg'),quality=92)
 errors=np.abs(np.asarray(errors));report['candidates'][candidate.stem]={'comparisons':len(errors),'abs_depth_difference_median_m':float(np.median(errors)) if len(errors) else None,'abs_depth_difference_p90_m':float(np.percentile(errors,90)) if len(errors) else None,'within_5cm_fraction':float(np.mean(errors<.05)) if len(errors) else None,'rows':rows}
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:({n:{x:y for x,y in v.items() if x!='rows'} for n,v in val.items()} if k=='candidates' else val) for k,val in report.items()}))
