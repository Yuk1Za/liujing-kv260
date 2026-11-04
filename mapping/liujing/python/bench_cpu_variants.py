"""Keep the faster strict FP32 CPU implementation; compare exact outputs."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from native import Map,Camera
p=argparse.ArgumentParser();p.add_argument('--before',required=True);p.add_argument('--after',required=True);p.add_argument('--out',required=True);p.add_argument('--modes',default='0,3,4');a=p.parse_args();modes=[int(x) for x in a.modes.split(',')];out=Path(a.out);out.mkdir(parents=True,exist_ok=False);rows=[]
for scene in ('camera_preview_04','analytic_100k'):
 root=Path('runs')/scene;models=[Map(library=lib) for lib in (a.before,a.after)];cam=Camera.from_dict(json.loads((root/'camera.json').read_text()));cam.cy+=(480-cam.height)/2;cam.height=480
 try:
  for m in models:m.restore(np.load(root/'surfels.npy'))
  for mode in modes:
   for m in models:m.render(cam,np.eye(4),mode)
   for repeat in range(3):
    for view in range(10):
     T=np.eye(4,dtype=np.float32);T[0,3]=.1*np.sin(view*.7);T[2,3]=-.1*(view%2);outputs={};timings={}
     for k in ((0,1) if (view+repeat)%2==0 else (1,0)):
      start=time.perf_counter();outputs[k]=models[k].render(cam,T,mode);timings[k]=(time.perf_counter()-start)*1000
     equal=all((x is None and y is None) or (x is not None and y is not None and np.array_equal(x,y)) for x,y in zip(outputs[0],outputs[1]))
     rows.append(dict(scene=scene,mode=mode,repeat=repeat,view=view,before_ms=timings[0],after_ms=timings[1],exact=equal))
 finally:
  for m in models:m.close()
summary={scene:{str(mode):{key:float(np.mean([r[key] for r in rows if r['scene']==scene and r['mode']==mode])) for key in ('before_ms','after_ms')} for mode in modes} for scene in ('camera_preview_04','analytic_100k')}
r=dict(all_exact=all(x['exact'] for x in rows),summary=summary,rows=rows,sha256={s:hashlib.sha256(Path(s).read_bytes()).hexdigest() for s in (a.before,a.after)});(out/'report.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='rows'}))
if not r['all_exact']:raise RuntimeError('CPU vector contract mismatch')
