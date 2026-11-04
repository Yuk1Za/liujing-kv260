import argparse,json,time
from pathlib import Path
import numpy as np
from native import Map,Camera
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();rows=[]
 for scene in ('runs/camera_preview_04','runs/analytic_100k'):
  root=Path(scene);m=Map();m.restore(np.load(root/'surfels.npy'));c=Camera.from_dict(json.loads((root/'camera.json').read_text()));c.cy+=(480-c.height)/2;c.height=480
  for mode in (0,3,4):
   m.render(c,np.eye(4),mode);times=[]
   for i in range(10):
    T=np.eye(4);T[0,3]=.1*np.sin(i*.7);T[2,3]=-.1*(i%2);start=time.perf_counter();m.render(c,T,mode);times.append((time.perf_counter()-start)*1000)
   rows.append(dict(scene=scene,mode=mode,mean_ms=float(np.mean(times)),p99_ms=float(np.percentile(times,99))))
  m.close()
 Path(a.out).write_text(json.dumps(rows,indent=2));print(json.dumps(rows))
if __name__=='__main__':main()
