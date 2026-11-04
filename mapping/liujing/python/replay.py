"""Replay a recorded NPZ sequence; never invent a pose for missing frames."""
import argparse,json,time
from pathlib import Path
import numpy as np
from PIL import Image
from native import Camera,Map
def main():
    p=argparse.ArgumentParser();p.add_argument('sequence');p.add_argument('--out',required=True);a=p.parse_args()
    root=Path(a.sequence);out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    camera=Camera.from_dict(json.loads((root/'camera.json').read_text()));m=Map();rows=[]
    for name in sorted((root/'keyframes').glob('*.npz')):
        with np.load(name,allow_pickle=False) as f:
            begin=time.perf_counter();added=m.integrate(f['depth'],f['rgb'],camera,f['T'],float(f['stamp']));fused=time.perf_counter();rgb,depth,ids=m.render(camera,f['T']);end=time.perf_counter()
            Image.fromarray(rgb).save(out/(name.stem+'.png'));rows.append({'frame':name.name,'added':added,'fusion_ms':(fused-begin)*1000,'render_ms':(end-fused)*1000})
    if not rows:raise ValueError('no replay frames')
    (out/'report.json').write_text(json.dumps({'mode':'recorded_replay','rows':rows,'count':m.count},indent=2));np.save(out/'surfels.npy',m.surfels())
if __name__=='__main__':main()
