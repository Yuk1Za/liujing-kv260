"""Deterministic analytic 100k-surface workload, explicitly not a room scan."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
u,v=np.meshgrid(np.linspace(20,619,400,dtype=np.float32),np.linspace(20,459,250,dtype=np.float32));x=(u.ravel()-319.5)/480;y=(v.ravel()-239.5)/480;z=2/(1-.1*x)
s=np.zeros((len(z),16),np.float32);s[:,0]=x*z;s[:,1]=y*z;s[:,2]=z;s[:,3:6]=np.array([.1,0,-1],np.float32)/np.sqrt(1.01);s[:,6]=.005;s[:,7]=(u.ravel()/639)*255;s[:,8]=(v.ravel()/479)*255;s[:,9]=120;s[:,10]=1
np.save(out/'surfels.npy',s);(out/'camera.json').write_text(json.dumps({'width':640,'height':480,'fx':480,'fy':480,'cx':319.5,'cy':239.5,'near_z':.3,'far_z':5}));(out/'provenance.json').write_text(json.dumps({'kind':'analytic inclined plane, not real map','surfels':len(s),'all_visible_in_reference_view':True}));print(len(s))
