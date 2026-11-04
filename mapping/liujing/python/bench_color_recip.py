"""Experimental color reciprocal: exact geometry and independent FP32 quality."""
import argparse,json,time
from pathlib import Path
import numpy as np
from native import Map,Camera
p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();rows=[]
for scene in ('camera_preview_04','analytic_100k'):
 root=Path('runs')/scene;m=Map();m.restore(np.load(root/'surfels.npy'));c=Camera.from_dict(json.loads((root/'camera.json').read_text()));c.cy+=(480-c.height)/2;c.height=480
 try:
  for view in range(10):
   T=np.eye(4,dtype=np.float32);T[0,3]=.1*np.sin(view*.7);T[2,3]=-.1*(view%2);values={};times={}
   for mode in (0,4,5):
    start=time.perf_counter();values[mode]=m.render(c,T,mode);times[mode]=(time.perf_counter()-start)*1000
   f,fd,fi=values[0];q,qd,qi=values[5];valid=fi>=0;err=q[valid].astype(float)-f[valid].astype(float);mse=float(np.mean(err*err))
   rows.append(dict(scene=scene,view=view,ids_exact=bool(np.array_equal(fi,qi)),depth_bits_exact=bool(np.array_equal(fd.view(np.uint32),qd.view(np.uint32))),psnr_db=120. if mse==0 else float(10*np.log10(255**2/mse)),color_max=int(abs(err).max()),timing_ms=times))
 finally:m.close()
# Stress the coverage predicate at adjacent representable values, powers of
# two, and random exponents, including small NORMAL denominators.
rng=np.random.default_rng(913);b=rng.integers(0x00800000,0x7f7fffff,200000,dtype=np.uint32).view(np.float32)
with np.errstate(over='ignore',invalid='ignore',divide='ignore'):
 for aa in (np.nextafter(b,np.float32(0)),b,np.nextafter(b,np.float32(np.inf)),rng.integers(0,0x7f7fffff,len(b),dtype=np.uint32).view(np.float32)):
  np.testing.assert_array_equal(aa/b<1,aa<b)
result=dict(scope='qrcp_v1 CPU candidate, not hardware acceptance or matched speedup',coverage_predicate_cases=800000,all_geometry_exact=all(x['ids_exact'] and x['depth_bits_exact'] for x in rows),min_psnr_db=min(x['psnr_db'] for x in rows),rows=rows)
Path(a.out).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
assert result['all_geometry_exact'] and result['min_psnr_db']>=40
