"""Independent offline point-to-point ICP diagnostic, never a replacement pose."""
import argparse
import json
from pathlib import Path
import numpy as np


def sample(points, voxel, limit):
    points=np.asarray(points,dtype=np.float64)
    r=np.linalg.norm(points,axis=1)
    points=points[np.isfinite(points).all(1)&(r>.5)&(r<15)]
    _, ids=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
    points=points[np.sort(ids)]
    if len(points)>limit:
        points=points[np.linspace(0,len(points)-1,limit).astype(int)]
    return points


def nearest(a,b):
    indices=[]; distances=[]; bb=np.sum(b*b,axis=1)
    for start in range(0,len(a),192):
        x=a[start:start+192]
        d=np.maximum(np.sum(x*x,axis=1)[:,None]+bb[None,:]-2*x@b.T,0)
        idx=np.argmin(d,axis=1)
        indices.append(idx); distances.append(d[np.arange(len(x)),idx])
    return np.concatenate(indices),np.sqrt(np.concatenate(distances))


def align(source,target):
    T=np.eye(4); stages=[]
    for voxel,limit,gate in [(.3,900,1.),(.15,2200,.45),(.08,4000,.2)]:
        src=sample(source,voxel,limit); dst=sample(target,voxel,limit)
        if min(len(src),len(dst))<50:raise ValueError('insufficient geometry')
        for iteration in range(25):
            x=src@T[:3,:3].T+T[:3,3]
            ids,d=nearest(x,dst); valid=d<gate
            if valid.sum()<50:raise ValueError('insufficient overlapping points')
            valid&=d<=np.quantile(d[valid],.85)
            a=x[valid]; b=dst[ids[valid]]; ac=a.mean(0);bc=b.mean(0)
            u,_,vt=np.linalg.svd((a-ac).T@(b-bc)); R=vt.T@u.T
            if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
            shift=bc-R@ac; update=np.eye(4);update[:3,:3]=R;update[:3,3]=shift;T=update@T
            if np.linalg.norm(shift)<1e-5 and np.linalg.norm(R-np.eye(3))<1e-5:break
        _,d=nearest(src@T[:3,:3].T+T[:3,3],dst)
        stages.append(dict(voxel_m=voxel,iterations=iteration+1,overlap_fraction=float(np.mean(d<gate)),
                           median_nn_m=float(np.median(d)),p90_nn_m=float(np.quantile(d,.9))))
    return dict(accepted=False,scope='Offline raw-cloud registration; local ICP can fail or be degenerate; no ground truth',
        transform_source_to_target=T.tolist(),translation_norm_m=float(np.linalg.norm(T[:3,3])),
        rotation_deg=float(np.degrees(np.arccos(np.clip((np.trace(T[:3,:3])-1)/2,-1,1)))),stages=stages)


def main():
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('target');p.add_argument('--out',required=True)
    a=p.parse_args();result=align(np.load(a.source,allow_pickle=False),np.load(a.target,allow_pickle=False))
    Path(a.out).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
