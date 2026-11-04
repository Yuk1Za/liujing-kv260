"""Persist a complete map snapshot and local-coordinate submaps atomically per file."""
import json,os
from pathlib import Path
import numpy as np

def save_snapshot(root,surfels,version):
 root=Path(root);root.mkdir(parents=True,exist_ok=True);a=np.asarray(surfels,np.float32)
 if a.ndim!=2 or a.shape[1]!=16 or not np.isfinite(a).all():raise ValueError('invalid snapshot')
 directory=root/'submaps';directory.mkdir(exist_ok=True);entries=[]
 for tag in np.unique(a[:,12]).astype(int):
  indices=np.flatnonzero(a[:,12]==tag);part=a[indices].copy();origin=part[:,:3].mean(0,dtype=np.float64);part[:,:3]-=origin
  T=np.eye(4);T[:3,3]=origin;filename=f'{tag:06}.npz';temp=directory/(filename+'.tmp')
  with temp.open('wb') as f:np.savez_compressed(f,surfels=part,T_map_submap=T,global_ids=indices)
  os.replace(temp,directory/filename);entries.append({'id':int(tag),'file':'submaps/'+filename,'surfels':len(part)})
 temp=root/'surfels.npy.tmp'
 with temp.open('wb') as f:np.save(f,a)
 os.replace(temp,root/'surfels.npy')
 manifest={'map_version':int(version),'surfels':len(a),'submaps':entries,'coordinate_contract':'p_map=T_map_submap*p_submap; normals rotate only','timestamp_contract':'surfel last_seen is seconds relative to session first fused stamp; frame source stamps in keyframes','snapshot':'saved complete version; no loop-closure deformation'}
 temp=root/'map_manifest.json.tmp';temp.write_text(json.dumps(manifest,indent=2));os.replace(temp,root/'map_manifest.json')
