"""Count exact clipped fragment work; cycle floor is a model, not a speed claim."""
import argparse,ctypes as C,json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from native import Map,Camera,ptr
p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('--out',required=True);a=p.parse_args();root=Path(a.scene);m=Map();m.restore(np.load(root/'surfels.npy'));cam=Camera.from_dict(json.loads((root/'camera.json').read_text()));cam.cy+=(480-cam.height)/2;cam.height=480
s=np.zeros((m.count,16),np.float32);off=np.zeros(1201,np.uint32);idx=np.zeros(4000000,np.uint32);T=np.eye(4,dtype=np.float32);start=time.perf_counter();refs=m.lib.lj_prepare(m.handle,C.byref(cam),ptr(T),ptr(s),ptr(off),ptr(idx),len(idx));elapsed=time.perf_counter()-start
if refs<0:raise RuntimeError('prepare failed')
fragments=0;txs=(cam.width+15)//16;tiles=txs*((cam.height+15)//16)
for tile in range(tiles):
 b=s[idx[off[tile]:off[tile+1]],12:16].astype(int);x=tile%txs*16;y=tile//txs*16
 fragments+=int(np.sum(np.maximum(0,np.minimum(b[:,1],x+15)-np.maximum(b[:,0],x)+1)*np.maximum(0,np.minimum(b[:,3],y+15)-np.maximum(b[:,2],y)+1)))
report={'scope':'exact workload counts and an arithmetic model; ignores memory stalls and loop overhead','map_surfels':m.count,'visible_surfels':int(len(np.unique(idx[:refs]))),'tile_references':refs,'fragments_per_pass':fragments,'passes':2,'prepare_ms_single_observation':elapsed*1000,'fragment_II7_cycle_floor_ms_at_100MHz':fragments*2*7/100000,'fragment_II1_cycle_floor_ms_at_100MHz':fragments*2/100000,'input_surfel_read_bytes_no_cache_two_passes':refs*64*2,'output_bytes':cam.width*cam.height*12};Path(a.out).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));m.close()
