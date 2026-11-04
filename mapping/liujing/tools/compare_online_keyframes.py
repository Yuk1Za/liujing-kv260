"""CPU/PL matched replay of actual fused RGB-D and measured LO-SAM poses.

Consumes registered keyframes, not synthetic identity poses; excludes acquisition
and localization runtime. Does not claim end-to-end speedup or calibration.
"""
import argparse,fcntl,json,os,time,hashlib
from pathlib import Path
import numpy as np
from native import Map,Camera
from viewer import encode_frame,compose_trajectory


def sha(x):return hashlib.sha256(x if isinstance(x,bytes) else x.tobytes()).hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('visual');p.add_argument('--bit',required=True);p.add_argument('--out',required=True);p.add_argument('--frames',type=int,default=10);p.add_argument('--views',type=int,choices=(1,3),default=3);a=p.parse_args()
    source=Path(a.visual);out=Path(a.out);out.mkdir(exist_ok=False,parents=True)
    files=sorted((source/'keyframes').glob('*.npz'))[:a.frames]
    if not files:raise ValueError('no actual online keyframes')
    lease=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from hardware_ownership import require_no_localization_clients
    require_no_localization_clients()
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    from pynq import Overlay
    from accelerator import Raster
    overlay=Overlay(a.bit);r=Raster(overlay,surfel_capacity=1000000,numeric_contract='qrcp_v1')
    cam=Camera.from_dict(json.loads((source/'camera.json').read_text()));viewcam=Camera(640,480,cam.fx,cam.fy,319.5,239.5,cam.near_z,cam.far_z)
    maps={b:Map() for b in ('cpu','pl')};path=[];rows=[];passed=True
    try:
        for index,file in enumerate(files):
            with np.load(file,allow_pickle=False) as f:d=f['depth'];rgb=f['rgb'];T=f['T'];stamp=float(f['stamp'])
            path.append(T[:3,3].copy());pair={}
            views=[T.copy()]
            if a.views==3:
                translated=T.copy();translated[:3,3]+=.1*T[:3,0];views.append(translated)
                rotated=T.copy();c,s=np.cos(.1),np.sin(.1);rotated[:3,:3]=T[:3,:3]@np.array([[c,0,s],[0,1,0],[-s,0,c]]);views.append(rotated)
            for backend in (('cpu','pl') if index%2==0 else ('pl','cpu')):
                m=maps[backend];start=time.perf_counter();cpu_start=time.process_time();version=m.version
                _,ad,ai=r.render(m,cam,T,3,frame_id=stamp) if backend=='pl' else m.render(cam,T,3)
                association_ms=(time.perf_counter()-start)*1000;stage=time.perf_counter()
                m.integrate(d,rgb,cam,T,stamp,association=(ad,ai,version))
                fusion_ms=(time.perf_counter()-stage)*1000;rendered=[];view_ms=[]
                for view in views:
                    stage=time.perf_counter()
                    im,vd,vi=r.render(m,viewcam,view,frame_id=stamp) if backend=='pl' else m.render(viewcam,view,5)
                    jpeg=encode_frame(compose_trajectory(im,path,view,viewcam));view_ms.append((time.perf_counter()-stage)*1000)
                    rendered.append({'view_id':sha(vi),'view_depth':sha(vd),'rgb':sha(im),'jpeg':sha(jpeg)})
                pair[backend]={'ms':(time.perf_counter()-start)*1000,'process_cpu_ms':(time.process_time()-cpu_start)*1000,'association_ms':association_ms,'fusion_ms':fusion_ms,'view_ms':view_ms,'association_id':sha(ai),'association_depth':sha(ad),'views':rendered,'map':sha(m.surfels()),'map_version':m.version}
                (out/(backend+'_last.jpg')).write_bytes(jpeg)
            equal=all(pair['cpu'][k]==pair['pl'][k] for k in ('association_id','association_depth','views','map','map_version'));passed &=equal
            rows.append({'frame':file.name,'source_stamp':stamp,'source_sha256':sha(file.read_bytes()),'view_requests':[v.tolist() for v in views],'exact':equal,**pair})
            (out/'progress.json').write_text(json.dumps({'completed':len(rows),'passed':passed}))
        result={'scope':'matched registered keyframe compute using actual LO-SAM poses; not acquisition or localization throughput','timing_scope':'association, integration, requested views, JPEG and view checksums; excludes disk write and map checksum','passed':passed,'frames':len(rows),'views_per_frame':a.views,'rows':rows,'bit_sha256':sha(Path(a.bit).read_bytes()),'library_sha256':sha(Path(maps['cpu'].lib._name).read_bytes())}
        (out/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps({'passed':passed,'frames':len(rows)}))
        if not passed:raise RuntimeError('online keyframe CPU/PL mismatch')
    finally:
        r.close()
        for m in maps.values():m.close()


if __name__=='__main__':main()
