"""Board experiments separating fixed output cost from fragment work.

This is a diagnostic, never an end-to-end speedup benchmark. All calls use the
same driver, including preparation and transfer. No timed call reuses outputs.
"""
import argparse,fcntl,hashlib,json,os,time
from pathlib import Path
import numpy as np
from native import Map,Camera

def main():
    p=argparse.ArgumentParser();p.add_argument('--bit',required=True);p.add_argument('--restore-bit',required=True);p.add_argument('--out',required=True);p.add_argument('--numeric-contract',default='mixed',choices=['mixed','qrcp_v1']);a=p.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from hardware_ownership import require_no_localization_clients
    require_no_localization_clients()
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
    from pynq import Overlay
    from accelerator import Raster
    r=None;rows=[];active=None
    try:
        r=Raster(Overlay(a.bit),numeric_contract=a.numeric_contract)
        for scene in ('empty','single','camera_preview_04','analytic_100k'):
            initial=np.load('runs/camera_preview_04/surfels.npy' if scene in ('empty','single') else 'runs/'+scene+'/surfels.npy',allow_pickle=False)
            if scene=='empty':initial=initial[:0]
            if scene=='single':initial=initial[:1]
            m=Map();m.restore(initial)
            try:
                for width,height in ((320,240),(640,480)):
                    c=Camera(width,height,450*width/640,450*width/640,width/2,height/2,.1,10)
                    for mode in (3,0):
                        for repeat in range(4):
                            active=dict(scene=scene,width=width,height=height,mode=mode,repeat=repeat)
                            (out/'active.json').write_text(json.dumps(active))
                            _,d,i=r.render(m,c,np.eye(4,dtype=np.float32),mode)
                            if scene=='empty':assert np.all(i==-1) and np.all(d==0)
                            if repeat:rows.append(dict(scene=scene,width=width,height=height,mode=mode,repeat=repeat,surfels=m.count,**r.timing))
            finally:m.close()
        report=dict(scope='diagnostic of empty/output floor and increasing fragment work, not acceptance',rows=rows,bit_sha256=hashlib.sha256(Path(a.bit).read_bytes()).hexdigest(),numeric_contract=a.numeric_contract,dma_bases_aligned_64={k:int(b.physical_address)%64==0 for k,b in r.buffers.items()})
        (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    except Exception as e:
        failure=dict(active=active,error=repr(e),completed_rows=rows)
        if r:
            failure['job']=r.last_job
            failure['control']={hex(x):int(r.ip.read(x)) for x in range(0,0xa8,4)}
        (out/'failure.json').write_text(json.dumps(failure,indent=2))
        raise
    finally:
        Overlay(a.restore_bit)
        if r:r.failed=False;r.close()
        (out/'restored.json').write_text(json.dumps(dict(bit=a.restore_bit)))
if __name__=='__main__':main()
