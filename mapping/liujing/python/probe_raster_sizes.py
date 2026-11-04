"""Persist each submission before execution, including failures; no acceptance claims."""
import argparse,fcntl,json,os,time,hashlib
from pathlib import Path
import numpy as np
from native import Map,Camera

def main():
    p=argparse.ArgumentParser();p.add_argument('--bit',required=True);p.add_argument('--out',required=True);p.add_argument('--width',type=int,required=True);p.add_argument('--height',type=int,required=True);p.add_argument('--mode',type=int,default=3);a=p.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from hardware_ownership import require_no_localization_clients
    require_no_localization_clients()
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
    from pynq import Overlay
    from accelerator import Raster
    r=None;m=Map();result=dict(width=a.width,height=a.height,mode=a.mode,bit_sha256=hashlib.sha256(Path(a.bit).read_bytes()).hexdigest(),scope='empty-map completion diagnostic')
    try:
        r=Raster(Overlay(a.bit),numeric_contract='qrcp_v1')
        result['dma']={k:int(v.physical_address) for k,v in r.buffers.items()}
        (out/'submitted.json').write_text(json.dumps(result,indent=2))
        c=Camera(a.width,a.height,200,200,a.width/2,a.height/2,.1,10)
        start=time.perf_counter()
        try:
            _,d,i=r.render(m,c,np.eye(4,dtype=np.float32),a.mode)
            result.update(completed=True,exact_empty=bool(np.all(d==0) and np.all(i==-1)),timing=r.timing,depth_sample=d.ravel()[:16].tolist(),id_sample=i.ravel()[:16].tolist())
        except Exception as e:
            result.update(completed=False,error=repr(e),elapsed_ms=(time.perf_counter()-start)*1000,control={hex(x):int(r.ip.read(x)) for x in range(0,0xa8,4)})
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    finally:
        Overlay('/home/ubuntu/losam_live_20261021/artifacts/c16_feature.bit')
        if r:r.failed=False;r.close()
        m.close();(out/'restored.json').write_text(json.dumps(dict(original_c16=True)))
if __name__=='__main__':main()
