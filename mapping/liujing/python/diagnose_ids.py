"""Preserved-bit experiment: compiler contraction vs exact mask/owner parity."""
import argparse,ctypes as C,fcntl,hashlib,json,os,time
from pathlib import Path
import numpy as np
from native import Map,Camera,ptr

def c_model(m,cam,T):
    s=np.zeros((m.count,16),np.float32);off=np.zeros(1201,np.uint32);idx=np.zeros(4000000,np.uint32)
    n=m.lib.lj_prepare(m.handle,C.byref(cam),ptr(T),ptr(s),ptr(off),ptr(idx),len(idx))
    if n<0:raise RuntimeError(n)
    shape=(cam.height,cam.width);d=np.empty(shape,np.float32);i=np.empty(shape,np.int32);rgb=np.empty(shape,np.uint32)
    f=m.lib.lj_raster;f.argtypes=[C.c_void_p]*6+[C.c_int]*2+[C.c_float]*7;f.restype=None
    f(ptr(s),ptr(off),ptr(idx),ptr(d),ptr(i),ptr(rgb),cam.width,cam.height,cam.fx,cam.fy,cam.cx,cam.cy,cam.near_z,cam.far_z,.01)
    return d,i,s,off,idx[:n]

def compare(d,i,rd,ri):
    valid=(i>=0)&(ri>=0);err=np.abs(d[valid]-rd[valid])
    return dict(mask_mismatch=int(np.sum((i>=0)!=(ri>=0))),id_mismatch=int(np.sum(i!=ri)),depth_max_m=float(err.max()) if len(err) else 0.,depth_bits_mismatch=int(np.sum(d.view(np.uint32)!=rd.view(np.uint32))))

def trace(s,indices,x,y,c):
    # Explicit binary32 rounding after every operation: independent diagnostic.
    F=np.float32;rx=F(F(F(x)-F(c.cx))/F(c.fx));ry=F(F(F(y)-F(c.cy))/F(c.fy));out=[]
    def hx(v):return f'{int(np.asarray(F(v)).view(np.uint32)):08x}'
    for id in indices:
        a=s[id];den=F(F(F(a[3]*rx)+F(a[4]*ry))+a[5]);num=F(F(F(a[3]*a[0])+F(a[4]*a[1]))+F(a[5]*a[2]))
        if abs(den)<F(1e-6):continue
        z=F(num/den);dx=F(F(rx*z)-a[0]);dy=F(F(ry*z)-a[1]);dz=F(z-a[2]);q=F(F(F(F(dx*dx)+F(dy*dy))+F(dz*dz))/F(a[6]*a[6]))
        if not (a[12]<=x<=a[13] and a[14]<=y<=a[15]):continue
        out.append(dict(id=int(id),bounds=a[12:16].astype(int).tolist(),ray_hex=[hx(rx),hx(ry)],den_hex=hx(den),num_hex=hx(num),z_hex=hx(z),q_hex=hx(q),z=float(z),q=float(q),covered=bool(q<1 and c.near_z<=z<=c.far_z)))
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--bit',required=True);p.add_argument('--restore-bit',required=True);p.add_argument('--contract',default='build_contract/libliujing.so');a=p.parse_args()
    out=Path(a.out);out.mkdir(exist_ok=False,parents=True);lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from hardware_ownership import require_no_localization_clients
    require_no_localization_clients()
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','')
    from pynq import Overlay
    from accelerator import Raster
    r=None;maps=[];rows=[];traces=[]
    try:
        ov=Overlay(a.bit);r=Raster(ov,surfel_capacity=100000)
        for scene in ('runs/camera_preview_04','runs/analytic_100k'):
            root=Path(scene);surf=np.load(root/'surfels.npy');cam=Camera.from_dict(json.loads((root/'camera.json').read_text()));cam.cy+=(480-cam.height)/2;cam.height=480
            old=Map();new=Map(library=str(Path(a.contract).resolve()));maps.extend([old,new]);old.restore(surf);new.restore(surf)
            for view in range(10):
                T=np.eye(4,dtype=np.float32);T[0,3]=.1*np.sin(view*.7);T[2,3]=-.1*(view%2)
                _,od,oi=old.render(cam,T);_,nd,ni=new.render(cam,T);cd,ci,s,off,idx=c_model(new,cam,T);im,pd,pi=r.render(new,cam,T)
                row=dict(scene=scene,view=view,old_vs_contract=compare(od,oi,nd,ni),contract_vs_c=compare(nd,ni,cd,ci),contract_vs_pl=compare(nd,ni,pd,pi),pl_timing=r.timing);rows.append(row);print(json.dumps(row),flush=True)
                # Save exact differences plus reproducible prepared descriptors.
                mask=(oi!=ni)|(ni!=pi);ys,xs=np.where(mask)
                for y,x in list(zip(ys,xs))[:8]:
                    tile=(y//16)*((cam.width+15)//16)+x//16
                    traces.append(dict(scene=scene,view=view,x=int(x),y=int(y),old=int(oi[y,x]),contract=int(ni[y,x]),pl=int(pi[y,x]),candidates=trace(s,idx[off[tile]:off[tile+1]],x,y,cam)))
                if view in (0,2,5,7,8):np.savez_compressed(out/f'{root.name}_{view}.npz',T=T,old_ids=oi,contract_ids=ni,pl_ids=pi,old_depth=od,contract_depth=nd,pl_depth=pd)
        report=dict(rows=rows,all_contract_ids_exact=all(x['contract_vs_pl']['id_mismatch']==0 for x in rows),library_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (Path('build/libliujing.so'),Path(a.contract))},bit_sha256=hashlib.sha256(Path(a.bit).read_bytes()).hexdigest())
        (out/'report.json').write_text(json.dumps(report,indent=2));(out/'traces.json').write_text(json.dumps(traces,indent=2));print('SUMMARY '+json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
    finally:
        Overlay(a.restore_bit);(out/'restored.json').write_text(json.dumps({'bit':a.restore_bit,'sha256':hashlib.sha256(Path(a.restore_bit).read_bytes()).hexdigest()}))
        if r:r.failed=False;r.close()
        for m in maps:m.close()
if __name__=='__main__':main()
