"""Analytic RGB-D replay on the host running this script (KV260 for PS evidence)."""
import argparse,json,time,platform
from pathlib import Path
import numpy as np
from PIL import Image
from native import Camera,Map
def synthetic(cam,T):
    yy,xx=np.indices((cam.height,cam.width),dtype=np.float32)
    rays=np.stack([(xx-cam.cx)/cam.fx,(yy-cam.cy)/cam.fy,np.ones_like(xx)],-1)
    wr=rays@T[:3,:3].T;o=T[:3,3]
    z=(2.8-o[2])/wr[:,:,2];world=o+wr*z[:,:,None]
    board=(1.6-o[2])/wr[:,:,2];hit=o+wr*board[:,:,None]
    mask=(abs(hit[:,:,0])<.28)&(abs(hit[:,:,1])<.30)&(board>0)
    z=np.where(mask,board,z).astype(np.float32);world=o+wr*z[:,:,None]
    checker=((np.floor(world[:,:,0]*12)+np.floor(world[:,:,1]*12))%2).astype(np.uint8)
    rgb=np.empty((*z.shape,3),np.uint8);rgb[:,:,0]=150+checker*50;rgb[:,:,1]=125+checker*40;rgb[:,:,2]=95+checker*30
    rgb[mask]=np.stack([40+180*((np.floor(hit[:,:,0]*25).astype(int)%3)==0),80+100*((np.floor(hit[:,:,0]*25).astype(int)%3)==1),np.full_like(xx,160)],-1).astype(np.uint8)[mask]
    return z,rgb
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);ap.add_argument('--frames',type=int,default=20);ap.add_argument('--width',type=int,default=320);ap.add_argument('--height',type=int,default=200);ap.add_argument('--repeats',type=int,default=1);a=ap.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True);cam=Camera(a.width,a.height,a.width*.8,a.width*.8,(a.width-1)/2,(a.height-1)/2,.3,5)
    rows=[];m=Map();keydir=out/'keyframes';keydir.mkdir(exist_ok=True)
    for i in range(a.frames):
        T=np.eye(4,dtype=np.float32);T[0,3]=.2*np.sin(i*.10);d,rgb=synthetic(cam,T)
        t=time.perf_counter();added=m.integrate(d,rgb,cam,T,1+i*.1);f=time.perf_counter();image,dep,ids=m.render(cam,T);end=time.perf_counter()
        rows.append({'frame':i,'added':added,'surfels':m.count,'version':m.version,'fusion_ms':(f-t)*1000,'render_ms':(end-f)*1000,'total_ms':(end-t)*1000,'coverage':float(np.mean(ids>=0))})
        if i%5==0 or i==a.frames-1:
            Image.fromarray(image).save(out/f'view_{i:03}.png');Image.fromarray(rgb).save(out/f'input_{i:03}.png')
            np.savez_compressed(keydir/f'{i:04}.npz',depth=d,rgb=rgb,T=T,stamp=1+i*.1)
    # A novel camera position: real 3-D reprojection, not replay of source images.
    T=np.eye(4,dtype=np.float32);T[0,3]=-.25;T[2,3]=-.25
    image,dep,ids=m.render(cam,T);Image.fromarray(image).save(out/'novel_view.png')
    np.save(out/'surfels.npy',m.surfels());np.save(out/'novel_depth.npy',dep)
    (out/'camera.json').write_text(json.dumps(cam.as_dict(),indent=2))
    report={'mode':'synthetic_rgbd_known_poses','machine':platform.machine(),'backend':'native_cpu_fp32','camera':cam.as_dict(),'rows':rows,'capacity_drops':m.capacity_drops,'hardware_acceleration':False,
        'summary':{key:{'mean':float(np.mean([r[key] for r in rows])),'p99':float(np.percentile([r[key] for r in rows],99))} for key in ['fusion_ms','render_ms','total_ms']}}
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({'machine':report['machine'],'mode':report['mode'],'surfels':m.count,'timing':report['summary']}));m.close()
if __name__=='__main__':main()
