"""Bounded timestamps, SE(3) interpolation, and explicit calibration validity."""
from collections import deque,Counter
from functools import lru_cache
import numpy as np

@lru_cache(maxsize=4)
def _rectification_maps(w,h,fx,fy,cx,cy,distortion):
    import cv2
    k=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)
    return cv2.initUndistortRectifyMap(k,np.asarray(distortion,float),None,k,(w,h),cv2.CV_32FC1)
def checked_pose(T):
    T=np.asarray(T,dtype=np.float64)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-5):raise ValueError('invalid pose')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-3) or abs(np.linalg.det(R)-1)>1e-3:raise ValueError('non-rigid pose')
    return T
def quat_matrix(q):
    q=np.asarray(q,float);norm=np.linalg.norm(q)
    if q.shape!=(4,) or not np.isfinite(q).all() or norm<1e-8:raise ValueError('invalid quaternion')
    x,y,z,w=q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def interpolate(a,b,t):
    # Rodrigues interpolation; the pi case uses a stable symmetric eigensystem.
    R=a[:3,:3].T@b[:3,:3];angle=np.arccos(np.clip((np.trace(R)-1)/2,-1,1))
    if angle<1e-9:rot=np.eye(3)
    else:
        if abs(np.pi-angle)<1e-5:
            _,v=np.linalg.eigh((R+R.T)*.5);axis=v[:,-1]
        else:axis=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/(2*np.sin(angle))
        x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        rot=np.eye(3)+np.sin(angle*t)*K+(1-np.cos(angle*t))*(K@K)
    out=np.eye(4);out[:3,:3]=a[:3,:3]@rot;out[:3,3]=(1-t)*a[:3,3]+t*b[:3,3];return out
class PoseBuffer:
    def __init__(self,capacity=128,max_gap=.3,max_speed=None,max_angular_speed=None):
        self.items=deque(maxlen=capacity);self.max_gap=max_gap;self.stats=Counter();self.max_speed=max_speed;self.max_angular_speed=max_angular_speed;self.fault=None
    def add(self,stamp,T):
        T=checked_pose(T)
        if not np.isfinite(stamp):raise ValueError('invalid timestamp')
        if self.fault:self.stats['pose_fault_rejected']+=1;return False
        if self.items and stamp<=self.items[-1][0]:self.stats['pose_nonmonotonic']+=1;return False
        if self.items:
            previous,old=self.items[-1];dt=stamp-previous;speed=np.linalg.norm(T[:3,3]-old[:3,3])/dt;angle=np.arccos(np.clip((np.trace(old[:3,:3].T@T[:3,:3])-1)/2,-1,1))/dt
            if (self.max_speed is not None and speed>self.max_speed) or (self.max_angular_speed is not None and angle>self.max_angular_speed):
                self.fault='pose discontinuity; restart with a new map after localization recovery';self.items.clear();self.stats['pose_jump']+=1;return False
        self.items.append((float(stamp),T.copy()));return True
    def at(self,stamp):
        if self.fault:return None
        for ts,T in self.items:
            if abs(ts-stamp)<1e-7:return T.copy()
        for (ta,a),(tb,b) in zip(self.items,list(self.items)[1:]):
            if ta<stamp<tb:
                if tb-ta>self.max_gap:self.stats['pose_gap']+=1;return None
                return interpolate(a,b,(stamp-ta)/(tb-ta))
        return None
class PairQueue:
    def __init__(self,capacity=8,tolerance=.02,max_age=.5):
        self.rgb=deque();self.depth=deque();self.capacity=capacity;self.tolerance=tolerance;self.max_age=max_age;self.stats=Counter()
    def push(self,kind,stamp,data,received):
        q=getattr(self,kind);self.stats[kind+'_input']+=1
        if q and stamp<=q[-1][0]:self.stats[kind+'_nonmonotonic']+=1;return
        if len(q)>=self.capacity:q.popleft();self.stats[kind+'_overflow']+=1
        q.append((stamp,data,received))
    def take(self,now):
        for kind in ('rgb','depth'):
            q=getattr(self,kind)
            while q and now-q[0][2]>self.max_age:q.popleft();self.stats[kind+'_expired']+=1
        while self.rgb and self.depth:
            delta=self.rgb[0][0]-self.depth[0][0]
            if abs(delta)<=self.tolerance:return self.rgb.popleft(),self.depth.popleft()
            q=self.depth if delta>0 else self.rgb;q.popleft();self.stats['pair_mismatch']+=1
        return None
def validate_calibration(c,live=True):
    for key in ('T_color_depth','T_pose_depth','T_map_source'):
        checked_pose(c[key])
    if live and not all(c.get('verified',{}).get(k,False) for k in ('intrinsics','depth_color','pose_camera','time_alignment','pose_axes')):
        raise ValueError('live fusion requires verified intrinsics, depth/color, pose/camera, time alignment and pose axes; template identities are not calibration')
    for key in ('depth','color'):
        p=c[key]
        if p['width']<=0 or p['height']<=0 or p['fx']<=0 or p['fy']<=0:raise ValueError('invalid intrinsics')
def register_color(depth,rgb,cal,native_library=None,profile=None):
    """Rectify depth rays, map onto distorted RGB, z-buffer color occlusion.

    Returns rectified depth grid and color; points without a color match are
    invalid for colored fusion rather than silently painted black.
    """
    import cv2
    import time
    start=time.perf_counter()
    dc,cc=cal['depth'],cal['color'];h,w=depth.shape
    if (h,w)!=(dc['height'],dc['width']) or rgb.shape!=(cc['height'],cc['width'],3):raise ValueError('calibration resolution mismatch')
    def K(p):return np.array([[p['fx'],0,p['cx']],[0,p['fy'],p['cy']],[0,0,1]],float)
    kd,kc=K(dc),K(cc);distd=np.asarray(dc.get('distortion',[0]*8),float);distc=np.asarray(cc.get('distortion',[0]*8),float)
    # Cache by calibration values, not object identity: replacing calibration
    # cannot reuse obsolete rays. Keep the independent reference uncached.
    if native_library is not None:
        mx,my=_rectification_maps(w,h,dc['fx'],dc['fy'],dc['cx'],dc['cy'],tuple(distd))
    else:
        mx,my=cv2.initUndistortRectifyMap(kd,distd,None,kd,(w,h),cv2.CV_32FC1)
    if profile is not None:profile['maps_ms']=(time.perf_counter()-start)*1000
    stage=time.perf_counter()
    dep=cv2.remap(np.asarray(depth,np.float32),mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
    observed=np.isfinite(dep)&(dep>0)
    dep[~observed]=0
    dep=cv2.bilateralFilter(dep,5,.02,2)
    dep[~observed]=0  # Denoise measured values; never invent missing depth.
    if profile is not None:profile['remap_filter_ms']=(time.perf_counter()-stage)*1000
    stage=time.perf_counter()
    if native_library is not None:
        import ctypes as C
        from native import ptr
        f=native_library.lj_register_color;f.argtypes=[C.c_void_p]*4+[C.c_int]*4+[C.c_void_p]*4;f.restype=C.c_int
        dcam=np.array([dc[k] for k in ('fx','fy','cx','cy')],np.float64);ccam=np.array([cc[k] for k in ('fx','fy','cx','cy')],np.float64);dist=np.zeros(8,np.float64);dist[:len(distc)]=distc
        transform=np.ascontiguousarray(checked_pose(cal['T_color_depth']),np.float64);source=np.ascontiguousarray(rgb,np.uint8);outdepth=np.empty_like(dep);out=np.empty((h,w,3),np.uint8)
        if f(ptr(dep),ptr(source),ptr(outdepth),ptr(out),w,h,cc['width'],cc['height'],ptr(dcam),ptr(ccam),ptr(dist),ptr(transform))!=0:raise ValueError('native registration rejected calibration')
        if profile is not None:profile['projection_occlusion_ms']=(time.perf_counter()-stage)*1000
        return outdepth,out
    yy,xx=np.indices((h,w));points=np.stack([(xx-dc['cx'])*dep/dc['fx'],(yy-dc['cy'])*dep/dc['fy'],dep],-1).reshape(-1,3)
    T=checked_pose(cal['T_color_depth']);pc=points@T[:3,:3].T+T[:3,3]
    valid=(dep.ravel()>0)&np.isfinite(pc).all(1)&(pc[:,2]>0)
    out=np.zeros((h*w,3),np.uint8);idx=np.flatnonzero(valid)
    if len(idx):
        uv,_=cv2.projectPoints(pc[idx],np.zeros(3),np.zeros(3),kc,distc);uv=np.rint(uv.reshape(-1,2)).astype(int)
        inside=(uv[:,0]>=0)&(uv[:,0]<cc['width'])&(uv[:,1]>=0)&(uv[:,1]<cc['height']);idx=idx[inside];uv=uv[inside]
        pix=uv[:,1]*cc['width']+uv[:,0];zmin=np.full(cc['width']*cc['height'],np.inf);np.minimum.at(zmin,pix,pc[idx,2]);keep=pc[idx,2]<=zmin[pix]+.02;idx=idx[keep];uv=uv[keep]
        out[idx]=rgb[uv[:,1],uv[:,0]]
    matched=np.zeros(h*w,bool);matched[idx]=True;dep.ravel()[~matched]=0
    return dep,out.reshape(h,w,3)
