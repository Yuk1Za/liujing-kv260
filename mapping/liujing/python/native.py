"""Strict numpy/C ABI wrapper. All scene computation executes where this runs."""
from pathlib import Path
import ctypes as C,os
import numpy as np
class Camera(C.Structure):
    _fields_=[('width',C.c_int),('height',C.c_int)]+[(x,C.c_float) for x in ('fx','fy','cx','cy','near_z','far_z')]
    @classmethod
    def from_dict(cls,d):
        return cls(*[d[k] for k,_ in cls._fields_])
    def as_dict(self): return {k:getattr(self,k) for k,_ in self._fields_}
P=C.c_void_p
class RejectedFrame(ValueError):
    def __init__(self,code):self.code=code;super().__init__(f'fusion rejected {code}')
def ptr(a): return a.ctypes.data_as(P)
class Map:
    def __init__(self,capacity=1000000,stride=2,library=None,threads=4):
        self.closed=True
        path=Path(library or os.environ.get('LIUJING_LIBRARY',Path(__file__).resolve().parents[1]/'build/libliujing.so'))
        self.lib=C.CDLL(str(path));self.closed=True
        signatures={'lj_create':([C.c_int,C.c_int],P),'lj_destroy':([P],None),
            'lj_count':([P],C.c_uint32),'lj_version':([P],C.c_uint64),'lj_capacity_drops':([P],C.c_uint64),
            'lj_integrate':([P,P,P,P,P,C.c_double],C.c_int),'lj_render':([P,P,P,P,P,P,C.c_int],C.c_int),
            'lj_copy':([P,P,C.c_int],C.c_int),'lj_set':([P,P,C.c_int],C.c_int),
            'lj_prepare':([P,P,P,P,P,P,C.c_int],C.c_int),'lj_threads':([C.c_int],C.c_int)}
        for name,(args,ret) in signatures.items():
            f=getattr(self.lib,name);f.argtypes=args;f.restype=ret
        if hasattr(self.lib,'lj_prepare_packed'):
            self.lib.lj_prepare_packed.argtypes=signatures['lj_prepare'][0];self.lib.lj_prepare_packed.restype=C.c_int
        if hasattr(self.lib,'lj_prepare_checked'):
            self.lib.lj_prepare_checked.argtypes=signatures['lj_prepare'][0]+[C.c_int,C.c_uint64,C.c_int];self.lib.lj_prepare_checked.restype=C.c_int
        if hasattr(self.lib,'lj_integrate_associated'):
            self.lib.lj_integrate_associated.argtypes=[P,P,P,P,P,C.c_double,P,P,C.c_uint64];self.lib.lj_integrate_associated.restype=C.c_int
        if hasattr(self.lib,'lj_unpack_color'):
            self.lib.lj_unpack_color.argtypes=[P,P,C.c_int];self.lib.lj_unpack_color.restype=None
        self.threads=self.lib.lj_threads(threads);self.handle=self.lib.lj_create(capacity,stride)
        if not self.handle: raise ValueError('invalid map capacity/stride')
        self.closed=False
    def close(self):
        if not self.closed:self.lib.lj_destroy(self.handle);self.closed=True
    def __del__(self): self.close()
    @property
    def count(self): return self.lib.lj_count(self.handle)
    @property
    def version(self): return self.lib.lj_version(self.handle)
    @property
    def capacity_drops(self): return self.lib.lj_capacity_drops(self.handle)
    def integrate(self,depth,rgb,cam,T,stamp,association=None):
        d=np.ascontiguousarray(depth,dtype=np.float32);im=np.ascontiguousarray(rgb,dtype=np.uint8);T=np.ascontiguousarray(T,dtype=np.float32)
        if d.shape!=(cam.height,cam.width) or im.shape!=(cam.height,cam.width,3) or T.shape!=(4,4):raise ValueError('frame dimensions')
        if association is None:result=self.lib.lj_integrate(self.handle,ptr(d),ptr(im),C.byref(cam),ptr(T),stamp)
        else:
            rd,ri,version=association;rd=np.ascontiguousarray(rd,dtype=np.float32);ri=np.ascontiguousarray(ri,dtype=np.int32)
            if rd.shape!=d.shape or ri.shape!=d.shape:raise ValueError('association dimensions')
            result=self.lib.lj_integrate_associated(self.handle,ptr(d),ptr(im),C.byref(cam),ptr(T),stamp,ptr(rd),ptr(ri),version)
        if result<0:raise RejectedFrame(result)
        return result
    def render(self,cam,T,mode=0):
        T=np.ascontiguousarray(T,dtype=np.float32)
        if T.shape!=(4,4):raise ValueError('pose shape')
        shape=(cam.height,cam.width);d=np.empty(shape,np.float32);ids=np.empty(shape,np.int32);rgb=np.empty(shape,np.uint32)
        result=self.lib.lj_render(self.handle,C.byref(cam),ptr(T),ptr(d),ptr(ids),ptr(rgb),mode)
        if result<0:raise ValueError(f'render rejected {result}')
        image=None if mode==3 else self.unpack(rgb)
        return image,d,ids
    def unpack(self,rgb):
        if hasattr(self.lib,'lj_unpack_color'):
            im=np.empty((*rgb.shape,3),np.uint8);self.lib.lj_unpack_color(ptr(rgb),ptr(im),rgb.size);return im
        return np.stack([rgb&255,(rgb>>8)&255,(rgb>>16)&255],axis=-1).astype(np.uint8)
    def surfels(self):
        a=np.empty((self.count,16),np.float32)
        if self.lib.lj_copy(self.handle,ptr(a),len(a))<0:raise RuntimeError('snapshot size changed')
        return a
    def restore(self,a):
        a=np.ascontiguousarray(a,dtype=np.float32)
        if a.ndim!=2 or a.shape[1]!=16:raise ValueError('surfel shape')
        if self.lib.lj_set(self.handle,ptr(a),len(a))<0:raise ValueError('invalid map')
