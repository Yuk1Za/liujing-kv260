"""In-memory C16 integrated-IP check, using the existing HLS analytic pattern.

No overlay reload, captured data, files containing points, or localization claim.
"""
import hashlib,struct,time
import numpy as np
from pynq import allocate
class C16Probe:
    def __init__(self,overlay):
        self.ip=overlay.fpga_feature_extract_0;self.rm=self.ip.register_map;self.buffers={}
        for key,n,dtype,dim in [('input',115200,'<f4',4),('point',28800,'<f4',4),('range',28800,'<f4',1),('lessflat',28800,'u1',1),('ground',28800,'u1',1),('flat',384,'<i2',2),('sharp',192,'<i2',2),('lesssharp',1920,'<i2',2)]:
            b=allocate(shape=(n,dim) if dim>1 else (n,),dtype=dtype);self.buffers[key]=b
            reg='laserCloudInArray' if key=='input' else 'rangeimage_'+key
            setattr(self.rm,reg+'_1',int(b.physical_address)&0xffffffff);setattr(self.rm,reg+'_2',int(b.physical_address)>>32)
        a=-np.pi+(np.arange(1800)+.5)*2*np.pi/1800
        vertical=np.deg2rad(-15+2*np.arange(16));radius=10+.5*np.sin(7*a)
        p=np.zeros((1800,16,4),'<f4');p[:,:,0]=(radius*np.cos(a))[:,None];p[:,:,1]=(radius*np.sin(a))[:,None];p[:,:,2]=radius[:,None]*np.tan(vertical)[None,:]
        self.points=p.reshape(-1,4)
    def run(self,single_ring=False):
        points=self.points[::16] if single_ring else self.points
        b=self.buffers;b['input'][:len(points)]=points;b['input'][len(points)]=points[-1];b['input'].flush()
        for key,buffer in b.items():
            if key!='input':buffer[:]=0;buffer.flush()
        self.rm.cloudSize=len(points)+1
        self.rm.start_point_ori=struct.unpack('<I',struct.pack('<f',-np.pi))[0];self.rm.end_point_ori=struct.unpack('<I',struct.pack('<f',np.pi))[0]
        start=time.monotonic();self.ip.write(0,1)
        while not self.ip.read(0)&2:
            if time.monotonic()-start>5:
                from hardware_fault import latch_timeout
                latch_timeout({'reason':'C16 AP_DONE timeout during analytic integration check'})
                raise RuntimeError('C16 timeout; reboot required')
            time.sleep(.00005)
        elapsed=(time.monotonic()-start)*1000
        for key,buffer in b.items():
            if key!='input':buffer.invalidate()
        grid=np.asarray(b['point']);valid=np.linalg.norm(grid[:,:3],axis=1)>.7;indices=np.flatnonzero(valid);rings=indices%16
        angle=np.rad2deg(np.arctan2(grid[valid,2],np.linalg.norm(grid[valid,:2],axis=1)))
        if not np.isfinite(grid).all() or np.max(np.abs(angle-(-15+2*rings)))>.1:raise RuntimeError('C16 analytic geometry mismatch')
        coverage=np.bincount(rings,minlength=16)
        if single_ring:
            if coverage[0]<1700 or np.any(coverage[1:]):raise RuntimeError('C16 stale ring contents')
        elif np.any(coverage<1700):raise RuntimeError('C16 incomplete ring coverage')
        for key in ('flat','sharp','lesssharp'):
            locations=np.asarray(b[key]);active=locations[np.any(locations!=0,axis=1)]
            if np.any(active<0) or np.any(active[:,0]>=16) or np.any(active[:,1]>=1800):raise RuntimeError('C16 invalid feature index')
            if np.any(~valid[active[:,1].astype(int)*16+active[:,0]]):raise RuntimeError('C16 feature on empty cell')
        return {'kernel_call_ms':elapsed,'occupied':len(indices),'ring_coverage':coverage.tolist(),'hash':hashlib.sha256(b''.join(np.asarray(v).tobytes() for k,v in b.items() if k!='input')).hexdigest()}
