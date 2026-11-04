"""PYNQ raster driver. No automatic overlay download or CPU fallback.

Projection and tile-list preparation remain on PS in this first accelerator.
The timed render path includes preparation, flush, PL and invalidate/conversion.
"""
import ctypes as C,time,struct,json,hashlib,threading
from pathlib import Path
import numpy as np
from native import ptr
from hardware_fault import require_healthy,latch_timeout
REG={'camera_surfels':0x10,'offsets':0x1c,'indices':0x28,'depth':0x34,'ids':0x40,'rgb':0x4c,'width':0x58,'height':0x60,'fx':0x68,'fy':0x70,'cx':0x78,'cy':0x80,'near_z':0x88,'far_z':0x90,'blend_band':0x98}
METRICS=('signature','cycles','surfel_read_beats','surfel_read_wait','surfel_read_backpressure','surfel_address_wait','index_read_beats','index_read_wait','output_write_beats','output_write_backpressure','output_address_wait','index_address_wait')
class Raster:
    def __init__(self,overlay,surfel_capacity=100000,index_capacity=4000000,numeric_contract='mixed'):
        require_healthy()
        from pynq import allocate
        if 'lj_raster_0' not in overlay.ip_dict:raise ValueError('overlay lacks LiuJing renderer')
        if numeric_contract not in ('mixed','qrcp_v1'):raise ValueError('unknown numeric contract')
        self.numeric_contract=numeric_contract
        self.descriptor_layout='float_bounds'
        self.dma_alignment=1
        self.metrics_contract=None
        if numeric_contract=='qrcp_v1':
            bit=Path(overlay.bitfile_name);manifest=json.loads(bit.with_name('manifest.json').read_text())
            if manifest.get('numeric_contract')!=numeric_contract or manifest['sha256'][bit.name]!=hashlib.sha256(bit.read_bytes()).hexdigest():raise ValueError('numeric contract / bit manifest mismatch')
            self.descriptor_layout=manifest.get('descriptor_layout','float_bounds')
            if self.descriptor_layout not in ('float_bounds','int_bounds_v1'):raise ValueError('unsupported descriptor layout')
            self.dma_alignment=manifest.get('dma_alignment',1)
            if self.dma_alignment not in (1,64):raise ValueError('unsupported DMA alignment contract')
            self.metrics_contract=manifest.get('metrics_contract')
            if self.metrics_contract not in (None,'passive_axi_metrics_v1'):raise ValueError('unsupported hardware metrics contract')
        self.overlay=overlay;self.ip=overlay.lj_raster_0;self.failed=False;self.surfel_capacity=surfel_capacity;self.index_capacity=index_capacity
        # Register layout must match generated HWH, not just a similarly named core.
        regs=overlay.ip_dict['lj_raster_0'].get('registers',{})
        self.mode_offset=next((int(v['address_offset']) for k,v in regs.items() if k.lower()=='mode'),None)
        for name,offset in REG.items():
            matches=[v for k,v in regs.items() if k.lower() in (name,name+'_1')]
            if not matches or int(matches[0]['address_offset'])!=offset:raise ValueError('HWH register mismatch: '+name)
        if self.metrics_contract:
            for i,name in enumerate(METRICS):
                entry=regs.get('metric_'+name,{})
                if int(entry.get('address_offset',-1))!=0xb0+4*i:raise ValueError('HWH counter register mismatch: '+name)
            if self.ip.read(0xb0)!=0x4c4a0101:raise RuntimeError('hardware metrics signature mismatch')
        self.buffers={}
        try:
            for name,shape,dtype in [('camera_surfels',(surfel_capacity,16),'float32'),('offsets',(1201,),'uint32'),('indices',(index_capacity,),'uint32'),('depth',(307200,),'float32'),('ids',(307200,),'int32'),('rgb',(307200,),'uint32')]:
                self.buffers[name]=allocate(shape=shape,dtype=dtype,cacheable=True)
                if int(self.buffers[name].physical_address)%self.dma_alignment:raise ValueError('DMA allocation violates hardware alignment contract: '+name)
        except Exception:
            self.close();raise
        self.timing={};self.sequence=0;self.last_job=None;self.call_lock=threading.Lock()
    def render(self,m,cam,T,mode=0,frame_id=None):
        if not self.call_lock.acquire(blocking=False):raise RuntimeError('renderer already has an active task; bounded scheduler must serialize submissions')
        try:return self._render(m,cam,T,mode,frame_id)
        finally:self.call_lock.release()
    def _render(self,m,cam,T,mode=0,frame_id=None):
        start=time.perf_counter()
        require_healthy()
        if self.failed:raise RuntimeError('accelerator previously timed out; reset outside active pipeline')
        if mode not in (0,1,2,3) or (mode==3 and self.mode_offset is None):raise ValueError('unsupported PL render mode')
        version=m.version;count=m.count
        if count>self.surfel_capacity:raise ValueError('accelerator surfel capacity exceeded; no silent clipping')
        if cam.width<1 or cam.height<1 or cam.width>640 or cam.height>480:raise ValueError('render dimensions')
        T=np.ascontiguousarray(T,dtype=np.float32)
        if T.shape!=(4,4):raise ValueError('pose shape')
        b=self.buffers
        if self.numeric_contract=='qrcp_v1':
            if not hasattr(m.lib,'lj_validate_qrcp_normal_reciprocal'):raise RuntimeError('CPU library lacks reciprocal domain validation; rebuild before qrcp tasks')
            f=m.lib.lj_validate_qrcp_normal_reciprocal;f.argtypes=[C.c_void_p];f.restype=C.c_int
            if f(m.handle)!=0:raise ValueError('qrcp_v1 requires normal finite radius squared and reciprocal; unsupported input rejected')
        if not hasattr(m.lib,'lj_prepare_checked'):raise RuntimeError('CPU library lacks atomic checked export; rebuild before submitting hardware tasks')
        refs=m.lib.lj_prepare_checked(m.handle,C.byref(cam),ptr(T),ptr(b['camera_surfels']),ptr(b['offsets']),ptr(b['indices']),self.index_capacity,self.surfel_capacity,version,int(self.descriptor_layout=='int_bounds_v1'))
        if refs==-5:raise RuntimeError('map version changed during task preparation; no hardware submission')
        if refs<0:raise ValueError('projection/binning failed '+str(refs))
        self.sequence+=1
        self.last_job=dict(sequence=self.sequence,frame_id=self.sequence if frame_id is None else frame_id,map_version=version,numeric_contract=self.numeric_contract,descriptor_layout=self.descriptor_layout,mode=mode,view=T.tolist(),completed=False)
        if mode==1:b['camera_surfels'][:count,7:10]=127.5*(b['camera_surfels'][:count,3:6]+1)
        elif mode==2:
            conf=np.minimum(1.,b['camera_surfels'][:count,10]/8);b['camera_surfels'][:count,7]=255*(1-conf);b['camera_surfels'][:count,8]=255*conf;b['camera_surfels'][:count,9]=32
        prepared=time.perf_counter()
        tiles=((cam.width+15)//16)*((cam.height+15)//16)
        # Verified PYNQ slices retain BO offset and pass their own nbytes to flush.
        b['camera_surfels'][:max(1,count)].flush();b['offsets'][:tiles+1].flush()
        if refs:b['indices'][:refs].flush()
        for key,buf in b.items():
            address=int(buf.physical_address);self.ip.write(REG[key],address&0xffffffff);self.ip.write(REG[key]+4,address>>32)
        for key in ('width','height'):self.ip.write(REG[key],getattr(cam,key))
        if self.mode_offset is not None:self.ip.write(self.mode_offset,3 if mode==3 else 0)
        for key in ('fx','fy','cx','cy','near_z','far_z','blend_band'):
            v=.01 if key=='blend_band' else getattr(cam,key);self.ip.write(REG[key],struct.unpack('<I',struct.pack('<f',v))[0])
        self.ip.read(0);submitted=time.perf_counter();self.ip.write(0,1)
        deadline=submitted+10
        while not self.ip.read(0)&2:
            if time.perf_counter()>deadline:
                self.failed=True
                latch_timeout(self.last_job)
                raise TimeoutError('PL raster AP_DONE timeout; DMA buffers retained; reboot required before further acceptance')
            time.sleep(.00005)
        finished=time.perf_counter()
        hardware={name:int(self.ip.read(0xb0+4*i)) for i,name in enumerate(METRICS)} if self.metrics_contract else {}
        if hardware and hardware['signature']!=0x4c4a0101:raise RuntimeError('hardware metrics signature changed during task')
        shape=(cam.height,cam.width);n=cam.height*cam.width
        for key in (('depth','ids') if mode==3 else ('depth','ids','rgb')):b[key][:n].invalidate()
        packed=np.asarray(b['rgb'][:n]).reshape(shape);im=m.unpack(packed) if mode!=3 else None;d=np.array(b['depth'][:n]).reshape(shape);ids=np.array(b['ids'][:n]).reshape(shape);end=time.perf_counter()
        self.timing={'prepare_ms':(prepared-start)*1000,'transfer_submit_ms':(submitted-prepared)*1000,'pl_wait_ms':(finished-submitted)*1000,'readback_convert_ms':(end-finished)*1000,'total_ms':(end-start)*1000,'tile_references':refs}
        self.timing.update({'hw_'+name:value for name,value in hardware.items() if name!='signature'})
        self.last_job.update(completed=True,total_ms=self.timing['total_ms'],hardware=hardware)
        return im,d,ids
    def close(self):
        # Never release buffers while a timed-out master might still access them.
        if getattr(self,'failed',False):return
        for b in getattr(self,'buffers',{}).values():b.freebuffer()
        self.buffers={}
