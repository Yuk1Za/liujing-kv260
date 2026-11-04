"""Single overlay manager owns uncached buffers for native opaque_v1 transport."""
import json, os, hashlib
from pathlib import Path

def allocate_native(overlay, run, bit):
    import numpy as np
    from pynq import allocate
    import pynq._3rdparty.xrt as xrt
    ip=overlay.ip_dict['lj_raster_0']
    registers={}
    sizes={'camera_surfels':40000*64,'offsets':1201*4,'indices':100000*4,
           'rgb':640*480*4,'depth':640*480*4,'ids':640*480*4}
    for name in (*sizes,'width','height','fx','fy','cx','cy','near_z','far_z','debug'):
        all_regs=ip['registers']
        r=all_regs.get(name,all_regs.get(name+'_1'))
        if r is None:raise ValueError('missing register '+name)
        registers[name]=int(r['address_offset'])
        if name in sizes:
            if int(r['size'])==64:registers[name+'_high']=registers[name]+4
            elif int(r['size'])==32 and name+'_2' in all_regs:
                registers[name+'_high']=int(all_regs[name+'_2']['address_offset'])
                if registers[name+'_high']!=registers[name]+4:raise ValueError('noncontiguous pointer registers')
            else:raise ValueError('opaque_v1 pointer width')
    from instrument_renderer import NAMES
    for name in NAMES:
        registers['metric_'+name]=int(ip['registers']['metric_'+name]['address_offset'])
    buffers=[];entries={}
    for name,size in sizes.items():
        b=allocate(shape=(size,),dtype=np.uint8,cacheable=False)
        b.native_export_fd=xrt.xclExportBO(b.device.handle,b.bo)
        if b.native_export_fd<0:raise RuntimeError('DMA-BUF export failed')
        buffers.append(b);entries[name]={'address':int(b.physical_address),'nbytes':size,'fd':int(b.native_export_fd)}
    path=run/'native_dma.json'
    path.write_text(json.dumps({'contract':'opaque_v1','transport':'inherited_dma_buf_v1','metrics_contract':'passive_axi_metrics_v1','cacheable':False,'owner_pid':os.getpid(),
        'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'bit_sha256':hashlib.sha256(Path(bit).read_bytes()).hexdigest(),
        'control':{'address':int(ip['phys_addr']),'nbytes':int(ip['addr_range'])},
        'registers':registers,'buffers':entries},indent=2))
    return buffers,path
