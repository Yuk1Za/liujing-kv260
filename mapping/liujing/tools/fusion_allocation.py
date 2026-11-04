"""Manager-owned resident DDR table and bounded association buffers."""
import hashlib,json,os
from pathlib import Path

def allocate_fusion(overlay,run,bit,contract='fusion_association_v1'):
    import numpy as np
    from pynq import allocate
    import pynq._3rdparty.xrt as xrt
    ip=overlay.ip_dict['lj_associate_0'];registers={}
    if contract not in ('fusion_association_v1','fusion_variable_v1','fusion_cached_v2'):raise ValueError('unknown fusion contract')
    sizes=({'table':2097152*32,'blocks':1048576*128} if contract in ('fusion_variable_v1','fusion_cached_v2') else {'table':2097152*128})
    sizes.update(groups=64000*32,queries=64000*32,matches=64000*4,stats=8*4)
    if contract=='fusion_cached_v2':sizes['updates']=4*1024*1024
    dual=any(key in ip['registers'] for key in ('table_b','table_b_1','table_b_r','table_b_r_1'))
    pointers=list(sizes)+(['table_b'] if dual else [])
    scalars=['group_count','query_count','table_mask','epoch']+(['block_capacity'] if contract in ('fusion_variable_v1','fusion_cached_v2') else [])
    if contract=='fusion_cached_v2':scalars+=['operation','update_words']
    for name in (*pointers,*scalars):
        all_regs=ip['registers'];aliases=[key for key in (name,name+'_1',name+'_r',name+'_r_1') if key in all_regs]
        if len(aliases)!=1:raise ValueError('missing or ambiguous association register '+name)
        key=aliases[0];r=all_regs[key]
        registers[name]=int(r['address_offset'])
        if name in pointers:
            if int(r['size'])==64:registers[name+'_high']=registers[name]+4
            elif int(r['size'])==32 and key.endswith('_1') and key[:-2]+'_2' in all_regs:
                registers[name+'_high']=int(all_regs[key[:-2]+'_2']['address_offset'])
                if registers[name+'_high']!=registers[name]+4:raise ValueError('association pointer register layout')
            else:raise ValueError('association pointer width')
    buffers=[];entries={}
    for name,size in sizes.items():
        b=allocate(shape=(size,),dtype=np.uint8,cacheable=False)
        b.native_export_fd=xrt.xclExportBO(b.device.handle,b.bo)
        if b.native_export_fd<0:raise RuntimeError('association DMA-BUF export failed')
        buffers.append(b);entries[name]={'address':int(b.physical_address),'nbytes':size,'fd':int(b.native_export_fd)}
    path=run/'fusion_dma.json'
    path.write_text(json.dumps({'contract':contract,'transport':'inherited_dma_buf_v1','cacheable':False,'owner_pid':os.getpid(),
        'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'bit_sha256':hashlib.sha256(Path(bit).read_bytes()).hexdigest(),
        'control':{'address':int(ip['phys_addr']),'nbytes':int(ip['addr_range'])},'registers':registers,'buffers':entries},indent=2))
    return buffers,path
