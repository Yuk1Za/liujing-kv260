"""One allocated scratch page; never touches PL or an existing client's memory."""
import os,struct,subprocess
from pathlib import Path
from pynq import allocate
import pynq._3rdparty.xrt as xrt
root=Path(__file__).resolve().parents[1]
b=allocate(shape=(4096,),dtype='u1',cacheable=False)
fd=xrt.xclExportBO(b.device.handle,b.bo)
try:
    if fd<0:raise RuntimeError('export failed')
    b[:4]=list(struct.pack('<I',0x12345678))
    subprocess.run([str(root/'native_v2_build/lj2_dma_probe'),str(fd)],pass_fds=(fd,),check=True)
    assert bytes(b[4:8])==struct.pack('<I',0x89abcdef)
    print('PASS manager observes native child writes without Python per-frame conversion')
finally:
    if fd>=0:os.close(fd)
    b.freebuffer()
