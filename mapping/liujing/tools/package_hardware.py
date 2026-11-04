"""Preserve completed implementation and exact input IPs; make board-only archive."""
from pathlib import Path
import hashlib,json,shutil,tarfile,zipfile
root=Path(__file__).resolve().parents[1];build=Path('E:/tmp/liujing_integrated_v2_20261024');hls=Path('E:/tmp/liujing_raster_v2_20261024');out=root/'artifacts';out.mkdir(exist_ok=True)
for name in ('liujing.bit','liujing.hwh','timing.rpt','drc.rpt','check_timing.rpt'):
 if not (build/'artifacts'/name).is_file():raise SystemExit('Implementation not complete: '+name)
for name in ('raster.cpp','raster.hpp','tb.cpp','build.tcl'):
 if (root/'hardware'/name).read_bytes()!=(hls/'hardware'/name).read_bytes():raise SystemExit('HLS input differs from current source: '+name)
for name in ('liujing.bit','liujing.hwh'):shutil.copyfile(build/'artifacts'/name,out/name)
for label,ip in [('c16',Path('E:/tmp/losam_c16_clean_20261022/c16_hls/solution/impl/ip')),('raster',hls/'liujing_raster/solution/impl/ip')]:
 with zipfile.ZipFile(out/(label+'_ip.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for file in sorted(ip.rglob('*')):
   if file.is_file():z.write(file,file.relative_to(ip).as_posix())
shutil.copyfile(build/'platform.tcl',out/'implemented_platform.tcl')
manifest={'part':'xck26-sfvc784-2LV-c','clock_target_mhz':100,'tool':'Vivado/Vitis 2026.1','scope':'implemented hardware; board acceptance recorded separately','sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.iterdir()) if f.suffix in ('.bit','.hwh','.zip','.tcl')},'hls_input_sha256':{name:hashlib.sha256((hls/'hardware'/name).read_bytes()).hexdigest() for name in ('raster.cpp','raster.hpp','tb.cpp','build.tcl')}}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
with tarfile.open(root/'evidence/board_hardware.tgz','w:gz') as tar:
 for name in ('liujing.bit','liujing.hwh','manifest.json'):tar.add(out/name,arcname='liujing/artifacts/'+name)
print(json.dumps(manifest,indent=2));print('board archive bytes:',(root/'evidence/board_hardware.tgz').stat().st_size)
