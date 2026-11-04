"""Copy authoritative tool reports; never infer board acceptance from synthesis."""
from pathlib import Path
import hashlib,json,shutil
root=Path(__file__).resolve().parents[1];out=root/'evidence';out.mkdir(exist_ok=True)
inputs={
 'hls_synthesis.rpt':Path('E:/tmp/liujing_raster_v2_20261024/liujing_raster/solution/syn/report/lj_raster_csynth.rpt'),
 'hls_cosim.rpt':Path('E:/tmp/liujing_raster_v2_20261024/liujing_raster/solution/sim/report/lj_raster_cosim.rpt'),
 'registers.h':Path('E:/tmp/liujing_raster_v2_20261024/liujing_raster/solution/impl/ip/drivers/lj_raster_v1_0/src/xlj_raster_hw.h')}
for name in ['utilization.rpt','timing.rpt','route.rpt','drc.rpt','check_timing.rpt']:
 inputs['integrated_'+name]=Path('E:/tmp/liujing_integrated_v2_20261024/artifacts')/name
for name,p in inputs.items():
 if p.exists():shutil.copyfile(p,out/name)
files={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and not any(x in ('__pycache__','runs','build','evidence','.git') for x in p.relative_to(root).parts)}
(out/'source_manifest.json').write_text(json.dumps({'client_date':'2026-10-24','files':files},indent=2))
print('reports copied:',[name for name in inputs if (out/name).exists()])
