"""Separate HLS experiment; never changes the measured baseline kernel."""
from pathlib import Path
import shutil,hashlib,json
root=Path(__file__).resolve().parents[1];out=root/'hardware/ii1_experiment';out.mkdir(exist_ok=False);build=Path('E:/tmp/liujing_raster_ii1_20261024');build.mkdir(exist_ok=False)
text=(root/'hardware/raster.cpp').read_text();needle=' float zbuf[256],sw[256],sr[256],sg[256],sb[256]; int ibuf[256];'
assert text.count(needle)==1
text=text.replace(needle,needle+'\n'+''.join('#pragma HLS BIND_STORAGE variable='+name+' type=ram_2p impl=bram\n' for name in ('zbuf','ibuf','sw','sr','sg','sb')))
needle='    for(int fragment_index=0;fragment_index<count;fragment_index++){\n#pragma HLS PIPELINE'
assert text.count(needle)==1
proof='''    // Within this rectangle p is strictly increasing: row advances are
    // 16-(x1-x0), at least one. Each pixel is touched once per inner loop.
    // LOOP_FLATTEN off keeps successive surfels/passes outside this contract;
    // their pipelines drain before the next rectangle can update the same p.
'''
text=text.replace(needle,proof+'    for(int fragment_index=0;fragment_index<count;fragment_index++){\n#pragma HLS LOOP_FLATTEN off\n#pragma HLS PIPELINE II=1\n'+''.join('#pragma HLS DEPENDENCE variable='+name+' inter false\n' for name in ('zbuf','ibuf','sw','sr','sg','sb')))
(out/'raster.cpp').write_text(text)
for source,target in [(root/'hardware/raster.hpp','raster.hpp'),(root/'hardware/ii1_tb.cpp','tb.cpp'),(root/'src/core.cpp','core.cpp'),(root/'src/core.hpp','core.hpp')]:shutil.copyfile(source,out/target)
(out/'build.tcl').write_text('''open_project ii1_raster
set_top lj_raster
add_files raster.cpp -cflags "-std=c++17 -I."
add_files -tb core.cpp -cflags "-std=c++17 -I."
add_files -tb tb.cpp -cflags "-std=c++17 -I."
open_solution solution -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default
csim_design
csynth_design
cosim_design -rtl verilog
export_design -format ip_catalog
exit
''')
for file in out.iterdir():shutil.copyfile(file,build/file.name)
(out/'manifest.json').write_text(json.dumps({'status':'EXPERIMENT_NOT_BOARD_VALIDATED','baseline_sha256':hashlib.sha256((root/'hardware/raster.cpp').read_bytes()).hexdigest(),'changes':'dual-port tile caches; experimental inter-iteration dependence assertions requiring RTL checker validation; unchanged FP32 operations','build':str(build)},indent=2));print(build)
