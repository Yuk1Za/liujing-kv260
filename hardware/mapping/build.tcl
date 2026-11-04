# Run from an ASCII-only dedicated build directory containing hardware/.
open_project liujing_raster
set_top lj_raster
add_files hardware/raster.cpp -cflags "-std=c++14 -Ihardware"
add_files -tb hardware/tb.cpp -cflags "-std=c++14 -Ihardware"
open_solution solution -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default
csim_design
csynth_design
cosim_design -rtl verilog
export_design -format ip_catalog
exit
