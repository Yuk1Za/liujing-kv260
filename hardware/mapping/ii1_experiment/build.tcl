open_project ii1_raster
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
