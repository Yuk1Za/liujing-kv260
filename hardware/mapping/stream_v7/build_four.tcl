# Reproduction of the four-lane int_bounds_v1 candidate, 100 MHz.
open_project lanes_4
set_top lj_raster
add_files raster.cpp -cflags "-std=c++17 -ffp-contract=off -DLIUJING_LANES=4 -I."
add_files -tb core.cpp -cflags "-std=c++17 -ffp-contract=off -I."
add_files -tb tb.cpp -cflags "-std=c++17 -ffp-contract=off -I."
open_solution solution -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default
csim_design
csynth_design
cosim_design -rtl verilog -random_stall
export_design -format ip_catalog
exit
