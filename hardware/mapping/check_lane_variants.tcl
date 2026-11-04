# Standalone C functional check. This does not certify HLS scheduling or area.
foreach lanes {1 2 4} {
 open_project lanes_$lanes
 set_top lj_raster
 add_files raster.cpp -cflags "-std=c++17 -ffp-contract=off -DLIUJING_LANES=$lanes -I."
 add_files -tb core.cpp -cflags "-std=c++17 -ffp-contract=off -I."
 add_files -tb tb.cpp -cflags "-std=c++17 -ffp-contract=off -I."
 open_solution solution -flow_target vivado
 set_part {xck26-sfvc784-2LV-c}
 create_clock -period 10 -name default
 csim_design
 close_project
}
exit
