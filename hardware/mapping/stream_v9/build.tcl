# Reproduction of the four-lane int_bounds_v1 candidate, 100 MHz.
open_project liujing_stream
set_top lj_raster
add_files raster.cpp -cflags "-std=c++17 -ffp-contract=off -DLIUJING_LANES=4 -I."
add_files -tb core.cpp -cflags "-std=c++17 -ffp-contract=off -I."
add_files -tb tb.cpp -cflags "-std=c++17 -ffp-contract=off -I."
open_solution solution -flow_target vivado
# All six host DMA base addresses must satisfy this contract. It avoids
# synthesizing two aligned 512-bit loads for one 64-byte surfel.
# AMD UG1399 config_interface: Vivado IP default alignment is only one byte.
config_interface -m_axi_alignment_byte_size 64
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default
csim_design
csynth_design
cosim_design -rtl verilog -random_stall
export_design -format ip_catalog
exit
