open_project -reset work/hls/lj_raster_proj
add_files raster_fusion.cpp -cflags "-I."
set_top lj_raster
open_solution -reset solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10.000 -name default
config_interface -m_axi_addr64=false
csynth_design
export_design -format ip_catalog -vendor "xilinx.com" -library "hls" -version "1.0" -display_name "lj_raster"
exit
