cd /home/srtp2025_nie/fusion_probe/work/src
open_project -reset proj
add_files raster_fusion.cpp
set_top lj_raster
open_solution -reset solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10.000 -name default
config_interface -m_axi_addr64=false
csynth_design
export_design -format ip_catalog -vendor "xilinx.com" -library "hls" -version "1.0" -display_name "lj_raster"
exit
