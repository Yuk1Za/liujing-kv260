open_project wave_ip
set_top fusion_frame
add_files wave_frame.cpp -cflags "-std=c++17 -ffp-contract=off"
add_files -tb frame_tb.cpp -cflags "-std=c++17 -ffp-contract=off -DNATIONAL_NARROW -DNATIONAL_CYCLE_COUNTERS -DNATIONAL_HOST_NORMALS -DNATIONAL_INTEGER_RGB -DNATIONAL_CHAIN_WAVE"
open_solution kv260 -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10
config_interface -m_axi_alignment_byte_size 64
csim_design
csynth_design
export_design -format ip_catalog
exit
