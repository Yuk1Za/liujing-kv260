open_project /home/srtp2025_nie/fusion_probe/fe_csim/proj -reset
set_top fpga_feature_extract_hw
add_files /home/srtp2025_nie/fusion_probe/fe_csim/fpga_feature_extract_hw.cpp -cflags "-I/home/srtp2025_nie/fusion_probe/fe_csim"
add_files /home/srtp2025_nie/fusion_probe/fe_csim/registration_fpga.cpp -cflags "-I/home/srtp2025_nie/fusion_probe/fe_csim"
add_files -tb /home/srtp2025_nie/fusion_probe/fe_csim/tb.cpp -cflags "-I/home/srtp2025_nie/fusion_probe/fe_csim"
open_solution -reset sol -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 5 -name default
csim_design -argv "/home/srtp2025_nie/fusion_probe/fe_csim/in.bin /home/srtp2025_nie/fusion_probe/fe_csim/out 3.138138618 9.428035711"
exit
