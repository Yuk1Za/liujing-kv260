open_project -reset ip_lkv2/proj
set_top sparse_lk
add_files /home/srtp2025_nie/lk_lvisam/lk2_ip.cpp -cflags "-D__SDSVHLS__ -std=c++0x"
open_solution -reset sol
set_part xck26-sfvc784-2LV-c
create_clock -period 5.0 -name default
csynth_design
export_design -format ip_catalog -description "sparse LK, LVI-SAM faithful (21x21 pyramid, 30it/0.01px)" -vendor user -library hls -version 1.0 -output sparse_lk_lkv2.zip
exit
