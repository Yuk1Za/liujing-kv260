set SRC /home/srtp2025_nie/slk_hls
open_project -reset slk_proj
set_top sparse_lk
add_files $SRC/sparse_lk.cpp -cflags "-I$SRC -D__SDSVHLS__ -std=c++0x"
open_solution -reset slk_sol
set_part xck26-sfvc784-2LV-c
create_clock -period 5.0 -name default
csynth_design
export_design -format ip_catalog -vendor "user" -library "hls" -version "1.0" -description "sparse LK optical flow"
exit
