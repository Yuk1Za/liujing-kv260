# csynth_lk.tcl —— lk_lvisam 的综合脚本（与既有视觉 IP 同一套环境：xck26-sfvc784-2LV-c, 5.0 ns）
open_project -reset p2w
set_top lk_lvisam
add_files lk2.cpp
open_solution -reset sol1
set_part {xck26-sfvc784-2LV-c}
create_clock -period 5.0 -name default
csynth_design
exit
