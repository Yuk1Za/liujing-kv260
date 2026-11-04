open_project -reset lk_cosim
set_top lk_lvisam
add_files lk_lvisam.cpp
add_files -tb lk_tb.cpp
open_solution -reset sol1
set_part {xck26-sfvc784-2LV-c}
create_clock -period 5.0 -name default
csynth_design
cosim_design -rtl verilog -trace_level none
exit
