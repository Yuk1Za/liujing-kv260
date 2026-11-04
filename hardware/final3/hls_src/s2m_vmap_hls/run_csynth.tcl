# run_csynth.tcl —— v0 探底：只 csynth，不 impl。目标 143 MHz（与合并 bit 现状同频）
open_project -reset proj_vmap
set_top s2m_vmap
add_files s2m_vmap.cpp
add_files -tb tb_s2m_vmap.cpp
open_solution -reset sol1
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csynth_design
report_utilization -hierarchical -hierarchical_depth 2
exit
