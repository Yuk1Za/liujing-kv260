# run_csim.tcl —— 功能仿真（需要能用的 g++；Ubuntu 24.04 上 Vitis 2023.2 自带 gcc 会挂，见项目记忆）
open_project -reset proj_vmap_c
set_top s2m_vmap
add_files s2m_vmap.cpp
add_files -tb tb_s2m_vmap.cpp
open_solution -reset sol1
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csim_design
exit
