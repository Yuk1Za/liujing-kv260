# run_impl.tcl —— 从 C 综合一路走到 Vivado 实现，拿"落地后"的真实 LUT/DSP/时序
# 目标器件与合并 bit 一致（xck26-sfvc784-2LV-c），时钟按合并 bit 现状 143 MHz（7.0 ns）
open_project -reset proj_vmap_impl
set_top s2m_vmap
add_files s2m_vmap.cpp
open_solution -reset sol1
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csynth_design
export_design -flow impl -rtl verilog -format ip_catalog
exit
