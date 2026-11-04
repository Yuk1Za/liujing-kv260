# full_flow_s2m.tcl —— 单 IP 独立 BD（s2m_vmap + PS），给上板 bring-up 用
#   复用 视觉bitstream制作指南 §四 的模板；差异：本 IP 有 2 个 m_axi master（g0/g1），无 BRAM 口。
set IP_REPO /home/srtp2025_nie/s2m_vmap_hls/proj_vmap_impl/sol1/impl/ip
set PRJ     /home/srtp2025_nie/s2m_bd

create_project s2m_bd $PRJ -part xck26-sfvc784-2LV-c -force
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths $IP_REPO [current_project]
update_ip_catalog -rebuild

create_bd_design design_1
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ultra_ps_e_0
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells zynq_ultra_ps_e_0]

# 坑2：board preset 不开 PL→DDR 的从口；maxihpm1_fpd_aclk 会悬空
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__S_AXI_GP0 {1} CONFIG.PSU__USE__S_AXI_GP1 {1}] [get_bd_cells zynq_ultra_ps_e_0]
# PL 时钟 150 MHz（与最终合并 bit 的请求一致；本 IP 在 7 ns 下 WNS +1.54，150 MHz 仍有 ~1.2 ns 余量）
set_property -dict [list CONFIG.PSU__FPGA_PL0_ENABLE {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {150}] [get_bd_cells zynq_ultra_ps_e_0]

create_bd_cell -type ip -vlnv xilinx.com:hls:s2m_vmap:1.0 s2m_vmap_0

apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins s2m_vmap_0/s_axi_control]
# 坑1：DDR 从口叫 HPC（High Performance Coherent），不是 HP
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/s2m_vmap_0/m_axi_g0"} [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HPC0_FPD]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/s2m_vmap_0/m_axi_g1"} [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HPC1_FPD]
# 坑2b：preset 启用了 HPM1 却没使用 → 该时钟悬空 → [BD 41-758]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins zynq_ultra_ps_e_0/maxihpm1_fpd_aclk]
assign_bd_address

validate_bd_design
save_bd_design
puts "BD_OK"

# 坑3：BD 建好 ≠ 能实现，必须生成 HDL wrapper 并设为顶层
set wr [make_wrapper -files [get_files design_1.bd] -top]
add_files -norecurse $wr
set_property top design_1_wrapper [current_fileset]
update_compile_order -fileset sources_1
puts "WRAPPER_OK top=design_1_wrapper"

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "IMPL_STATUS=[get_property STATUS [get_runs impl_1]]"
open_run impl_1
report_timing_summary -delay_type min_max -file $PRJ/timing.rpt
puts "DONE"
