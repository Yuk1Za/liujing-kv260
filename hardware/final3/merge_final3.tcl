# ============================================================
#  merge_fusion @150 MHz：**雷达 feature_extract + Shi-Tomasi + MIN_DIST NMS**
#  —— 为「视觉与雷达必须同一颗 bit」这个硬约束准备的最终形态。
#
#  为什么是这个组合（不是"雷达 + ST + LK + NMS"）：
#    ① LK 留在 CPU 又准又快（cv2 LK 6.1 ms / 精度 0.0436 ≈ 基线），
#       而 PL LK 10.5 ms 且流偏差 ~0.11 px ⇒ 端到端 0.0528（更差）——**LK 不该进 PL**；
#    ② 硬塞 LK4+NMS 到雷达 bit 要 113.7k LUT（97.1%），布线风险极高；
#       去掉 LK 后预计 ~91%，还留出余量；
#    ③ FAST 也不用（原视觉节点已改 ST 路线）。
#  ⇒ 视觉侧走 `--frontend stcv2`：ST(PL) + NMS(PL) + LK(cpu)，匹配 ~16.8 ms。
#
#  数据通路照 merge_st_150.tcl：雷达 8 路 m_axi 汇到 smartconnect→HPC0，
#  视觉各占一个 HP 口，互不干扰。
# ============================================================
set RADAR_IP /home/srtp2025_nie/fe_p0/final1/hls_build.prj/solution1/impl/ip
set SLK_IP   /home/srtp2025_nie/slk_hls/slk_proj/slk_sol/impl/ip
set S2M_IP   /home/srtp2025_nie/s2m_vmap_hls/proj_vmap_impl/sol1/impl/ip
set ST_IP    /home/srtp2025_nie/st_ip_2024
set NMS_IP   /home/srtp2025_nie/lk_lvisam/ipnms/proj/sol/impl/ip
set PROJ     /home/srtp2025_nie/fusion_bd_final3

create_project fusion_bd $PROJ -part xck26-sfvc784-2LV-c -force
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths "$RADAR_IP $ST_IP $NMS_IP $SLK_IP $S2M_IP" [current_project]
update_ip_catalog -rebuild

create_bd_design design_1
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ultra_ps_e_0
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells zynq_ultra_ps_e_0]

set_property -dict [list \
  CONFIG.PSU__USE__M_AXI_GP0 {1} \
  CONFIG.PSU__USE__S_AXI_GP0 {1} \
  CONFIG.PSU__USE__S_AXI_GP1 {1} \
  CONFIG.PSU__USE__S_AXI_GP2 {1} ] [get_bd_cells zynq_ultra_ps_e_0]
set_property -dict [list CONFIG.PSU__FPGA_PL0_ENABLE {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {150}] [get_bd_cells zynq_ultra_ps_e_0]
puts "===PL0_FREQ=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ [get_bd_cells zynq_ultra_ps_e_0]]==="

# 实例名与板端驱动一致（PYNQ ip_dict 按名索引，不能改）
create_bd_cell -type ip -vlnv xilinx.com:hls:fpga_feature_extract_hw:1.0 fpga_feature_extract_0
create_bd_cell -type ip -vlnv user:hls:shitomasi_accel:1.0 shitomasi_accel_0
create_bd_cell -type ip -vlnv user:hls:nms_min_dist:1.0 nms_accel_0
create_bd_cell -type ip -vlnv user:hls:sparse_lk:1.0 slk_accel_0
create_bd_cell -type ip -vlnv xilinx.com:hls:s2m_vmap:1.0 s2m_accel_0

puts "===INTF==="
foreach p [lsort [get_bd_intf_pins /fpga_feature_extract_0/*]] { puts "  FE  $p" }
foreach p [lsort [get_bd_intf_pins /shitomasi_accel_0/*]]   { puts "  ST  $p" }
foreach p [lsort [get_bd_intf_pins /nms_accel_0/*]]         { puts "  NMS $p" }
foreach p [lsort [get_bd_intf_pins /slk_accel_0/*]]         { puts "  SLK $p" }
foreach p [lsort [get_bd_intf_pins /s2m_accel_0/*]]         { puts "  S2M $p" }

# ---------------- 控制通路：三个 IP 的 s_axi 全挂 M_AXI_HPM0_FPD ----------------
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins fpga_feature_extract_0/s_axi_control_bus]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins shitomasi_accel_0/s_axi_control]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins nms_accel_0/s_axi_control]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins slk_accel_0/s_axi_control]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD"} [get_bd_intf_pins s2m_accel_0/s_axi_control]

# ---------------- 数据通路 ①④⑤：雷达 + 稀疏LK + s2m_vmap 全部 m_axi -> SmartConnect -> HPC0
#  PS 的 S_AXI_HP1/HP2 在当前 PS 配置下未使能（"No interface pins matched"），
#  因此三个 IP 的 master 共用一个 smartconnect，仅占 HPC0 一个口。
set gmem {}
foreach p [lsort [get_bd_intf_pins /fpga_feature_extract_0/*]] {
  set nm [get_property NAME $p]
  if {[string match "s_axi*" $nm]} { continue }
  lappend gmem $p
}
set RADAR_NSI [llength $gmem]
puts "===RADAR_MASTER_COUNT=$RADAR_NSI==="

foreach ip {slk_accel_0 s2m_accel_0} {
  set n 0
  foreach p [lsort [get_bd_intf_pins /$ip/*]] {
    set nm [get_property NAME $p]
    if {[string match "s_axi*" $nm]} { continue }
    lappend gmem $p
    incr n
  }
  puts "===${ip}_MASTER_COUNT=$n==="
}
set NSI [llength $gmem]
puts "===TOTAL_MASTER_COUNT=$NSI==="
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0
set_property -dict [list CONFIG.NUM_SI $NSI CONFIG.NUM_MI {1}] [get_bd_cells smartconnect_0]
for {set i 0} {$i < $NSI} {incr i} {
  connect_bd_intf_net [get_bd_intf_pins [lindex $gmem $i]] [get_bd_intf_pins smartconnect_0/S[format %02d $i]_AXI]
}
connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HPC0_FPD]

create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_sc
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins rst_sc/slowest_sync_clk]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins rst_sc/ext_reset_in]
connect_bd_net [get_bd_pins rst_sc/peripheral_aresetn] [get_bd_pins smartconnect_0/aresetn]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins smartconnect_0/aclk]

# ---------------- 数据通路 ②③：视觉各占一个口 ----------------
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/shitomasi_accel_0/m_axi_gmem0"} [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HPC1_FPD]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Clk "Auto" Master "/nms_accel_0/m_axi_gmem1"}       [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HP0_FPD]

foreach p [get_bd_pins -quiet zynq_ultra_ps_e_0/*] {
  if {[string match "*aclk*" $p] && [get_bd_nets -quiet -of_objects $p] eq ""} {
    puts "ACLK_FIX $p"
    catch { connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] $p }
  }
}

assign_bd_address
puts "===ADDR_MAP==="
foreach s [get_bd_addr_segs -of_objects [get_bd_cells zynq_ultra_ps_e_0]] {
  catch { puts "  $s -> [get_property offset $s] \[[get_property range $s]\]" }
}
validate_bd_design
save_bd_design
puts "===BD_OK==="

set wr [make_wrapper -files [get_files design_1.bd] -top]
add_files -norecurse $wr
set_property top design_1_wrapper [current_fileset]
update_compile_order -fileset sources_1
puts "===WRAPPER_OK top=[get_property top [current_fileset]]==="

if {[catch {set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]} msg]} {
  puts "===STRATEGY_TRY1_FAIL $msg==="
  catch {set_property strategy Flow_PerfOptimized_high [get_runs impl_1]}
}
puts "===IMPL_STRATEGY=[get_property strategy [get_runs impl_1]]==="

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "===IMPL=[get_property STATUS [get_runs impl_1]]==="
puts "===WNS=[get_property STATS.WNS [get_runs impl_1]]==="
puts "===WHS=[get_property STATS.WHS [get_runs impl_1]]==="

open_run impl_1
report_utilization -file /home/srtp2025_nie/final3_util_total.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file /home/srtp2025_nie/final3_util_hier.rpt
report_timing_summary -delay_type min_max -max_paths 10 -file /home/srtp2025_nie/final3_timing.rpt
file copy -force $PROJ/fusion_bd.runs/impl_1/design_1_wrapper.bit  /home/srtp2025_nie/kv260_final3.bit
file copy -force $PROJ/fusion_bd.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh /home/srtp2025_nie/kv260_final3.hwh
puts "===ALL_DONE==="
exit
