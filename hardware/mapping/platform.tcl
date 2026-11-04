# Integrated C16 + LiuJing raster. Project-mode build, 100 MHz, K26.
set c16_repo E:/tmp/losam_c16_clean_20261022/c16_hls/solution/impl/ip
set render_repo E:/tmp/liujing_raster_v2_20261024/liujing_raster/solution/impl/ip
if {[info exists ::env(LIUJING_C16_IP)]} {set c16_repo $::env(LIUJING_C16_IP)}
if {[info exists ::env(LIUJING_RASTER_IP)]} {set render_repo $::env(LIUJING_RASTER_IP)}
foreach p [list $c16_repo $render_repo] {if {![file exists $p/component.xml]} {error "Missing packaged IP: $p"}}
create_project liujing ./platform -part xck26-sfvc784-2LV-c
if {[info exists ::env(LIUJING_IP_CACHE)]} {
 # Vivado checks configuration, RTL and tool-version cache IDs. Only unchanged
 # IPs (not the newly edited renderer) can reuse OOC synthesis.
 config_ip_cache -use_cache_location $::env(LIUJING_IP_CACHE)
}
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths [list $c16_repo $render_repo] [current_project]
update_ip_catalog
create_bd_design system
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells ps]
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} CONFIG.PSU__USE__S_AXI_GP2 {1} CONFIG.PSU__USE__S_AXI_GP3 {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] [get_bd_cells ps]
create_bd_cell -type ip -vlnv xilinx.com:hls:fpga_feature_extract_hw:1.0 fpga_feature_extract_0
create_bd_cell -type ip -vlnv xilinx.com:hls:lj_raster:1.0 lj_raster_0
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 control_bus
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] [get_bd_cells control_bus]
foreach {name count} {lidar_mem 8 raster_mem 3} {
 create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $name
 set_property CONFIG.NUM_SI $count [get_bd_cells $name]
}
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset
set_property CONFIG.C_EXT_RESET_HIGH 0 [get_bd_cells reset]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins control_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins control_bus/M00_AXI] [get_bd_intf_pins fpga_feature_extract_0/s_axi_control_bus]
connect_bd_intf_net [get_bd_intf_pins control_bus/M01_AXI] [get_bd_intf_pins lj_raster_0/s_axi_control]
for {set i 0} {$i<8} {incr i} {connect_bd_intf_net [get_bd_intf_pins fpga_feature_extract_0/m_axi_gmem${i}] [get_bd_intf_pins lidar_mem/S0${i}_AXI]}
set i 0
foreach port {surf index output_mem} {connect_bd_intf_net [get_bd_intf_pins lj_raster_0/m_axi_${port}] [get_bd_intf_pins raster_mem/S0${i}_AXI];incr i}
connect_bd_intf_net [get_bd_intf_pins lidar_mem/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_intf_net [get_bd_intf_pins raster_mem/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP1_FPD]
foreach pin {ps/maxihpm0_fpd_aclk ps/saxihp0_fpd_aclk ps/saxihp1_fpd_aclk control_bus/aclk lidar_mem/aclk raster_mem/aclk reset/slowest_sync_clk fpga_feature_extract_0/ap_clk lj_raster_0/ap_clk} {connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins $pin]}
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset/ext_reset_in]
foreach pin {control_bus/aresetn lidar_mem/aresetn raster_mem/aresetn fpga_feature_extract_0/ap_rst_n lj_raster_0/ap_rst_n} {connect_bd_net [get_bd_pins reset/peripheral_aresetn] [get_bd_pins $pin]}
assign_bd_address
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top
add_files -norecurse ./platform/liujing.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
if {[info exists ::env(LIUJING_IMPL_GATE)]} {
 puts "Waiting for local implementation scheduling gate: $::env(LIUJING_IMPL_GATE)"
 while {![file exists $::env(LIUJING_IMPL_GATE)]} {after 10000}
}
launch_runs impl_1 -to_step route_design -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Route failed"}
open_run impl_1
file mkdir artifacts
report_utilization -file artifacts/utilization.rpt
report_timing_summary -delay_type min_max -report_unconstrained -file artifacts/timing.rpt
report_route_status -file artifacts/route.rpt
report_drc -file artifacts/drc.rpt
check_timing -verbose -file artifacts/check_timing.rpt
set setup [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup eq "" || $hold eq "" || $setup<0 || $hold<0} {error "Timing failed"}
if {[llength [get_drc_violations -filter {SEVERITY == Error}]]>0} {error "DRC errors"}
# The routed design is already open. Avoid a second Vivado process loading the
# entire design alongside this process on a memory-limited development laptop.
write_bitstream ./artifacts/liujing.bit
file copy ./platform/liujing.gen/sources_1/bd/system/hw_handoff/system.hwh ./artifacts/liujing.hwh
exit
