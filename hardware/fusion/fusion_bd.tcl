# fusion_probe: opaque_v1 BD 底 + 我方压缩雷达核 + ST/NMS + lj_raster
# 源：~/liujing/hardware/platform.tcl（复制改写；原文件未改）
set root /home/srtp2025_nie/fusion_probe
set c16_repo $root/ip/radar
set render_repo $root/ip/lj_raster
set st_repo $root/ip/shi_tomasi
set nms_repo $root/ip/nms
foreach p [list $c16_repo $render_repo $st_repo $nms_repo] {if {![file exists $p/component.xml]} {error "Missing packaged IP: $p"}}
create_project fusion $root/work/bd -part xck26-sfvc784-2LV-c -force
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths [list $c16_repo $render_repo $st_repo $nms_repo] [current_project]
update_ip_catalog
create_bd_design system
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells ps]
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} CONFIG.PSU__USE__S_AXI_GP2 {1} CONFIG.PSU__USE__S_AXI_GP3 {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] [get_bd_cells ps]
create_bd_cell -type ip -vlnv xilinx.com:hls:fpga_feature_extract_hw:1.0 fpga_feature_extract_0
create_bd_cell -type ip -vlnv xilinx.com:hls:lj_raster:1.0 lj_raster_0
create_bd_cell -type ip -vlnv user:hls:shitomasi_accel:1.0 shitomasi_accel_0
create_bd_cell -type ip -vlnv user:hls:nms_min_dist:1.0 nms_accel_0
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 control_bus
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {4}] [get_bd_cells control_bus]
foreach {name count} {lidar_mem 10 raster_mem 3} {
 create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $name
 set_property CONFIG.NUM_SI $count [get_bd_cells $name]
}
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset
set_property CONFIG.C_EXT_RESET_HIGH 0 [get_bd_cells reset]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins control_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins control_bus/M00_AXI] [get_bd_intf_pins fpga_feature_extract_0/s_axi_control_bus]
connect_bd_intf_net [get_bd_intf_pins control_bus/M01_AXI] [get_bd_intf_pins lj_raster_0/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins control_bus/M02_AXI] [get_bd_intf_pins shitomasi_accel_0/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins control_bus/M03_AXI] [get_bd_intf_pins nms_accel_0/s_axi_control]
for {set i 0} {$i<8} {incr i} {connect_bd_intf_net [get_bd_intf_pins fpga_feature_extract_0/m_axi_gmem$i] [get_bd_intf_pins lidar_mem/S0${i}_AXI]}
connect_bd_intf_net [get_bd_intf_pins shitomasi_accel_0/m_axi_gmem0] [get_bd_intf_pins lidar_mem/S08_AXI]
connect_bd_intf_net [get_bd_intf_pins nms_accel_0/m_axi_gmem1] [get_bd_intf_pins lidar_mem/S09_AXI]
set i 0
foreach port {surf index output_mem} {connect_bd_intf_net [get_bd_intf_pins lj_raster_0/m_axi_${port}] [get_bd_intf_pins raster_mem/S0${i}_AXI];incr i}
connect_bd_intf_net [get_bd_intf_pins lidar_mem/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_intf_net [get_bd_intf_pins raster_mem/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP1_FPD]
foreach pin {ps/maxihpm0_fpd_aclk ps/saxihp0_fpd_aclk ps/saxihp1_fpd_aclk control_bus/aclk lidar_mem/aclk raster_mem/aclk reset/slowest_sync_clk fpga_feature_extract_0/ap_clk lj_raster_0/ap_clk shitomasi_accel_0/ap_clk nms_accel_0/ap_clk} {connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins $pin]}
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset/ext_reset_in]
foreach pin {control_bus/aresetn lidar_mem/aresetn raster_mem/aresetn fpga_feature_extract_0/ap_rst_n lj_raster_0/ap_rst_n shitomasi_accel_0/ap_rst_n nms_accel_0/ap_rst_n} {connect_bd_net [get_bd_pins reset/peripheral_aresetn] [get_bd_pins $pin]}
assign_bd_address
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top
add_files -norecurse $root/work/bd/fusion.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step route_design -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Route failed"}
open_run impl_1
file mkdir $root/work/artifacts
report_utilization -file $root/work/artifacts/utilization.rpt
report_utilization -hierarchical -file $root/work/artifacts/util_hier.rpt
report_timing_summary -delay_type min_max -report_unconstrained -file $root/work/artifacts/timing.rpt
report_route_status -file $root/work/artifacts/route.rpt
puts "=== SETUP_SLACK: [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]] ==="
exit
