cd [file dirname [file normalize [info script]]]
set c16_repo [file normalize ip/c16]
set association_repo [file normalize ip/fusion]
foreach p [list $c16_repo $association_repo] {if {![file exists $p/component.xml]} {error "Missing packaged IP: $p"}}
set_param general.maxThreads 1
create_project liujing ./platform -part xck26-sfvc784-2LV-c
if {[file isdirectory cache]} {
 # Vendor-keyed OOC cache only, never an optimized system boundary checkpoint.
 set_property IP_OUTPUT_REPO [file normalize cache] [current_project]
 set_property IP_CACHE_PERMISSIONS {read write} [current_project]
}
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths [list $c16_repo $association_repo [file normalize ip/shi] [file normalize ip/nms]] [current_project]
update_ip_catalog
create_bd_design system
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells ps]
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} CONFIG.PSU__USE__S_AXI_GP2 {1} CONFIG.PSU__USE__S_AXI_GP3 {1} CONFIG.PSU__USE__S_AXI_GP4 {1} CONFIG.PSU__USE__S_AXI_GP5 {1} CONFIG.PSU__SAXIGP5__DATA_WIDTH {32} CONFIG.PSU__USE__S_AXI_GP1 {1} CONFIG.PSU__USE__S_AXI_GP0 {1} CONFIG.PSU__SAXIGP0__DATA_WIDTH {32} CONFIG.PSU__SAXIGP1__DATA_WIDTH {32} CONFIG.PSU__SAXIGP4__DATA_WIDTH {32} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] [get_bd_cells ps]
create_bd_cell -type ip -vlnv xilinx.com:hls:fpga_feature_extract_hw:1.0 fpga_feature_extract_0
create_bd_cell -type ip -vlnv xilinx.com:hls:fusion_frame:1.0 fusion_frame_0
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 control_bus
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {4}] [get_bd_cells control_bus]
# Route C16 ports by their native data widths to avoid eight width converters.
foreach {name count width} {lidar_mem 2 128 lidar_writes 6 32} {
 create_bd_cell -type ip -vlnv xilinx.com:ip:axi_crossbar:2.1 $name
 set_property -dict [list CONFIG.NUM_MI 1 CONFIG.NUM_SI $count CONFIG.DATA_WIDTH $width CONFIG.M00_WRITE_ISSUING 16] [get_bd_cells $name]
}
set_property CONFIG.M00_READ_ISSUING 16 [get_bd_cells lidar_mem]
connect_bd_intf_net [get_bd_intf_pins lidar_writes/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP3_FPD]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset
set_property CONFIG.C_EXT_RESET_HIGH 0 [get_bd_cells reset]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins control_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins control_bus/M00_AXI] [get_bd_intf_pins fpga_feature_extract_0/s_axi_control_bus]
connect_bd_intf_net [get_bd_intf_pins control_bus/M01_AXI] [get_bd_intf_pins fusion_frame_0/s_axi_control]
create_bd_cell -type ip -vlnv user:hls:shitomasi_accel:1.0 shitomasi_accel_0
create_bd_cell -type ip -vlnv user:hls:nms_min_dist:1.0 nms_accel_0
foreach {src dst} {
 control_bus/M02_AXI shitomasi_accel_0/s_axi_control
 control_bus/M03_AXI nms_accel_0/s_axi_control
 shitomasi_accel_0/m_axi_gmem0 ps/S_AXI_HPC1_FPD
 nms_accel_0/m_axi_gmem1 ps/S_AXI_HPC0_FPD
} {connect_bd_intf_net [get_bd_intf_pins $src] [get_bd_intf_pins $dst]}
for {set i 0} {$i<8} {incr i} {
 set port lidar_mem/S0${i}_AXI
 if {$i>=2} {set port lidar_writes/S0[expr {$i-2}]_AXI}
 connect_bd_intf_net [get_bd_intf_pins fpga_feature_extract_0/m_axi_gmem${i}] [get_bd_intf_pins $port]
}
connect_bd_intf_net [get_bd_intf_pins fusion_frame_0/m_axi_mapbus] [get_bd_intf_pins ps/S_AXI_HP1_FPD]
connect_bd_intf_net [get_bd_intf_pins fusion_frame_0/m_axi_iobus] [get_bd_intf_pins ps/S_AXI_HP2_FPD]
connect_bd_intf_net [get_bd_intf_pins lidar_mem/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
foreach pin {ps/maxihpm0_fpd_aclk ps/saxihp0_fpd_aclk ps/saxihp1_fpd_aclk ps/saxihp2_fpd_aclk ps/saxihp3_fpd_aclk ps/saxihpc1_fpd_aclk lidar_writes/aclk ps/saxihpc0_fpd_aclk shitomasi_accel_0/ap_clk nms_accel_0/ap_clk control_bus/aclk lidar_mem/aclk reset/slowest_sync_clk fpga_feature_extract_0/ap_clk fusion_frame_0/ap_clk} {connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins $pin]}
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset/ext_reset_in]
if {[llength [get_bd_pins -quiet fusion_frame_0/fabric_cycle]]} {
 create_bd_cell -type ip -vlnv xilinx.com:ip:c_counter_binary:12.0 fusion_cycles
 set_property -dict [list CONFIG.Output_Width {32} CONFIG.CE {false}] [get_bd_cells fusion_cycles]
 connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins fusion_cycles/CLK]
 connect_bd_net [get_bd_pins fusion_cycles/Q] [get_bd_pins fusion_frame_0/fabric_cycle]
}
foreach pin {lidar_writes/aresetn shitomasi_accel_0/ap_rst_n nms_accel_0/ap_rst_n control_bus/aresetn lidar_mem/aresetn fpga_feature_extract_0/ap_rst_n fusion_frame_0/ap_rst_n} {connect_bd_net [get_bd_pins reset/peripheral_aresetn] [get_bd_pins $pin]}
foreach {ip addr} {fpga_feature_extract_0 0xA0000000 fusion_frame_0 0xA0010000 shitomasi_accel_0 0xA0020000 nms_accel_0 0xA0030000} {
 assign_bd_address -offset $addr -range 64K -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs $ip/*/Reg]
}
assign_bd_address
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top
add_files -norecurse ./platform/liujing.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]
if {[info exists ::env(LIUJING_STAGE_PROJECT)]} {close_project;exit}
create_ip_run [get_files system.bd]
foreach run [get_runs -filter {IS_SYNTHESIS == 1}] {
 set_property STEPS.SYNTH_DESIGN.TCL.PRE [file normalize one_thread.tcl] $run
}
set_property STEPS.SYNTH_DESIGN.TCL.PRE [file normalize shi_synth.tcl] [get_runs system_shitomasi_accel_0_0_synth_1]
set_property STEPS.SYNTH_DESIGN.TCL.PRE [file normalize shi_synth.tcl] [get_runs system_fpga_feature_extract_0_0_synth_1]
foreach ip {fpga_feature_extract_0 fusion_frame_0 shitomasi_accel_0 nms_accel_0} {
 set run [get_runs system_${ip}_0_synth_1]
 set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE AreaOptimized_high $run
}
set_property STEPS.OPT_DESIGN.TCL.PRE [file normalize one_thread.tcl] [get_runs impl_1]
launch_runs synth_1 -jobs 1
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE ExploreSequentialArea [get_runs impl_1]
launch_runs impl_1 -to_step route_design -jobs 1
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Route failed"}
open_run impl_1
file mkdir artifacts
report_utilization -hierarchical -file artifacts/util_hier.rpt
report_utilization -file artifacts/utilization.rpt
report_timing_summary -delay_type min_max -report_unconstrained -file artifacts/timing.rpt
report_route_status -file artifacts/route.rpt
report_drc -file artifacts/drc.rpt
report_methodology -file artifacts/methodology.rpt
report_clock_interaction -file artifacts/clock_interaction.rpt
report_cdc -file artifacts/cdc.rpt
report_clocks -file artifacts/clocks.rpt
check_timing -verbose -file artifacts/check_timing.rpt
set setup [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup eq "" || $hold eq "" || $setup<0 || $hold<0} {error "Timing failed"}
if {[llength [get_drc_violations -filter {SEVERITY == Error}]]>0} {error "DRC errors"}
# The routed design is already open. Avoid a second Vivado process loading the
# entire design alongside this process on a memory-limited development laptop.
write_bitstream ./artifacts/liujing.bit
write_checkpoint ./artifacts/routed.dcp
file copy ./platform/liujing.gen/sources_1/bd/system/hw_handoff/system.hwh ./artifacts/liujing.hwh
exit

