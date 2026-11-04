# Synthesize only a new renderer. The accepted static system is reused as a
# netlist in replace_renderer.tcl, avoiding repeated C16 and interconnect jobs.
set synth_threads 1
if {[llength $argv]>1} {set synth_threads [lindex $argv 1]}
if {$synth_threads ni {1 2}} {error "Use one or two synthesis threads on this host"}
set_param general.maxThreads $synth_threads
create_project renderer ./renderer -part xck26-sfvc784-2LV-c
set_property ip_repo_paths [list [lindex $argv 0]] [current_project]
update_ip_catalog
create_ip -vlnv xilinx.com:hls:lj_raster:1.0 -module_name system_lj_raster_0_0
generate_target all [get_ips system_lj_raster_0_0]
synth_ip [get_ips system_lj_raster_0_0]
close_project
open_checkpoint ./renderer/renderer.gen/sources_1/ip/system_lj_raster_0_0/system_lj_raster_0_0.dcp
report_utilization -hierarchical -file renderer_utilization.rpt
exit
