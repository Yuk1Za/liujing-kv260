# Non-project full implementation. This is NOT partial reconfiguration and
# still produces one full C16+renderer bitstream. Only the renderer cell is
# replaced; PS, C16 and bus netlists/constraints come from the saved opt DCP.
set base [lindex $argv 0]
set renderer [lindex $argv 1]
set hwh [lindex $argv 2]
set opt_directive Default
if {[llength $argv]>3} {set opt_directive [lindex $argv 3]}
if {$opt_directive ni {Default ExploreArea ExploreSequentialArea ExploreWithRemap}} {error "Unsupported optimization directive"}
set_param general.maxThreads 2
open_checkpoint $base
set cell [get_cells -quiet system_i/lj_raster_0]
if {[llength $cell]!=1} {error "Expected exactly one renderer cell"}
update_design -cell $cell -black_box
read_checkpoint -cell system_i/lj_raster_0 $renderer
if {[llength [get_cells -hier -filter {IS_BLACKBOX}]]!=0} {error "Unresolved black box after renderer replacement"}
opt_design -directive $opt_directive
file mkdir checkpoints
write_checkpoint checkpoints/opt.dcp
place_design
write_checkpoint checkpoints/placed.dcp
phys_opt_design
write_checkpoint checkpoints/physopt.dcp
route_design
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
write_checkpoint artifacts/routed.dcp
write_bitstream artifacts/liujing.bit
# Reuse HWH only for a pin/register-compatible replacement. read_checkpoint
# checks netlist ports; the driver checks control register offsets before use.
file copy $hwh artifacts/liujing.hwh
exit
