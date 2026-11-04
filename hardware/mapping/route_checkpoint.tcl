# Recovery from a saved, fully placed/physopt checkpoint after host OOM.
# A single process owns the complete design; no parallel synth/route jobs.
set checkpoint [lindex $argv 0]
set hwh [lindex $argv 1]
set_param general.maxThreads 2
open_checkpoint $checkpoint
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
file copy $hwh artifacts/liujing.hwh
exit
