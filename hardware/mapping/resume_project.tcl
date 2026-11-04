# Resume an existing experiment after a host-memory failure. Caller first
# archives run logs and runs this without another synthesis/implementation job.
set_param general.maxThreads 2
open_project [lindex $argv 0]
foreach r [get_runs] {
 if {[get_property IS_SYNTHESIS $r] && [get_property PROGRESS $r] ne "100%"} {reset_run $r}
}
launch_runs synth_1 -jobs 1
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step route_design -jobs 1
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
write_bitstream artifacts/liujing.bit
file copy ./platform/liujing.gen/sources_1/bd/system/hw_handoff/system.hwh artifacts/liujing.hwh
exit
