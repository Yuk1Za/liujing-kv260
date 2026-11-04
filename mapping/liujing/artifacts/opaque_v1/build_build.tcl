# Re-place the CURRENT freshly integrated platform inside a bounded CLB region.
# This is not an old optimized platform checkpoint; no logic/AXI edits or clock relaxations.
set_param general.maxThreads 2
set source [file normalize [lindex $argv 0]]
open_checkpoint $source
file mkdir artifacts
report_utilization -hierarchical -hierarchical_depth 3 -file hierarchy_before.rpt
set all [get_sites -filter {SITE_TYPE =~ SLICE*}]
set total [llength $all]
if {$total != 14640} {error "Unexpected K26 slice inventory: $total"}
set byx [dict create]
set ymax 0
foreach site $all {
 if {![regexp {SLICE_X([0-9]+)Y([0-9]+)} $site -> x y]} {error "Unknown slice name $site"}
 dict incr byx $x
 if {$y>$ymax} {set ymax $y}
}
set count 0
set xmax -1
foreach x [lsort -integer [dict keys $byx]] {
 set next [expr {$count+[dict get $byx $x]}]
 if {$next>int(.895*$total)} {break}
 set count $next
 set xmax $x
}
if {$xmax<0} {error "No bounded placement region"}
create_pblock bounded_clb
resize_pblock [get_pblocks bounded_clb] -add "SLICE_X0Y0:SLICE_X${xmax}Y${ymax}"
# Keep every BRAM/DSP/URAM location available; only CLB placement is bounded.
foreach prefix {RAMB18 RAMB36 DSP48E2 URAM288} {
 set sites [get_sites -quiet ${prefix}_X*Y*]
 if {![llength $sites]} {continue}
 set maxx 0;set maxy 0
 foreach site $sites {
  regexp {_X([0-9]+)Y([0-9]+)} $site -> x y
  if {$x>$maxx} {set maxx $x}
  if {$y>$maxy} {set maxy $y}
 }
 resize_pblock [get_pblocks bounded_clb] -add "${prefix}_X0Y0:${prefix}_X${maxx}Y${maxy}"
}
add_cells_to_pblock [get_pblocks bounded_clb] [get_cells -hier -filter {IS_PRIMITIVE && REF_NAME != PS8 && REF_NAME !~ BUFG*}]
set_property IS_SOFT false [get_pblocks bounded_clb]
set f [open placement_budget.txt w]
puts $f "source=$source\nphysical_clb_total=$total\nallowed_clb_sites=$count\nmaximum_x=$xmax\nmaximum_y=$ymax\nclock_period_ns=10; unchanged"
close $f
write_checkpoint artifacts/bounded_input.dcp
place_design -directive Explore
phys_opt_design
route_design
report_utilization -file artifacts/utilization.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file artifacts/hierarchy.rpt
report_timing_summary -delay_type min_max -report_unconstrained -file artifacts/timing.rpt
report_route_status -file artifacts/route.rpt
report_drc -file artifacts/drc.rpt
check_timing -verbose -file artifacts/check_timing.rpt
write_checkpoint artifacts/routed.dcp
set setup [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
if {$setup eq "" || $hold eq "" || $setup<0 || $hold<0} {error "Timing failed"}
if {[llength [get_drc_violations -filter {SEVERITY == Error}]]>0} {error "DRC errors"}
write_bitstream artifacts/liujing.bit
exit
