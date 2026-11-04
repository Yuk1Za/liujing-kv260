# Existing lane C model project from check_lane_variants.tcl; no export/OOC
# Vivado here, so only one memory-heavy implementation runs on the host.
set lanes $env(LIUJING_LANES)
if {$lanes ni {1 2 4}} {error "lane count must be 1, 2 or 4"}
open_project lanes_$lanes
open_solution solution
csynth_design
exit
