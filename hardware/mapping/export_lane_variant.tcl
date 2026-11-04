set lanes $env(LIUJING_LANES)
if {$lanes ni {1 2 4}} {error "lane count must be 1, 2 or 4"}
open_project lanes_$lanes
open_solution solution
export_design -format ip_catalog
exit
