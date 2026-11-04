# Inspect an already optimized design: ABI metadata cannot prove retained logic.
set_param general.maxThreads 1
open_checkpoint [lindex $argv 0]
set f [open [lindex $argv 1] w]
foreach pin [lsort [get_pins -of_objects [get_cells system_i/lj_raster_0] -filter {DIRECTION == OUT}]] {
  if {![string match *m_axi* $pin]} {continue}
  set nets [get_nets -quiet -of_objects $pin]
  set sinks [get_pins -quiet -leaf -of_objects $nets -filter {DIRECTION == IN}]
  puts $f "$pin\t$nets\t[get_property -quiet TYPE $nets]\t[llength $sinks]\t$sinks"
}
close $f
exit
