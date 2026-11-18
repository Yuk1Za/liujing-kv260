cd [file dirname [file normalize [info script]]]
open_project -read_only platform/liujing.xpr
open_bd_design [get_files */system.bd]
puts "FOUR_CORE_BD [current_bd_design] [get_bd_cells]"
