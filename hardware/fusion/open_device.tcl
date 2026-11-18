cd [file dirname [file normalize [info script]]]
open_checkpoint artifacts/routed.dcp
foreach {ip color} {fpga_feature_extract_0 yellow shitomasi_accel_0 green nms_accel_0 cyan fusion_frame_0 magenta} {
    set cells [get_cells -hierarchical -filter "IS_PRIMITIVE && NAME =~ */${ip}/*"]
    if {![llength $cells]} {error "No physical cells matched: $ip"}
    highlight_objects -color $color $cells
    set highlighted [llength [get_highlighted_objects -color $color]]
    if {!$highlighted} {error "Vivado did not highlight $ip in $color"}
    puts "FOUR_CORE_COLOR $ip $color $highlighted"
}

