open_project -reset proj
set_top nms_min_dist
add_files /home/srtp2025_nie/lk_lvisam/nms_min_dist.cpp -cflags "-D__SDSVHLS__ -std=c++0x"
open_solution -reset sol
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csynth_design
export_design -format ip_catalog -description "MIN_DIST NMS top-N (greedy, cv2-equivalent)" -vendor user -library hls -version 1.0 -output nms_min_dist.zip
exit
