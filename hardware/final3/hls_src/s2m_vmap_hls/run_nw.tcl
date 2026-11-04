open_project -reset proj_vmap_nw
set_top s2m_vmap
add_files s2m_vmap.cpp -cflags "-DNOWEIGHT=1"
add_files -tb tb_s2m_vmap.cpp
open_solution -reset sol1
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csynth_design
exit
