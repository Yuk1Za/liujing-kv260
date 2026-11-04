set SRC /home/srtp2025_nie/shitomasi_hls
set VL  /home/srtp2025_nie/Vitis_Libraries/vision
open_project -reset shitomasi_proj
set_top shitomasi_accel
add_files $SRC/xf_shitomasi_accel.cpp -cflags "-I$SRC -I$SRC/features -I$VL/L1/include -I$VL/L1/include/features -D__SDSVHLS__ -std=c++0x"
open_solution -reset shitomasi_sol
set_part xck26-sfvc784-2LV-c
create_clock -period 7.0 -name default
csynth_design
exit
