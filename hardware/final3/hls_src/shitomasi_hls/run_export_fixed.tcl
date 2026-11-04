# Export the FIXED Shi-Tomasi kernel as an IP catalog entry.
# Reuses the already-synthesised solution (no re-csynth).
open_project /home/srtp2025_nie/shitomasi_hls/csynth_fixed_proj
set_top shitomasi_accel
open_solution fixed_sol
export_design -format ip_catalog -description "Shi-Tomasi corner detector, response-output (XF_RESP_SHIFT=2)" -vendor "user" -library "hls" -version "1.0"
puts "===EXPORT_DONE==="
exit
