set root /home/srtp2025_nie/fusion_probe
open_checkpoint $root/work/bd/fusion.runs/impl_1/system_wrapper_routed.dcp
write_bitstream -force $root/work/artifacts/fusion.bit
file copy -force $root/work/bd/fusion.gen/sources_1/bd/system/hw_handoff/system.hwh $root/work/artifacts/fusion.hwh
puts "WRITE_BIT_DONE"
exit
