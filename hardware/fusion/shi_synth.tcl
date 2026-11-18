set_param general.maxThreads 1
set legacy [file join [file dirname [file normalize [info script]]] ip shi_2024_runtime]
read_vhdl -library xbip_utils_v3_0_14 [file join $legacy xbip_utils_v3_0_14/xbip_utils_v3_0_vh_rfs.vhd]
read_vhdl -library axi_utils_v2_0_10 [file join $legacy axi_utils_v2_0_10/axi_utils_v2_0_vh_rfs.vhd]
read_vhdl -library xbip_pipe_v3_0_10 [file join $legacy xbip_pipe_v3_0_10/xbip_pipe_v3_0_vh_rfs.vhd]
read_vhdl -library xbip_dsp48_wrapper_v3_0_6 [file join $legacy xbip_dsp48_wrapper_v3_0_6/xbip_dsp48_wrapper_v3_0_vh_rfs.vhd]
read_vhdl -library mult_gen_v12_0_22 [file join $legacy mult_gen_v12_0_22/mult_gen_v12_0_vh_rfs.vhd]
read_vhdl -library floating_point_v7_1_19 [file join $legacy floating_point_v7_1_19/floating_point_v7_1_rfs.vhd]
read_verilog -library floating_point_v7_1_19 [file join $legacy floating_point_v7_1_19/floating_point_v7_1_rfs.v]
