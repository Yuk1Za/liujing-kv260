# Run only after the original HLS run ends and its log/report/testbench have
# been archived. Replace tb.cpp with the separately tracked stress testbench.
open_project liujing_stream
open_solution solution
csim_design
cosim_design -rtl verilog -random_stall
exit
