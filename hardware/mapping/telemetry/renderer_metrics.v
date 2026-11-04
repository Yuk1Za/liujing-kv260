`timescale 1ns/1ps
// Passive instrumentation. No READY/VALID or datapath output is modified.
// Counts stop on ap_done and remain readable until the next rising ap_start.
// The software contract disables auto-restart; elapsed counts clocks after
// rising-start detection through the done sampling edge. 32 bits cover 42.9 s
// at 100 MHz, above the driver's 10 s timeout.
module renderer_metrics(
 input wire clk, reset, start, done,
 input wire surf_arvalid,surf_arready,
 input wire [7:0] surf_arlen,
 input wire surf_rvalid,surf_rready,
 input wire index_arvalid,index_arready,
 input wire [7:0] index_arlen,
 input wire index_rvalid,index_rready,
 input wire out_awvalid,out_awready,out_wvalid,out_wready,
 output wire [383:0] metrics
);
 reg previous_start,active;
 reg [31:0] cycles,surf_beats,surf_wait,surf_block,surf_address_wait;
 reg [31:0] index_beats,index_wait,out_beats,out_block,out_address_wait,index_address_wait;
 reg [15:0] surf_pending,index_pending;
 wire surf_ar=surf_arvalid&&surf_arready, surf_r=surf_rvalid&&surf_rready;
 wire index_ar=index_arvalid&&index_arready, index_r=index_rvalid&&index_rready;
 assign metrics={index_address_wait,out_address_wait,out_block,out_beats,
                 index_wait,index_beats,surf_address_wait,surf_block,
                 surf_wait,surf_beats,cycles,32'h4c4a0101};
 always @(posedge clk) begin
  if(reset) begin
   previous_start<=0;active<=0;cycles<=0;surf_beats<=0;surf_wait<=0;
   surf_block<=0;surf_address_wait<=0;index_beats<=0;index_wait<=0;
   out_beats<=0;out_block<=0;out_address_wait<=0;index_address_wait<=0;
   surf_pending<=0;index_pending<=0;
  end else begin
   previous_start<=start;
   if(start&&!previous_start) begin
    active<=1;cycles<=0;surf_beats<=0;surf_wait<=0;surf_block<=0;
    surf_address_wait<=0;index_beats<=0;index_wait<=0;out_beats<=0;
    out_block<=0;out_address_wait<=0;index_address_wait<=0;
    surf_pending<=0;index_pending<=0;
   end else if(active) begin
    cycles<=cycles+1;
    surf_beats<=surf_beats+surf_r;
    surf_wait<=surf_wait+((surf_pending!=0)&&!surf_rvalid);
    surf_block<=surf_block+(surf_rvalid&&!surf_rready);
    surf_address_wait<=surf_address_wait+(surf_arvalid&&!surf_arready);
    index_beats<=index_beats+index_r;
    index_wait<=index_wait+((index_pending!=0)&&!index_rvalid);
    index_address_wait<=index_address_wait+(index_arvalid&&!index_arready);
    out_beats<=out_beats+(out_wvalid&&out_wready);
    out_block<=out_block+(out_wvalid&&!out_wready);
    out_address_wait<=out_address_wait+(out_awvalid&&!out_awready);
    // Counts requested beats, not cycles where an idle master happens to have
    // RREADY high. Simultaneous requests/completions update the same credit.
    surf_pending<=surf_pending+(surf_ar?({8'b0,surf_arlen}+16'd1):16'd0)-(surf_r?16'd1:16'd0);
    index_pending<=index_pending+(index_ar?({8'b0,index_arlen}+16'd1):16'd0)-(index_r?16'd1:16'd0);
    if(done)active<=0;
   end
  end
 end
endmodule
