`timescale 1ns/1ps
module tb_control_metrics;
 reg clk=0;always #5 clk=~clk;
 reg reset=1;reg [7:0] awaddr=0,araddr=0;reg awvalid=0,wvalid=0,bready=0,arvalid=0,rready=0;
 reg [31:0] wdata=0;reg [3:0] wstrb=15;
 wire awready,wready,bvalid,arready,rvalid;wire [31:0] rdata,width,mode;
 wire [1:0] bresp,rresp;reg [383:0] metrics;
 lj_raster_control_s_axi dut(.ACLK(clk),.ARESET(reset),.ACLK_EN(1'b1),
  .AWADDR(awaddr),.AWVALID(awvalid),.AWREADY(awready),.WDATA(wdata),.WSTRB(wstrb),.WVALID(wvalid),.WREADY(wready),.BRESP(bresp),.BVALID(bvalid),.BREADY(bready),
  .ARADDR(araddr),.ARVALID(arvalid),.ARREADY(arready),.RDATA(rdata),.RRESP(rresp),.RVALID(rvalid),.RREADY(rready),.width(width),.mode(mode),
  .ap_done(1'b0),.ap_ready(1'b0),.ap_idle(1'b1),.metrics(metrics));
 task write_reg(input [7:0] address,input [31:0] value);begin
  @(negedge clk);awaddr=address;awvalid=1;
  do @(posedge clk);while(!awready);
  @(negedge clk);awvalid=0;repeat(3)@(negedge clk);wdata=value;wvalid=1;
  do @(posedge clk);while(!wready);
  @(negedge clk);wvalid=0;wait(bvalid);repeat(3)@(negedge clk);
  if(!bvalid||bresp!=0)$fatal(1,"write response not held");bready=1;@(negedge clk);bready=0;
 end endtask
 task read_reg(input [7:0] address,input [31:0] expected);begin
  @(negedge clk);araddr=address;arvalid=1;
  do @(posedge clk);while(!arready);
  @(negedge clk);arvalid=0;wait(rvalid);
  repeat(4)begin @(negedge clk);if(!rvalid||rresp!=0||rdata!==expected)$fatal(1,"read mismatch address=%h actual=%h expected=%h",address,rdata,expected);end
  rready=1;@(negedge clk);rready=0;
 end endtask
 integer i;
 initial begin
  for(i=0;i<12;i=i+1)metrics[i*32+:32]=32'ha0000000+i;
  repeat(3)@(negedge clk);reset=0;
  write_reg(8'h58,37);read_reg(8'h58,37);if(width!=37)$fatal(1,"legacy width write broken");
  write_reg(8'ha0,3);read_reg(8'ha0,3);
  for(i=0;i<12;i=i+1)read_reg(8'hb0+i*4,32'ha0000000+i);
  write_reg(8'hb4,32'hdeadbeef);read_reg(8'hb4,32'ha0000001);read_reg(8'ha0,3);
  $display("PASS control metrics: legacy registers, all counter offsets, read-only extension, independent AW/W, B/R backpressure");$finish;
 end
 initial begin #30000;$fatal(1,"timeout");end
endmodule
