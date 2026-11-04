`timescale 1ns/1ps
module tb_metrics;
 reg clk=0;always #5 clk=~clk;
 reg reset=1,start=0,done=0;
 reg sav=0,sar=0,srv=0,srr=0,iav=0,iar=0,irv=0,irr=0;
 reg [7:0] sal=0,ial=0;
 reg oav=0,oar=0,owv=0,owr=0;
 wire [383:0] metrics;
 renderer_metrics dut(clk,reset,start,done,sav,sar,sal,srv,srr,iav,iar,ial,irv,irr,oav,oar,owv,owr,metrics);
 task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
 task expect_word(input integer n,input integer value);begin
  if(metrics[n*32+:32]!==value)begin
   $display("FAIL word %0d got %0d expected %0d",n,metrics[n*32+:32],value);$fatal(1);
  end
 end endtask
 initial begin
  tick();reset=0;start=1;tick();start=0;
  // Request four surfel beats and two index beats; also stalled write address.
  sav=1;sar=1;sal=3;iav=1;iar=1;ial=1;oav=1;tick();
  sav=0;iav=0;oav=0;tick(); // one cycle waiting on both outstanding reads
  srv=1;srr=0;irv=1;irr=1;owv=1;tick(); // surf backpressure, first index beat
  srr=1;owr=1;tick(); // first surf beat, second index beat, first write beat
  irv=0;owv=0;tick(); // second surf beat
  tick();             // third surf beat
  done=1;tick();       // fourth surf beat, finish: exactly seven cycles
  done=0;srv=0;srr=0;irr=0;owr=0;
  expect_word(0,32'h4c4a0101);expect_word(1,7);expect_word(2,4);
  expect_word(3,1);expect_word(4,1);expect_word(5,0);expect_word(6,2);
  expect_word(7,1);expect_word(8,1);expect_word(9,1);expect_word(10,1);expect_word(11,0);
  repeat(4)tick();expect_word(1,7); // completion snapshot is stable
  start=1;tick();repeat(3)tick(); // held ap_start must not repeatedly reset
  done=1;tick();done=0;tick();expect_word(1,4);expect_word(2,0);
  start=0;tick();start=1;tick();start=0;done=1;tick();expect_word(1,1);
  reset=1;tick();expect_word(1,0);expect_word(2,0);
  $display("PASS metrics: outstanding credits, read/write stalls, done freeze, held start, restart, reset");$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
