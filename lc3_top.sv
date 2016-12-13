// File Name: risc_top.sv
// This file is the top level of a system that tests a simple RISC processor.

`default_nettype none
`timescale 1ns / 100ps
module lc3_top;

	bit clk;
	
	// Clock generator
	initial begin
		clk = 0;
		forever #50 clk = ~clk;
	end
	
	//Interfaces
	//mem_if dut_mem_if(clk);
	test_if tbdut_if(clk);
	
	// Test bench
	lc3_tb my_tb();
	//memory dut_mem(.clk(clk), .memWE(dut_mem_if.memwe), .memOut(dut_mem_if.memOut), .reset(dut_mem_if.reset), .mdrOut(dut_mem_if.mdr), .MARReg(dut_mem_if.mar));
	memory dut_mem(.clk(clk), .memWE(tbdut_if.memwe), .memOut(tbdut_if.memOut), .reset(tbdut_if.reset), .mdrOut(tbdut_if.mdr), .MARReg(tbdut_if.mar));
	
	// DUT
	ammon_lc3 my_lc3(.clk(tbdut_if.clk),
        .reset(tbdut_if.reset), 
        .memwe(tbdut_if.memwe), 
        .mdr(tbdut_if.mdr), 
        .mar(tbdut_if.mar), 
        .memOut(tbdut_if.memOut), 
        .pc(tbdut_if.pc), 
        .n_flag(tbdut_if.n_flag), 
        .z_flag(tbdut_if.z_flag), 
        .p_flag(tbdut_if.p_flag), 
        .r0_out(tbdut_if.r0), 
        .r1_out(tbdut_if.r1), 
        .r2_out(tbdut_if.r2), 
        .r3_out(tbdut_if.r3), 
        .r4_out(tbdut_if.r4), 
        .r5_out(tbdut_if.r5), 
        .r6_out(tbdut_if.r6), 
        .r7_out(tbdut_if.r7));
	
endmodule
