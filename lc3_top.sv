// File Name: risc_top.sv
// This file is the top level of a system that tests a simple RISC processor.

`default_nettype none
`timescale 1ns / 1ns
module lc3_top;

	bit clk;
	
	// Clock generator
	initial begin
		clk = 0;
		forever #50 clk = ~clk;
	end
	
	
	
	//Interfaces
	test_if tbdut_if_0(clk);
	test_if tbdut_if_1(clk);
	
	// Test bench
	lc3_tb my_tb();
	
	// Memory
	memory dut_mem_0(.clk(clk), .memWE(tbdut_if_0.memwe), .memOut(tbdut_if_0.memOut), .reset(tbdut_if_0.reset), .mdrOut(tbdut_if_0.mdr), .MARReg(tbdut_if_0.mar));
	memory dut_mem_1(.clk(clk), .memWE(tbdut_if_1.memwe), .memOut(tbdut_if_1.memOut), .reset(tbdut_if_1.reset), .mdrOut(tbdut_if_1.mdr), .MARReg(tbdut_if_1.mar));
	
	// DUT
	ammon_lc3 my_lc3_0(.clk(tbdut_if_0.clk),
        .reset(tbdut_if_0.reset), 
        .memwe(tbdut_if_0.memwe), 
        .mdr(tbdut_if_0.mdr), 
        .mar(tbdut_if_0.mar), 
        .memOut(tbdut_if_0.memOut));
	
	khalil_LC3 my_lc3_1(.clk(tbdut_if_1.clk),
        .reset(tbdut_if_1.reset), 
        .memwe(tbdut_if_1.memwe), 
        .mdr(tbdut_if_1.mdr), 
        .mar(tbdut_if_1.mar), 
        .memOut(tbdut_if_1.memOut));
		
endmodule
