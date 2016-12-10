// File Name: risc_top.sv
// This file is the top level of a system that tests a simple RISC processor.

`default_nettype none
`timescale 1ns / 100ps
module lc3_top;

	bit clk, reset;
	parameter ADDRESS_WIDTH = 16;
	
	// Clock generator
	initial begin
		clk = 0;
		forever #50 clk = ~clk;
	end
	
	//Interfaces
	mem_if #(ADDRESS_WIDTH) dut_mem_if(clk);
	test_if tbdut_if(clk);
	
	// Test bench
	lc3_tb #(ADDRESS_WIDTH) my_tb();
	memory dut_mem(clk, dut_mem_if.MEM2TB);
	
	// DUT
	ammon_lc3 my_lc3(clk, tbdut_if.DUT2TB);
	
endmodule