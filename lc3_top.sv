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
	
	lc3_if #(ADDRESS_WIDTH) my_if(clk);
	lc3_tb #(ADDRESS_WIDTH) my_tb();
	//TODO: instance the LC3
	
endmodule