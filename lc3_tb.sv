// File Name: risc_tb.v
// This file tests the RISC processor with constrained random testing.

`default_nettype none
`timescale 1ns / 100ps
program automatic lc3_tb();
	
	import lc3_pkg::*;
	
	Component test;
	initial begin
		test = Factory::get_test();
		test.run_test();
	end
	
endprogram