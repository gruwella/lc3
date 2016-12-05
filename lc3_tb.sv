// File Name: risc_tb.v
// This file tests the RISC processor with constrained random testing.

`default_nettype none
`timescale 1ns / 100ps
program automatic risc_tb #(ADDRESS_WIDTH=8); //risc_if my_if);

	import risc_pkg::*;
	Environment #(ADDRESS_WIDTH) env;
	Driver_cbs_coverage #(ADDRESS_WIDTH) dcv;
	virtual risc_if #(ADDRESS_WIDTH).TB my_if;
	
	initial begin
		my_if = $root.risc_top.my_if.TB;
		env = new(my_if);
		env.build();
		begin
			dcv = new();
			env.drv.cbs.push_back(dcv);
		end
		env.run();
	end
	
endprogram