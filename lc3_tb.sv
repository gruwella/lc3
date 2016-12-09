// File Name: risc_tb.v
// This file tests the RISC processor with constrained random testing.

`default_nettype none
`timescale 1ns / 100ps
program automatic lc3_tb #(ADDRESS_WIDTH=8);

	import lc3_pkg::*;
	Environment #(ADDRESS_WIDTH) env;
	Driver_cbs_coverage #(ADDRESS_WIDTH) dcv;
	Driver_cbs_expected #(ADDRESS_WIDTH) dex;
	virtual lc3_if #(ADDRESS_WIDTH).TB my_if;
	
	initial begin
		my_if = $root.lc3_top.my_if.TB;
		env = new(my_if);
		env.build();
		begin
			dcv = new();
			dex = new();
			env.drv.cbs.push_back(dcv);
			env.drv.cbs.push_back(dex);
		end
		env.run();
	end
	
endprogram