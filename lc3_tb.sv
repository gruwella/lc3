// File Name: risc_tb.v
// This file tests the RISC processor with constrained random testing.

`default_nettype none
`timescale 1ns / 100ps
program automatic lc3_tb #(ADDRESS_WIDTH=8) (reset);

	input logic reset;
	import lc3_pkg::*;
	Environment #(ADDRESS_WIDTH) env;
	Driver_cbs_coverage #(ADDRESS_WIDTH) dcv;
	Driver_cbs_expected #(ADDRESS_WIDTH) dex;
	virtual test_if #(ADDRESS_WIDTH).TB2DUT tbdut_if;
	virtual mem_if #(ADDRESS_WIDTH).TB2MEM dutmem_if;
	
	initial begin
		tbdut_if = $root.lc3_top.dut_mem_if.TB2DUT;
		dutmem_if = $root.lc3_top.dut_mem_if.TB2MEM;
		env = new(tbdut_if, dutmem_if);
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