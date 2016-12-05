// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface lc3_if #(ADDRESS_WIDTH=8) (input bit clk);

	logic rst;
	logic [7:0] data_out;
	logic [ADDRESS_WIDTH-1:0] address;
	logic [7:0] data_in;
	logic write;
	
	clocking cb @(posedge clk);
		output data_out;
		input address;
		input data_in;
		input write;
	endclocking
	
	modport DUT(input rst, data_out, output address, data_in, write);
	modport TB(clocking cb, output rst);

endinterface