// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface lc3_if #(ADDRESS_WIDTH=16) (input bit clk);

	logic reset, memwe;
	logic [15:0] mdr, memOut;
	logic [ADDRESS_WIDTH-1:0] mar;
	
	clocking cb @(posedge clk);
		output memOut;
		input memwe;
		input mdr;
		input mar;
	endclocking
	
	//(clk, reset, memWE, mdrOut, MARReg, memOut)
	modport DUT(input reset, output memwe, mdr, mar, input memOut);
	modport MEM(input reset, memwe, mdr, mar, output memOut);
	modport TB(clocking cb, output reset);

endinterface