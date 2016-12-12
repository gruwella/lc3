// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface mem_if (input bit clk);

	logic memwe, reset;
	logic [15:0] mdr, memOut;
	logic [15:0] mar;
	
	modport MEM2TB(input clk, reset, memwe, mdr, mar, output memOut);
	modport TB2MEM(output clk, reset, memwe, mdr, mar, input memOut);

endinterface

interface test_if (input bit clk);
	logic memwe, reset;
	logic [15:0] mdr, memOut;
	logic [15:0] mar;
	logic [15:0] pc, r0, r1, r2, r3, r4, r5, r6, r7;
	logic n_flag, z_flag, p_flag;
	
	clocking cb @(posedge clk);
		output memOut;
		input memwe;
		input mdr;
		input mar;
		input pc;
		input r0;
		input r1;
		input r2;
		input r3;
		input r4;
		input r5;
		input r6;
		input r7;
		input n_flag;
		input z_flag;
		input p_flag;
	endclocking
	
	modport TB2DUT(clocking cb, output reset);
	modport DUT2TB(input clk, memOut, reset, output memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag);
endinterface