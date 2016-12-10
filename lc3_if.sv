// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface mem_if #(ADDRESS_WIDTH=16) (input bit clk);

	logic memwe, reset;
	logic [15:0] mdr, memOut;
	logic [ADDRESS_WIDTH-1:0] mar;
	
	modport MEM2TB(input reset, memwe, mdr, mar, output memOut);
	modport TB2MEM(output reset, memwe, mdr, mar, input memOut);

endinterface

interface test_if #(ADDRESS_WIDTH=16) (input bit clk);
	logic memwe, reset;
	logic [15:0] mdr, memOut;
	logic [ADDRESS_WIDTH-1:0] mar;
	logic [15:0] pc, r0, r1, r2, r3, r4, r5, r6, r7;
	logic n_flag, z_flag, p_flag;
	
	modport TB2DUT(output reset, memOut, input memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag);
	modport DUT2TB(input memOut, reset, output memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag);
endinterface