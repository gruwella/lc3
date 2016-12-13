// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface test_if (input bit clk);
	logic reset;
 	logic memwe;
	logic [15:0] mdr, memOut;
	logic [15:0] mar;
	logic [15:0] pc, r0, r1, r2, r3, r4, r5, r6, r7;
	logic n_flag, z_flag, p_flag;
	
	modport TB2DUT(output reset, input clk, memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag, memOut);
	modport DUT2TB(input clk, reset, memOut, output memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag);
	modport DUT2MEM(input clk, reset, mar, mdr, memwe, memOut);
endinterface