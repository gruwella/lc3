`timescale 1ns / 1ps

module PC(
	output [15:0] PC,
	input clk,
	input reset,
	input ldPC,
	input [15:0] eabOut,
	input [15:0] Buss,
	input [1:0] selPC
	);
	wire [15:0] selPCout, PCinc;

	assign PCinc = PC + 16'b0000000000000001;
	mux41 PCMux(selPCout, PCinc, eabOut, Buss, 16'bZZZZZZZZZZZZZZZZ, selPC);
	
	register PCregister(PC, clk, selPCout, reset, ldPC);
endmodule
