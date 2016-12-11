`timescale 1ns / 1ps

module NZP(
	output N,
	output Z,
	output P,
	input clk,
	input flagWE,
	input reset,
	input [15:0] Buss
	);
	wire Ndin, Zdin, Pdin;
	
	assign Ndin = (Buss[15] == 1'b1) ? 1'b1 : 1'b0;
	assign Zdin = (Buss == 16'b0000000000000000) ? 1'b1 : 1'b0;
	assign Pdin = ((Buss[15] == 1'b0) & (Buss != 16'b0000000000000000)) ? 1'b1 : 1'b0;

	register1bit Nflag(N, clk, Ndin, reset, flagWE);
	register1bit Zflag(Z, clk, Zdin, reset, flagWE);
	register1bit Pflag(P, clk, Pdin, reset, flagWE);
endmodule
