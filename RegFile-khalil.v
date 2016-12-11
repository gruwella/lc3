`timescale 1ns / 1ps

module RegFile(
	output [15:0] Ra,
	output [15:0] Rb,
	input clk,
	input regWE,
	input reset,
	input [2:0] DR,
	input [2:0] SR1,
	input [2:0] SR2,
	input [15:0] Buss
	);
	wire [7:0] decodedWE;
	wire R0WE, R1WE, R2WE, R3WE, R4WE, R5WE, R6WE, R7WE;
	wire [15:0] R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out;
	
	and(R0WE, decodedWE[0], regWE);
	and(R1WE, decodedWE[1], regWE);
	and(R2WE, decodedWE[2], regWE);
	and(R3WE, decodedWE[3], regWE);
	and(R4WE, decodedWE[4], regWE);
	and(R5WE, decodedWE[5], regWE);
	and(R6WE, decodedWE[6], regWE);
	and(R7WE, decodedWE[7], regWE);
	
	decoder81 decodeWE(decodedWE, DR);
	
	register R0(R0out, clk, Buss, reset, R0WE);
	register R1(R1out, clk, Buss, reset, R1WE);
	register R2(R2out, clk, Buss, reset, R2WE);
	register R3(R3out, clk, Buss, reset, R3WE);
	register R4(R4out, clk, Buss, reset, R4WE);
	register R5(R5out, clk, Buss, reset, R5WE);
	register R6(R6out, clk, Buss, reset, R6WE);
	register R7(R7out, clk, Buss, reset, R7WE);
	
	mux81 readRa(Ra, R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out, SR1);
	mux81 readRb(Rb, R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out, SR2);
endmodule
