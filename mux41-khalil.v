`timescale 1ns / 1ps

module mux41(
	output [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input [15:0] ValIn2,
	input [15:0] ValIn3,
	input [1:0] sel
	);

	assign ValOut = (sel == 2'b00) ? ValIn0 :
		(sel == 2'b01) ? ValIn1 :
		(sel == 2'b10) ? ValIn2 :
		ValIn3;
endmodule
