`timescale 1ns / 1ps

module mux81(
	output [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input [15:0] ValIn2,
	input [15:0] ValIn3,
	input [15:0] ValIn4,
	input [15:0] ValIn5,
	input [15:0] ValIn6,
	input [15:0] ValIn7,
	input [2:0] sel
	);

	assign ValOut = (sel == 3'b000) ? ValIn0 :
		(sel == 3'b001) ? ValIn1 :
		(sel == 3'b010) ? ValIn2 :
		(sel == 3'b011) ? ValIn3 :
		(sel == 3'b100) ? ValIn4 :
		(sel == 3'b101) ? ValIn5 :
		(sel == 3'b110) ? ValIn6 :
		ValIn7;
endmodule
