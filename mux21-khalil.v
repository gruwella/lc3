`timescale 1ns / 1ps

module mux21(
	input [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input sel
	);

	assign ValOut = (sel == 1'b0) ? ValIn0 : ValIn1;
endmodule
