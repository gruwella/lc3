`timescale 1ns / 1ps

module decoder81(
	output [7:0] DecodeOut,
	input [2:0] sel
	);

	assign DecodeOut = (sel == 3'b000) ? 8'b00000001 :
		(sel == 3'b001) ? 8'b00000010 :
		(sel == 3'b010) ? 8'b00000100 :
		(sel == 3'b011) ? 8'b00001000 :
		(sel == 3'b100) ? 8'b00010000 :
		(sel == 3'b101) ? 8'b00100000 :
		(sel == 3'b110) ? 8'b01000000 :
		8'b10000000;
endmodule
