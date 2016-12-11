`timescale 1ns / 1ps

module decoder416(
	input [3:0] sel,
	output [3:0] decode
	);

	assign decode = (sel == 4'b0000) ? 5'b01000 :
		(sel == 4'b0001) ? 5'b00100 :
		(sel == 4'b0010) ? 5'b01001 :
		(sel == 4'b0011) ? 5'b01100 :
		(sel == 4'b0100) ? 5'b00110 :
		(sel == 4'b0101) ? 5'b00011 :
		(sel == 4'b0110) ? 5'b00000 :
		(sel == 4'b0111) ? 5'b00000 :
		(sel == 4'b1000) ? 5'b00000 :
		(sel == 4'b1001) ? 5'b00101 :
		(sel == 4'b1010) ? 5'b00000 :
		(sel == 4'b1011) ? 5'b00000 :
		(sel == 4'b1100) ? 5'b01111 :
		(sel == 4'b1101) ? 5'b00000 :
		(sel == 4'b1110) ? 5'b00000 :
		5'b00000;
endmodule
