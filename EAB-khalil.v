`timescale 1ns / 1ps

module EAB(
	output [15:0] eabOut,
	input [10:0] IR,
	input [15:0] Ra,
	input [15:0] PC,
	input selEAB1,
	input [1:0] selEAB2
	);
	wire [15:0] IRExtend10, IRExtend8, IRExtend5;
	wire [15:0] mux21out, mux41out;
	
	assign IRExtend10 = {{4'b0101{IR[10]}}, IR[10:0]};
	assign IRExtend8 = {{4'b0111{IR[8]}}, IR[8:0]};
	assign IRExtend5 = {{4'b1010{IR[5]}}, IR[5:0]};

	mux21 ADDR1MUX(mux21out, PC, Ra, selEAB1);
	mux41 ADDR2MUX(mux41out, 16'b0000000000000000, IRExtend5, IRExtend8, IRExtend10, selEAB2);
	
	assign eabOut = mux21out + mux41out;
endmodule
