`timescale 1ns / 1ps

module MARMux(
	output [15:0] MARMuxOut,
	input [7:0] IR,
	input [15:0] eabOut,
	input selMAR
	);
	wire [15:0] IRExtend;
	
	assign IRExtend = {8'd0, IR};
	assign MARMuxOut = selMAR ? IRExtend : eabOut;
endmodule
