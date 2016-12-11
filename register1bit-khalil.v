`timescale 1ns / 1ps

module register1bit(dout, clk, din, reset, load);
	input clk, reset, load;
	input din;
	output reg dout;

	always @(posedge clk)
		if (reset) dout <= 1'd0;
		else if (load) dout <= din;
endmodule
