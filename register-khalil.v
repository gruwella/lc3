`timescale 1ns / 1ps

module register(dout, clk, din, reset, load);
	input clk, reset, load;
	input [15:0] din;
	output reg [15:0] dout;

	always @(posedge clk)
		if (reset) dout <= 16'd0;
		else if (load) dout <= din;
endmodule
