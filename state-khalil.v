`timescale 1ns / 1ps

module state(dout, clk, din, reset, load);
	input clk, reset, load;
	input [14:0] din;
	output reg [14:0] dout;

	always @(posedge clk)
		if (reset) dout <= 15'b000000000000001;
		else if (load) dout <= din;
endmodule
