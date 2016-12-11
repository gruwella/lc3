`timescale 1ns / 1ps

module ts_driver ( dout, din, ctrl );
	input [15:0] din;
	input ctrl;
	output [15:0] dout;

	assign dout = (ctrl)? din:(16'bZZZZZZZZZZZZZZZZ);
endmodule
