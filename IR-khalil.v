`timescale 1ns / 1ps

module IR(
	output [15:0] IR,
	input clk,
	input ldIR,
	input reset,
	input [15:0] Buss
	);

	reg[15:0] ir_reg;

		always@(posedge clk)
						begin
  								if (reset)
					    begin
   							    ir_reg = 16'b0;
  						end
  						else if(ldIR)
  						begin
    					ir_reg = Buss;
  						end
		end

assign IR = ir_reg;
endmodule
