`timescale 1ns / 1ps

module ALU(
	output [15:0] aluOut,
	input [1:0] aluControl,
	input [5:0] IR,
	input [15:0] Ra,
	input [15:0] Rb
	);
	wire [15:0] IRb, ALUb, ALUa;
	
	assign ALUa = Ra;
	assign ALUb = (IR[5]) ? IRb : Rb;
	
	assign IRb = {{6'd11{IR[4]}}, IR[4:0]};

	assign aluOut = (aluControl == 2'b00) ? ALUa :
		(aluControl == 2'b01) ? (ALUa + ALUb) :
		(aluControl == 2'b10) ? (ALUa & ALUb) :
		~ALUa;
endmodule
