`timescale 1ns / 1ps

module LC3_Datapath(
	output [15:0] IR_out,
	output N,
	output Z,
	output P,
	input clk,
	input reset,
	input [1:0] aluControl,
	input [2:0] SR1,
	input [2:0] SR2,
	input [2:0] DR,
	input [1:0] selPC,
	input [1:0] selEAB2,
	input enaALU,
	input regWE,
	input enaMARM,
	input selMAR,
	input selEAB1,
	input enaPC,
	input ldPC,
	input ldIR,
	input ldMAR,
	input ldMDR,
	input selMDR,
	input memWE,
	input enaMDR,
	input flagWE
	);

	wire[15:0] MARMuxOut, Buss, PC, Ra, Rb, mdrOut, aluOut, eabOut, IR_temp;
	
	PC LC3_PC(.PC(PC), .clk(clk), .reset(reset), .ldPC(ldPC), .eabOut(eabOut), .Buss(Buss), .selPC(selPC));


	MARMux LC3_MARMux(.MARMuxOut(MARMuxOut), .IR(IR_temp[7:0]), .eabOut(eabOut), .selMAr(selMAR));


	EAB LC3_EAB( .IR(IR_temp[10:0]), .Ra(Ra), .PC(PC), .selEAB1(selEAB1), .selEAB2(selEAB2), .eabOut(eabOut));

	
	RegFile LC3_RegFile(.Rb(Rb), .Ra(Ra), .clk(clk), .regWE(regWE), .reset(reset), .DR(DR), .SR1(SR1), .SR2(SR2), .Buss(Buss));
	

	ALU LC3_ALU(.aluOut(aluOut), .Rb(Rb), .Ra(Ra), .IR(IR_temp[5:0]), .aluControl(aluControl));


	NZP LC3_NZP(.N(N), .Z(Z), .P(P), .clk(clk), .flagWE(flagWE), .reset(reset), .Buss(Buss));


	IR LC3_IR(.IR(IR_temp), .clk(clk), .ldIR(ldIR), .reset(reset), .Buss(Buss));


	Memory LC3_Memory(.mdrOut(mdrOut), .Buss(Buss), .clk(clk), .reset(reset), .ldMAR(ldMAR), .ldMDR(ldMDR), .selMDR(selMDR), .memWE(memWE));

	
	ts_driver tsd_enaMDR(.dout(Buss), .din(mdrOut), .ctrl(enaMDR));
	ts_driver tsd_enaALU(.dout(Buss), .din(aluOut), .ctrl(enaALU));
	ts_driver tsd_enaMARM(.dout(Buss), .din(MARMuxOut), .ctrl(enaMARM));
	ts_driver tsd_enaPC(.dout(Buss), .din(PC), .ctrl(enaPC));


endmodule
