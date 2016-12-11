`timescale 1ns / 1ps

module decodeFSM(aluControl,SR1,SR2,DR,selPC,selEAB,selEAB2,enaALU,regWE,flagWE,enaMARM,selMAR,enaPC,ldPC,ldIR,ldMAR,selMDR,memWE,enaMDR,IR,N,Z,P,clk,reset);
	output [1:0] aluControl;
	output [2:0] SR1;
	output [2:0] SR2;
	output [2:0] DR;
	output [1:0] selPC;
	output selEAB1;
	output [1:0] selEAB2;
	output enaALU;
	output regWE;
	output flagWE;
	output enaMARM;
	output selMAR;
	output enaPC;
	output ldPC;
	output ldIR;
	output ldMAR;
	output ldMDR;
	output selMDR;
	output memWE;
	output enaMDR;
	input [15:0] IR;
	input N, Z, P, clk, reset;
	
	wire [3:0] opcode;
	wire [14:0] currentState, nextState;
	wire [19:0] signals;
	
	assign opcode = IR[15:12];

	parameter FETCH0 = 15'b000000000000001;
	parameter FETCH1 = 15'b000000000000010;
	parameter FETCH2 = 15'b000000000000100;
	parameter DECODE = 15'b000000000001000;
	parameter ALU =    15'b000000000010000;
	parameter JSR0 =   15'b000000000100000;
	parameter JSR1 =   15'b000000001000000;
	parameter BR =     15'b000000010000000;
	parameter LD0 =    15'b000000100000000;
	parameter LD1 =    15'b000001000000000;
	parameter LD2 =    15'b000010000000000;
	parameter ST0 =    15'b000100000000000;
	parameter ST1 =    15'b001000000000000;
	parameter ST2 =    15'b010000000000000;
	parameter JMP =    15'b100000000000000;

	state FSM_State(currentState, clk, nextState, reset, 1'b1);
	
	assign nextState = (currentState == FETCH0) ? FETCH1 :
		(currentState == FETCH1) ? FETCH2 :
		(currentState == FETCH2) ? DECODE :
		(currentState == DECODE & ((opcode == 4'b0101) | (opcode == 4'b0001) | (opcode == 4'b1001))) ? ALU :
		(currentState == DECODE & opcode == 4'b0100 & IR[11] == 1'b1) ? JSR0 :
		(currentState == DECODE & opcode == 4'b0000 & ((N & IR[11]) | (Z & IR[10]) | (P & IR[9]))) ? BR :
		(currentState == DECODE & opcode == 4'b0010) ? LD0 :
		(currentState == DECODE & opcode == 4'b0011) ? ST0 :
		(currentState == DECODE & opcode == 4'b1100) ? JMP :
		(currentState == ALU) ? FETCH0 :
		(currentState == JSR0) ? JSR1 :
		(currentState == JSR1) ? FETCH0 :
		(currentState == BR) ? FETCH0 :
		(currentState == LD0) ? LD1 :
		(currentState == LD1) ? LD2 :
		(currentState == LD2) ? FETCH0 :
		(currentState == ST0) ? ST1 :
		(currentState == ST1) ? ST2 :
		(currentState == ST2) ? FETCH0 :
		(currentState == JMP) ? FETCH0 :
		FETCH0;
		
	assign signals = (currentState == FETCH0)    ? 20'b00000000000010010000 :
		(currentState == FETCH1)                  ? 20'b00000000000001001100 :
		(currentState == FETCH2)                  ? 20'b00000000000000100001 :
		(currentState == DECODE)                  ? 20'b00000000000000000000 :
		(currentState == ALU & (opcode == 4'b0101)) ? 20'b10000001110000000000 :
		(currentState == ALU & (opcode == 4'b0001)) ? 20'b01000001110000000000 :
		(currentState == ALU & (opcode == 4'b1001)) ? 20'b11000001110000000000 :
		(currentState == JSR0)                    ? 20'b00000000100010000000 :
		(currentState == JSR1)                    ? 20'b00010110000001000000 :
		(currentState == BR)                      ? 20'b00010100000001000000 :
		(currentState == LD0)                     ? 20'b00000100001000010000 :
		(currentState == LD1)                     ? 20'b00000000000000001100 :
		(currentState == LD2)                     ? 20'b00000000110000000001 :
		(currentState == ST0)                     ? 20'b00000100001000010000 :
		(currentState == ST1)                     ? 20'b00000001000000001000 :
		(currentState == ST2)                     ? 20'b00000000000000000010 :
		(currentState == JMP)                     ? 20'b00100001000001000000 :
		20'b00000000000000000000;
		
	assign aluControl = signals[19:18];
	assign selPC = signals[17:16];
	assign selEAB1 = signals[15];
	assign selEAB2 = signals[14:13];
	assign enaALU = signals[12];
	assign regWE = signals[11];
	assign flagWE = signals[10];
	assign enaMARM = signals[9];
	assign selMAR = signals[8];
	assign enaPC = signals[7];
	assign ldPC = signals[6];
	assign ldIR = signals[5];
	assign ldMAR = signals[4];
	assign ldMDR = signals[3];
	assign selMDR = signals[2];
	assign memWE = signals[1];
	assign enaMDR = signals[0];
	
	assign SR1 = (currentState == ST2) ? IR[11:9] : IR[8:6];
	assign SR2 = IR[2:0];
	assign DR = (currentState == JSR0) ? 3'b111 : IR[11:9];
endmodule
