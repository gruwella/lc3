`timescale 1ns / 1ps

//typedef enum {start, fetch0, fetch1, fetch2, decode, ex_ld1, ex_ld2, ex_st1, ex_st2, ex_ldi1, ex_ldi2, ex_ldi3, ex_ldi4, ex_sti1, ex_sti2, ex_sti3, ex_sti4, ex_str1, ex_str2, ex_ldr1, ex_ldr2} state_type;
//typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;


module decodeFSM(aluControl,SR1,SR2,DR,selPC,selEAB1,selEAB2,enaALU,regWE,flagWE,enaMARM,selMAR,enaPC,ldPC,ldIR,ldMAR,ldMDR,selMDR,memWE,enaMDR,IR,N,Z,P,clk,reset);
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
	//parameter LDR =    15'b100000000000000;

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

module register(dout, clk, din, reset, load);
	input clk, reset, load;
	input [15:0] din;
	output reg [15:0] dout;

	always @(posedge clk)
		if (reset) dout <= 16'd0;
		else if (load) dout <= din;
endmodule


module ts_driver ( dout, din, ctrl );
	input [15:0] din;
	input ctrl;
	output [15:0] dout;

	assign dout = (ctrl)? din:(16'bZZZZZZZZZZZZZZZZ);
endmodule


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



module NZP(
	output N,
	output Z,
	output P,
	input clk,
	input flagWE,
	input reset,
	input [15:0] Buss
	);
	wire Ndin, Zdin, Pdin;
	
	assign Ndin = (Buss[15] == 1'b1) ? 1'b1 : 1'b0;
	assign Zdin = (Buss == 16'b0000000000000000) ? 1'b1 : 1'b0;
	assign Pdin = ((Buss[15] == 1'b0) & (Buss != 16'b0000000000000000)) ? 1'b1 : 1'b0;

	register1bit Nflag(N, clk, Ndin, reset, flagWE);
	register1bit Zflag(Z, clk, Zdin, reset, flagWE);
	register1bit Pflag(P, clk, Pdin, reset, flagWE);
endmodule


module register1bit(dout, clk, din, reset, load);
	input clk, reset, load;
	input din;
	output reg dout;

	always @(posedge clk)
		if (reset) dout <= 1'd0;
		else if (load) dout <= din;
endmodule


module RegFile(
	output [15:0] Ra,
	output [15:0] Rb,
	input clk,
	input regWE,
	input reset,
	input [2:0] DR,
	input [2:0] SR1,
	input [2:0] SR2,
	input [15:0] Buss
	);
	wire [7:0] decodedWE;
	wire R0WE, R1WE, R2WE, R3WE, R4WE, R5WE, R6WE, R7WE;
	wire [15:0] R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out;
	
	and(R0WE, decodedWE[0], regWE);
	and(R1WE, decodedWE[1], regWE);
	and(R2WE, decodedWE[2], regWE);
	and(R3WE, decodedWE[3], regWE);
	and(R4WE, decodedWE[4], regWE);
	and(R5WE, decodedWE[5], regWE);
	and(R6WE, decodedWE[6], regWE);
	and(R7WE, decodedWE[7], regWE);
	
	decoder81 decodeWE(decodedWE, DR);
	
	register R0(R0out, clk, Buss, reset, R0WE);
	register R1(R1out, clk, Buss, reset, R1WE);
	register R2(R2out, clk, Buss, reset, R2WE);
	register R3(R3out, clk, Buss, reset, R3WE);
	register R4(R4out, clk, Buss, reset, R4WE);
	register R5(R5out, clk, Buss, reset, R5WE);
	register R6(R6out, clk, Buss, reset, R6WE);
	register R7(R7out, clk, Buss, reset, R7WE);
	
	mux81 readRa(Ra, R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out, SR1);
	mux81 readRb(Rb, R0out, R1out, R2out, R3out, R4out, R5out, R6out, R7out, SR2);
endmodule


module PC(
	output [15:0] PC,
	input clk,
	input reset,
	input ldPC,
	input [15:0] eabOut,
	input [15:0] Buss,
	input [1:0] selPC
	);
	wire [15:0] selPCout, PCinc;

	assign PCinc = PC + 16'b0000000000000001;
	mux41 PCMux(selPCout, PCinc, eabOut, Buss, 16'bZZZZZZZZZZZZZZZZ, selPC);
	
	register PCregister(PC, clk, selPCout, reset, ldPC);
endmodule

module mux21(
	input [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input sel
	);

	assign ValOut = (sel == 1'b0) ? ValIn0 : ValIn1;
endmodule


module mux41(
	output [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input [15:0] ValIn2,
	input [15:0] ValIn3,
	input [1:0] sel
	);

	assign ValOut = (sel == 2'b00) ? ValIn0 :
		(sel == 2'b01) ? ValIn1 :
		(sel == 2'b10) ? ValIn2 :
		ValIn3;
endmodule


module mux81(
	output [15:0] ValOut,
	input [15:0] ValIn0,
	input [15:0] ValIn1,
	input [15:0] ValIn2,
	input [15:0] ValIn3,
	input [15:0] ValIn4,
	input [15:0] ValIn5,
	input [15:0] ValIn6,
	input [15:0] ValIn7,
	input [2:0] sel
	);

	assign ValOut = (sel == 3'b000) ? ValIn0 :
		(sel == 3'b001) ? ValIn1 :
		(sel == 3'b010) ? ValIn2 :
		(sel == 3'b011) ? ValIn3 :
		(sel == 3'b100) ? ValIn4 :
		(sel == 3'b101) ? ValIn5 :
		(sel == 3'b110) ? ValIn6 :
		ValIn7;
endmodule


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


module decoder81(
	output [7:0] DecodeOut,
	input [2:0] sel
	);

	assign DecodeOut = (sel == 3'b000) ? 8'b00000001 :
		(sel == 3'b001) ? 8'b00000010 :
		(sel == 3'b010) ? 8'b00000100 :
		(sel == 3'b011) ? 8'b00001000 :
		(sel == 3'b100) ? 8'b00010000 :
		(sel == 3'b101) ? 8'b00100000 :
		(sel == 3'b110) ? 8'b01000000 :
		8'b10000000;
endmodule

module decoder416(
	input [3:0] sel,
	output [3:0] decode
	);

	assign decode = (sel == 4'b0000) ? 5'b01000 :
		(sel == 4'b0001) ? 5'b00100 :
		(sel == 4'b0010) ? 5'b01001 :
		(sel == 4'b0011) ? 5'b01100 :
		(sel == 4'b0100) ? 5'b00110 :
		(sel == 4'b0101) ? 5'b00011 :
		(sel == 4'b0110) ? 5'b00000 :
		(sel == 4'b0111) ? 5'b00000 :
		(sel == 4'b1000) ? 5'b00000 :
		(sel == 4'b1001) ? 5'b00101 :
		(sel == 4'b1010) ? 5'b00000 :
		(sel == 4'b1011) ? 5'b00000 :
		(sel == 4'b1100) ? 5'b01111 :
		(sel == 4'b1101) ? 5'b00000 :
		(sel == 4'b1110) ? 5'b00000 :
		5'b00000;
endmodule

module state(dout, clk, din, reset, load);
	input clk, reset, load;
	input [14:0] din;
	output reg [14:0] dout;

	always @(posedge clk)
		if (reset) dout <= 15'b000000000000001;
		else if (load) dout <= din;
endmodule