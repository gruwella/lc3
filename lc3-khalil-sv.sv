`include "assert_macros.sv"
	
	typedef enum {
	  
	  start  	,
	  fetch0  	,
	  fetch1	, 
	  fetch2	,
	  decode	, 
	  not_and_add0,	
	  jsr0,
	  br0 ,
	  ld0 ,
	  ld1 ,
	  ld2 ,
	  st0 ,
	  st1 ,
	  st2 ,
	  jmp0,
	  ldi0,
	  ldi1,
	  ldi2,
	  ldi3,
	  ldi4,
	  ldr0,
	  ldr1,
	  ldr2,
	  lea0,
	  sti0,
	  sti1,
	  sti2,
	  sti3,
	  sti4,
	  str0,
	  str1,
	  str2,
	  trap0,
	  trap1,
	  jsrr0,
	  rti,
	  ioe
	  
    } stateTypedef;


module khalil_LC3(clk, reset, memwe, mdr, mar, memOut);

    output logic [15:0] mdr, mar;
	output logic memwe;
	input logic [15:0] memOut;
    input logic clk; 
    input logic reset;
	logic [15:0] pc, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out;
	logic n_flag, z_flag, p_flag;
	
	int state_logic;
	assign state_logic = Control.currentState;
	int dut_id = 1;

	logic[15:0] ir;
    logic[1:0] aluControl;
    logic[2:0] SR1;
    logic[2:0] SR2;
	  logic[2:0] DR;
    logic[1:0] selPC;
    logic[1:0] selEAB2;
    logic enaALU;
    logic regWE;
    logic enaMARM;
    logic selMAR;
    logic selEAB1;
    logic enaPC;
    logic ldPC;
    logic ldIR;
    logic enaMDR;
    logic flagWE;
	logic [15:0] Buss; 
	
// 	ERR_RESET_SHOULD_SET_ALL_REGISTERS_TO_ZERO:
//		`assert_clk(reset |-> ##1 pc==16'h0 && ir==16'h0 && mdr==16'h0 && mar==16'h0 && n_flag==0 && z_flag==0 && p_flag==0);
//	
//	ERR_FLAGS_SHOULD_NOT_BE_HIGH_AT_THE_SAME_TIME:
//		`assert_clk_xrst(!((n_flag && p_flag) || (n_flag && z_flag) || (z_flag && p_flag)));
//	
//	ERR_MEMWE_IS_HIGH_DURING_NON_STORE_INSTRUCTION:
//		`assert_clk((Control.opCode != 4'b0011) && (Control.opCode != 4'b0111) && (Control.opCode != 4'b1011) |-> !memwe);
//	
//	ERR_MEMWE_IS_HIGH_FOR_MORE_THAN_ONE_CLK_CYCLE:
//		`assert_clk(memwe |-> ##1 !memwe);
//	
//	ERR_MORE_THAN_ONE_BUS_DRIVER:
//		`assert_clk($onehot0({enaPC, enaMARM, enaMDR, enaALU})); 



	Datapath Datapath(.N(n_flag), .Z(z_flag), .P(p_flag), .IR(ir), .aluControl(aluControl), .SR1(SR1), .SR2(SR2), .DR(DR), .selPC(selPC), .selEAB2(selEAB2), .enaALU(enaALU), .regWE(regWE), .enaMARM(enaMARM), .selMAR(selMAR), .selEAB1(selEAB1), .enaPC(enaPC), .ldPC(ldPC), .ldIR(ldIR), .ldMAR(ldMAR), .ldMDR(ldMDR), .selMDR(selMDR), .memWE(memwe), .enaMDR(enaMDR), .flagWE(flagWE), .clk(clk), .reset(reset), .Buss_out(Buss), .mdrOut(mdr), .PCW(pc), .r0_out(r0_out), .r1_out(r1_out), .r2_out(r2_out), .r3_out(r3_out), .r4_out(r4_out), .r5_out(r5_out), .r6_out(r6_out), .r7_out(r7_out));
	//Datapath Datapath(.N(n_flag), .Z(z_flag), .P(p_flag), .IR(ir), .Buss_out(Buss), .*);

//	ControlUnit Control(.N(n_flag), .Z(z_flag), .P(p_flag), .IR(ir), aluControl, SR1, SR2, DR, selPC, selEAB2, enaALU, regWE, enaMARM, selMAR, selEAB1, enaPC, ldPC, ldIR, ldMAR, ldMDR, selMDR, memWE, enaMDR, flagWE, clk, reset);
	ControlUnit Control(.N(n_flag), .Z(z_flag), .P(p_flag), .IR(ir), .memWE(memwe), .*);

	//MAR MAR0(Buss, clk, reset, ldMAR, .MARReg(mar));
	MAR MAR0(.MARReg(mar), .*);	  
	MDR MDR0(.MDRReg(mdr), .*);
	//MDR MDR0(Buss, memOut, selMDR, clk, reset, ldMDR, .mdrOut(mdr));

endmodule

/*     input logic memWE,
    input logic enaMDR,
    input logic flagWE,
    input logic clk,
    input logic reset,
    output logic [15:0] Buss_out,
    input logic [15:0] mdrOut,
    output logic[15:0] PCW, */

module ControlUnit(
   input logic N,
    input logic Z,
    input logic P,
    input logic [15:0] IR,
    output logic [1:0] aluControl,
    output logic [2:0] SR1,
    output logic [2:0] SR2,
    output logic [2:0] DR,
    output logic [1:0] selPC,
    output logic [1:0] selEAB2,
    output logic enaALU,
    output logic regWE,
    output logic enaMARM,
    output logic selMAR,
    output logic selEAB1,
    output logic enaPC,
    output logic ldPC,
    output logic ldIR,
    output logic ldMAR,
    output logic ldMDR,
    output logic selMDR,
    output logic memWE,
    output logic enaMDR,
    output logic flagWE,
    input logic clk,
    input logic reset
    );
	
	
	
	
	logic[3:0] opCode;
	logic[28:0] signals;
	stateTypedef nextState;
	stateTypedef currentState;
	

	
	assign opCode = IR[15:12];
		  
  stateTypedef decoded_nextState;
  logic TB;
  
  assign state = currentState;
  
  assign TB = ((N && IR[11])||(Z&&IR[10])||(P&&IR[9]));
  
  assign decoded_nextState = ( (opCode == 4'b1001) || (opCode == 4'b0001) || (opCode == 4'b0101) ) ? not_and_add0:
						(opCode == 4'b0100 && IR[11] == 1) ? jsr0:
						(opCode == 4'b0100 && IR[11] == 0) ? jsrr0:
						(opCode == 4'b0000) ? br0:
						(opCode == 4'b0010) ? ld0:
						(opCode == 4'b0011) ? st0:
						(opCode == 4'b1100) ? jmp0: 
						(opCode == 4'b1010) ? ldi0:
						(opCode == 4'b0110)? ldr0:
						(opCode == 4'b1110)? lea0:
						(opCode == 4'b1011)? sti0:
						(opCode == 4'b0111)? str0:
						(opCode == 4'b1111)? trap0:
						(opCode == 4'b1101)? ioe:
						(opCode == 4'b1000)? rti:
						//(opCode == 4'b1000)? jmp0:	//If opcode is rti, go to jmp0 (ret) state.				
						 fetch0;						
  
  
  always_ff @(posedge clk)
    begin
		if (reset)
			currentState <= start;
		else
			currentState <= nextState;
	 end
		
	 
  always_comb
unique case(currentState)
		start			:	nextState = fetch0;
		fetch0			:	nextState = fetch1;
		fetch1			:	nextState = fetch2;
		fetch2			:	nextState = decode;
		decode			:	nextState = decoded_nextState;
		not_and_add0	:	nextState = fetch0;
		jsr0			:	nextState = fetch0;
		//jsr1			:	nextState = fetch0;
		jsrr0			: 	nextState = fetch0;
		//jsrr1			:	nextState = fetch0;
		br0				:	nextState = fetch0;
		ld0				:	nextState = ld1;
		ld1				:	nextState = ld2;
		ld2				:	nextState = fetch0;
		st0				:	nextState = st1;
		st1				:	nextState = st2;
		st2				:	nextState = fetch0;
		jmp0			:	nextState = fetch0;
		ldi0			:	nextState = ldi1;
		ldi1			:	nextState = ldi2;
		ldi2			:	nextState = ldi3;
		ldi3			:	nextState = ldi4;
		ldi4			:	nextState = fetch0;
		ldr0			:	nextState = ldr1;
		ldr1			:	nextState = ldr2;
		ldr2			:	nextState = fetch0;
		lea0			: 	nextState = fetch0;
		sti0			:	nextState = sti1;
		sti1			:	nextState = sti2;
		sti2			:	nextState = sti3;
		sti3			:	nextState = sti4;
		sti4			:	nextState = fetch0;
		str0			:	nextState = str1;
		str1			:	nextState = str2;
		str2			:	nextState = fetch0;
		trap0			:	nextState = trap1;
		trap1			:	nextState = fetch0;
		ioe				: 	nextState = fetch0;
		rti				: 	nextState = fetch0;
		//trap2			:	nextState = fetch0;
		default			:	nextState = fetch0; //$display("Error: Bad state: %h", currentState);
		
		
	endcase
	 		
							  
	
	assign signals = (currentState == fetch0)? 29'b00000000000000000000010010000:// ldMAR enaPC
						  (currentState == fetch1)? 29'b00000000000000000000001001100:// selMDR ldMDR ldPC
						  (currentState == fetch2)? 29'b00000000000000000000000100001:// enaMDR ldIR
						  (currentState == decode)? 29'b00000000000000000000000000000:
						  (currentState == ioe)? 29'b00000000000000000000000000000:
						  (currentState == rti)? 29'b00000000000000000000000000000:
						  (currentState == not_and_add0)? {(IR[15:14] + 2'b01), IR[8:6], IR[2:0], IR[11:9], 18'b000001110000000000}:// flagWE regWE enaALU
						  (currentState == jsr0)? 29'b00000000111010110100011000000:// ldPC enaPC regWE selEAB2=11 selPC=01 DR=7
						  (currentState == jsrr0)? {2'b00, IR[8:6], 24'b000111011000100011000000}: // ldPC enaPC regWE selEAB1 selPC=01 DR=7
						  (currentState == br0)? {22'b0000000000001010000000, TB, 6'b000000}: // selEAB2=10 selPC=01
						  (currentState == ld0)? 29'b00000000000000100001000010000:// ldMAR enaMARM selEAB2=10
						  (currentState == ld1)? 29'b00000000000000000000000001100:// selMDR ldMDR
						  (currentState == ld2)? {8'b00000000, IR[11:9], 18'b000000110000000001}: // enaMDR flagWE regWE
						  (currentState == st0)? 29'b00000000000000100001000010000:// ldMAR enaMARM selEAB2=10
						  (currentState == st1)? {2'b00, IR[11:9], 24'b000000000001000000001000}:// ldMDR enaALU
						  (currentState == st2)? 29'b00000000000000000000000000010: // memWE
						  (currentState == ldi0)? 29'b00000000000000100001000010000: // ldMAR enaMARM selEAB2=10
						  (currentState == ldi1)? 29'b00000000000000000000000001100: // selMDR ldMDR
						  (currentState == ldi2)? 29'b00000000000000000000000010001:// enaMDR ldMAR
						  (currentState == ldi3)? 29'b00000000000000000000000001100: // selMDR ldMDR
						  (currentState == ldi4)? {8'b00000000, IR[11:9], 18'b000000110000000001}: // enaMDR flagWE regWE
						  (currentState == ldr0)? {2'b00, IR[8:6], 24'b000000001010001000010000}://ldMAR enaMARM selEAB2=01 selEAB1
						  (currentState == ldr1)? 29'b00000000000000000000000001100:// selMDR ldMDR 
						  (currentState == ldr2)? {8'b00000000, IR[11:9], 18'b000000110000000001}://enaMDR flagWE regWE
						  (currentState == lea0)? {8'b00000000, IR[11:9], 18'b000100111000000000}:// enaMARM flagWE regWE selEAB2=10
						  (currentState == sti0)? 29'b00000000000000100001000010000:// ldMAR enaMARM selEAB2=10
						  (currentState == sti1)? 29'b00000000000000000000000001100:// selMDR ldMDR
						  (currentState == sti2)? 29'b00000000000000000000000010001:// enaMDR ldMAR
						  (currentState == sti3)? {2'b00, IR[11:9], 24'b000000000001000000001000}:// ldMDR enaALU
						  (currentState == sti4)? 29'b00000000000000000000000000010:// memWE
						  (currentState == str0)? {2'b00, IR[8:6], 24'b000000001010001000010000}:// ldMAR enaMARM selEAB2=01 selEAB1
						  (currentState == str1)? {2'b00, IR[11:9], 24'b000000000001000000001000}:// ldMDR enaALU
						  (currentState == str2)? 29'b00000000000000000000000000010:// memWE
						  (currentState == trap0)? 29'b00000000111000000100010000000:// enaPC regWE DR=7
						  (currentState == trap1)? 29'b00000000000100000001101000000:// ldPC selMAR enaMARM selPC=10
						  {2'b00, IR[8:6], 24'b000000011000000001000000};// ldPC selEAB1 selPC=01 (JMP)
						  
	assign aluControl = signals[28:27];
	assign SR1 = signals[26:24];
	assign SR2 = signals[23:21];
	assign DR = signals[20:18];
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


endmodule


module Datapath(
    output logic[15:0] IR,
    output logic N,
    output logic Z,
    output logic P,
    input logic[1:0] aluControl,
    input logic[2:0] SR1,
    input logic[2:0] SR2,
    input logic[2:0] DR,
    input logic[1:0] selPC,
    input logic[1:0] selEAB2,
    input logic enaALU,
    input logic regWE,
    input logic enaMARM,
    input logic selMAR,
    input logic selEAB1,
    input logic enaPC,
    input logic ldPC,
    input logic ldIR,
    input logic ldMAR,
    input logic ldMDR,
    input logic selMDR,
    input logic memWE,
    input logic enaMDR,
    input logic flagWE,
    input logic clk,
    input logic reset,
    output logic [15:0] Buss_out,
    input logic [15:0] mdrOut,
    output logic[15:0] PCW,
    output logic[15:0] r0_out,
    output logic[15:0] r1_out,
    output logic[15:0] r2_out,
    output logic[15:0] r3_out,
    output logic[15:0] r4_out,
    output logic[15:0] r5_out,
    output logic[15:0] r6_out,
    output logic[15:0] r7_out
    );

	 
	 
	 wire[15:0] Buss;
	 assign Buss_out = Buss;
	 logic[15:0] MARMuxOut, Ra, Rb, aluOut, eabOut;
 	 
	 
	 
	 //MARMux Logic
	 wire[15:0] ZExt;
	 assign MARMuxOut = selMAR?ZExt:eabOut;
	 assign ZExt = { {8{1'b0}} , IR[7:0]};
	 
	//IR Logic
	register IRBox(IR, clk, Buss, reset, ldIR);

	//ALU Logic
	wire[15:0] a, b;
	assign a = { {11{IR[4]}} ,IR[4:0]};
	assign b = IR[5]?a:Rb;
	assign aluOut = (aluControl == 2'b00) ? Ra:
				    (aluControl == 2'b01) ? b + Ra:
					(aluControl == 2'b10) ? b & Ra:
						                       ~Ra;
						                       
	 //EAB Logic
	 wire[15:0] addr2mux_out, addr1mux_out, a1, b1, c1;
	 
	 assign addr2mux_out = (selEAB2 == 2'b00) ? 16'd0:
							   (selEAB2 == 2'b01) ? c1: 
							   (selEAB2 == 2'b10) ? b1: 
										   		    a1;
															 
	assign addr1mux_out = selEAB1?Ra:PCW;
		
	assign eabOut = addr2mux_out + addr1mux_out;
		
	assign c1 = { {10{IR[5]}} , IR[5:0]};
	assign b1 = { {7{IR[8]}} ,IR[8:0]};
	assign a1 = { {5{IR[10]}} ,IR[10:0]};

	//NZP logic
	wire[2:0] IFL_out;
	register_1bit ZBox(Z, clk, IFL_out[1], reset, flagWE);
	register_1bit NBox(N, clk, IFL_out[2], reset, flagWE);
	register_1bit PBox(P, clk, IFL_out[0], reset, flagWE);
	assign IFL_out = (Buss == 16'd0) ? 3'b010:
				  (Buss[15] == 1'b0) ? 3'b001:
									   3'b100;					

	//PC logic
	wire[15:0] PCMUX_out;
	 
	assign PCMUX_out = (selPC == 2'b00) ? (PCW + 16'd1):
		    				  (selPC == 2'b01) ? eabOut: 
						  						   Buss;
																													
	register pcBox(PCW, clk, PCMUX_out, reset, ldPC);
															

	 //Register File Logic
	 wire[7:0] WEline;
	 
	 register reg_1(r0_out, clk, Buss, reset, WEline[0]);
	 register reg_2(r1_out, clk, Buss, reset, WEline[1]);
	 register reg_3(r2_out, clk, Buss, reset, WEline[2]);
	 register reg_4(r3_out, clk, Buss, reset, WEline[3]);
	 register reg_5(r4_out, clk, Buss, reset, WEline[4]);
	 register reg_6(r5_out, clk, Buss, reset, WEline[5]);
	 register reg_7(r6_out, clk, Buss, reset, WEline[6]);
	 register reg_8(r7_out, clk, Buss, reset, WEline[7]);
	 
	 assign WEline = (regWE) ? 8'd1 << DR: 8'd0;
	 
	 
	 assign Ra = (SR1 == 3'b000) ? r0_out:
					 (SR1 == 3'b001) ? r1_out:
					 (SR1 == 3'b010) ? r2_out:
					 (SR1 == 3'b011) ? r3_out:
					 (SR1 == 3'b100) ? r4_out:
					 (SR1 == 3'b101) ? r5_out:
					 (SR1 == 3'b110) ? r6_out:
									   r7_out;
											 
	 assign Rb = (SR2 == 3'b000) ? r0_out:
				 (SR2 == 3'b001) ? r1_out:
				 (SR2 == 3'b010) ? r2_out:
				 (SR2 == 3'b011) ? r3_out:
				 (SR2 == 3'b100) ? r4_out:
				 (SR2 == 3'b101) ? r5_out:
				 (SR2 == 3'b110) ? r6_out:
								   r7_out;										 
	 
	 assign Buss = (enaPC)? PCW:(16'bZZZZZZZZZZZZZZZZ);
	 assign Buss = (enaMARM)? MARMuxOut:(16'bZZZZZZZZZZZZZZZZ);
	 assign Buss = (enaMDR)? mdrOut:(16'bZZZZZZZZZZZZZZZZ);
	 assign Buss = (enaALU)? aluOut:(16'bZZZZZZZZZZZZZZZZ); 
	 
 
endmodule


module MAR(Buss, clk, reset, ldMAR, MARReg);

	input [15:0] Buss;
	input clk, reset, ldMAR;
	output reg [15:0] MARReg;
	 
	  always @(posedge clk or posedge reset) 
		if (reset == 1'b1) 										   
				MARReg = 0; 
		else if (ldMAR)
				MARReg = Buss;								 					
endmodule				



module MDR(Buss, memOut, selMDR, clk, reset, ldMDR, MDRReg);

	input [15:0] Buss, memOut;
	input clk, reset, ldMDR, selMDR;
	output reg [15:0] MDRReg;

  always @(posedge clk or posedge reset) 
    if (reset == 1'b1) 										   
			MDRReg = 0; 
	else if (ldMDR)
			MDRReg = selMDR?memOut:Buss;		
endmodule


module register(dout, clk, din, reset, load);

	input clk, reset, load;
	input [15:0] din;
	output logic [15:0] dout;

	always @(posedge clk)
	 if (reset) dout <= 16'd0;
	else if (load) dout <= din;
endmodule


module register_1bit(dout, clk, din, reset, load);

	input clk, reset, load;
	input din;
	output logic dout;

	always @(posedge clk)
	 if (reset) dout <= 1'd0;
	else if (load) dout <= din;
endmodule

//n-bit register (parameterizable)
//module register_n(dout, clk, din, reset, load);
//
//	parameter WID=16;
//	input clk, reset, load;
//	input[WID-1:0]din;
//	output logic[WID-1:0] dout;
//
//	always @(posedge clk)
//		if(reset) dout <= 'b0;
//		else if (load) dout <= din;
//endmodule
