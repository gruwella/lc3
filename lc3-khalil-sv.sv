typedef enum bit[36:0] {
	  
	  fetch0  	=	 	37'b0000000000000000000000000000000000001,
	  fetch1	= 		37'b0000000000000000000000000000000000010, 
	  fetch2	= 		37'b0000000000000000000000000000000000100,
	  decode	= 		37'b0000000000000000000000000000000001000, 
	  not_and_add0	= 	37'b0000000000000000000000000000000010000,
	  jsr0	= 			37'b0000000000000000000000000000000100000,
	  jsr1	= 			37'b0000000000000000000000000000001000000,
	  br0	= 			37'b0000000000000000000000000000010000000,
	  ld0	= 			37'b0000000000000000000000000000100000000,
	  ld1	= 			37'b0000000000000000000000000001000000000,
	  ld2	= 			37'b0000000000000000000000000010000000000,
	  st0	= 			37'b0000000000000000000000000100000000000,
	  st1	= 			37'b0000000000000000000000001000000000000,
	  st2	= 			37'b0000000000000000000000010000000000000,
	  jmp0	= 			37'b0000000000000000000000100000000000000,
	  ldi0	= 			37'b0000000000000000000001000000000000000,
	  ldi1 	= 			37'b0000000000000000000010000000000000000,
	  ldi2	= 			37'b0000000000000000000100000000000000000,
	  ldi3	= 			37'b0000000000000000001000000000000000000,
	  ldi4	= 			37'b0000000000000000010000000000000000000,
	  ldr0	= 			37'b0000000000000000100000000000000000000,
	  ldr1	= 			37'b0000000000000001000000000000000000000,
	  ldr2	= 			37'b0000000000000010000000000000000000000,
	  lea0	= 			37'b0000000000000100000000000000000000000,
	  sti0	= 			37'b0000000000001000000000000000000000000,
	  sti1	= 			37'b0000000000010000000000000000000000000,
	  sti2	= 			37'b0000000000100000000000000000000000000,
	  sti3	= 			37'b0000000001000000000000000000000000000,
	  sti4	= 			37'b0000000010000000000000000000000000000,
	  str0  =			37'b0000000100000000000000000000000000000,
	  str1	= 			37'b0000001000000000000000000000000000000,
	  str2	= 			37'b0000010000000000000000000000000000000,
	  trap0 = 			37'b0000100000000000000000000000000000000,
	  trap1 = 			37'b0001000000000000000000000000000000000,
	  trap2 =			37'b0010000000000000000000000000000000000,
	  jsrr0 = 			37'b0100000000000000000000000000000000000,
	  jsrr1 =			37'b1000000000000000000000000000000000000
	  
    } stateTypedef;


module khalil_LC3(clk, reset,memwe, mdr, mar, memOut, pc, n_flag, z_flag, p_flag, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out);

    output logic memwe, mdr, mar, memOut, pc, n_flag, z_flag, p_flag, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out;

    input logic clk; 
    input logic reset;

	  logic[15:0] IR;
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
	  logic[15:0] mdrOut;



	Datapath Datapath(IR, N, Z, P, aluControl, SR1, SR2, DR, selPC, selEAB2, enaALU, regWE, enaMARM, selMAR, selEAB1, enaPC, ldPC, ldIR, ldMAR, ldMDR, selMDR, memWE, enaMDR, flagWE, clk, reset, Buss, mdrOut,PCW, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out);

	ControlUnit Control(N, Z, P, IR, aluControl, SR1, SR2, DR, selPC, selEAB2, enaALU, regWE, enaMARM, selMAR, selEAB1, enaPC, ldPC, ldIR, ldMAR, ldMDR, selMDR, memWE, enaMDR, flagWE, clk, reset);

	MAR MAR0(Buss, clk, reset, ldMAR, MARReg);	  
	MDR MDR0(Buss, memOut, selMDR, clk, reset, ldMDR, mdrOut);

endmodule



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
						(opCode == 4'b1000)? jmp0:	//If opcode is rti, go to jmp0 (ret) state.				
						 fetch0;						
  
  
  always_ff @(posedge clk)
    begin
		if (reset)
			currentState <= fetch0;
		else
			currentState <= nextState;
	 end
		
	 
  always_comb
unique case(currentState)
		fetch0			:	nextState = fetch1;
		fetch1			:	nextState = fetch2;
		fetch2			:	nextState = decode;
		decode			:	nextState = decoded_nextState;
		not_and_add0	:	nextState = fetch0;
		jsr0			:	nextState = jsr1;
		jsr1			:	nextState = fetch0;
		jsrr0			: 	nextState = jsrr1;
		jsrr1			:	nextState = fetch0;
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
		trap1			:	nextState = trap2;
		trap2			:	nextState = fetch0;
		
		
	endcase
	 		
							  
	
	assign signals = (currentState == fetch0)? 29'b00000000000000000000010010000:
						  (currentState == fetch1)? 29'b00000000000000000000001001100:
						  (currentState == fetch2)? 29'b00000000000000000000000100001:
						  (currentState == decode)? 29'b00000000000000000000000000000:
						  (currentState == not_and_add0)? {(IR[15:14] + 1'b1), IR[8:6], IR[2:0], IR[11:9], 18'b000001110000000000}:
						  (currentState == jsr0)? 29'b00000000111000000100010000000:
						  (currentState == jsr1)? 29'b00000000000010110000001000000:
						  (currentState == jsrr0)? 29'b00000000111000000100010000000:
						  (currentState == jsrr1)? {2'b00, IR[8:6], 24'b000000011000000001000000}:
						  (currentState == br0)? {22'b0000000000001010000000, TB, 6'b000000}:
						  (currentState == ld0)? 29'b00000000000000100001000010000:
						  (currentState == ld1)? 29'b00000000000000000000000001100:
						  (currentState == ld2)? {8'b00000000, IR[11:9], 18'b000000110000000001}:
						  (currentState == st0)? 29'b00000000000000100001000010000:
						  (currentState == st1)? {2'b00, IR[11:9], 24'b000000000001000000001000}:
						  (currentState == st2)? 29'b00000000000000000000000000010: 
						  (currentState == ldi0)? 29'b00000000000000100001000010000:
						  (currentState == ldi1)? 29'b00000000000000000000000001100:
						  (currentState == ldi2)? 29'b00000000000000000000000010001:
						  (currentState == ldi3)? 29'b00000000000000000000000001100:
						  (currentState == ldi4)? {8'b00000000, IR[11:9], 18'b000000110000000001}:
						  (currentState == ldr0)? {2'b00, IR[8:6], 24'b000000001010001000010000}:
						  (currentState == ldr1)? 29'b00000000000000000000000001100:
						  (currentState == ldr2)? {8'b00000000, IR[11:9], 18'b000000110000000001}:
						  (currentState == lea0)? {8'b00000000, IR[11:9], 18'b000100111000000000}:
						  (currentState == sti0)? 29'b00000000000000100001000010000:
						  (currentState == sti1)? 29'b00000000000000000000000001100:
						  (currentState == sti2)? 29'b00000000000000000000000010001:
						  (currentState == sti3)? {2'b00, IR[11:9], 24'b000000000001000000001000}:
						  (currentState == sti4)? 29'b00000000000000000000000000010:
						  (currentState == str0)? {2'b00, IR[8:6], 24'b000000001010001000010000}:
						  (currentState == str1)? {2'b00, IR[11:9], 24'b000000000001000000001000}:
						  (currentState == str2)? 29'b00000000000000000000000000010:
						  (currentState == trap0)? 29'b00000000000000000001100010000:
						  (currentState == trap1)? 29'b00000000111000000100010001100:
						  (currentState == trap2)? 29'b00000000000100000000001000001:
						  {2'b00, IR[8:6], 24'b000000011000000001000000};
						  
	 
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


module MAR(Buss, clk, reset, ldMAR, MAR);

	input [15:0] Buss;
	input clk, reset, ldMAR;
	output reg [15:0] MAR;
	 
	  always @(posedge clk or posedge reset) 
		if (reset == 1'b1) 										   
				MAR = 0; 
		else if (ldMAR)
				MAR = Buss;								 					
endmodule				



module MDR(Buss, memOut, selMDR, clk, reset, ldMDR, MDR);

	input [15:0] Buss, memOut;
	input clk, reset, ldMDR, selMDR;
	output reg [15:0] MDR;

  always @(posedge clk or posedge reset) 
    if (reset == 1'b1) 										   
			MDR = 0; 
	else if (ldMDR)
			MDR = selMDR?memOut:Buss;		
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