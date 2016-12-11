`timescale 1ns / 1ps

module LC3_Full(
	input reset, 
	input clk
	);
	wire [1:0] aluControl, selPC, selEAB2;
	wire [2:0] SR1, SR2, DR;
	wire selEAB1, enaALU, regWE, flagWE, enaMARM, selMAR, enaPC, ldPC, ldIR, ldMAR, ldMDR, selMDR, memWE, enaMDR, N, Z, P;
	wire [15:0] IR;

	decodeFSM controller(		.aluControl(aluControl), 
								.SR1(SR1), 
								.SR2(SR2), 
								.DR(DR), 
								.selPC(selPC), 
								.selEAB1(selEAB1), 
								.selEAB2(selEAB2), 
								.enaALU(enaALU), 
								.regWE(regWE), 	
								.flagWE(flagWE),
							    .enaMARM(enaMARM), 
								.selMAR(selMAR), 
								.enaPC(enaPC), 
								.ldPC(ldPC), 
								.ldIR(ldIR), 
								.ldMAR(ldMAR), 	
								.ldMDR(ldMDR), 
								.selMDR(selMDR), 
								.memWE(memWE), 
								.enaMDR(enaMDR), 
								.IR(IR), 
								.N(N), 
								.Z(Z), 
								.P(P), 	
								.clk(clk), 
								.reset(reset));
		
	LC3_Datapath LC3(
								.aluControl(aluControl), 
								.SR1(SR1), 
								.SR2(SR2), 
								.DR(DR), 
								.selPC(selPC), 
								.selEAB1(selEAB1), 
								.selEAB2(selEAB2), 
								.enaALU(enaALU), 
								.regWE(regWE), 	
								.flagWE(flagWE),
							    .enaMARM(enaMARM), 
								.selMAR(selMAR), 
								.enaPC(enaPC), 
								.ldPC(ldPC), 
								.ldIR(ldIR), 
								.ldMAR(ldMAR), 	
								.ldMDR(ldMDR), 
								.selMDR(selMDR), 
								.memWE(memWE), 
								.enaMDR(enaMDR), 
								.IR(IR), 
								.N(N), 
								.Z(Z), 
								.P(P), 	
								.clk(clk), 
								.reset(reset));
		
endmodule
