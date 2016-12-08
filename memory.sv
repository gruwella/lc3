module memory(memOut, clk, reset, memWE, mdrOut, MARReg);
	input logic clk, reset, memWE;
	input logic [15:0] mdrOut, MARReg;
	output logic [15:0] memOut;
  
	//MAR MAR0(Buss, clk, reset, ldMAR, MARReg);	  
	//MDR MDR0(Buss, memOut, selMDR, clk, reset, ldMDR, mdrOut);

	logic [15:0] my_memory [0:255];
	assign memOut = my_memory[MARReg]; 	   
	always @(posedge clk) 
	begin		  	 
		if (memWE)
			my_memory[MARReg] <= mdrOut;
	end
endmodule