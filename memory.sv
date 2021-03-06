module memory(memOut, clk, reset, memWE, mdrOut, MARReg);
	input logic clk, reset, memWE;
	input logic [15:0] mdrOut, MARReg;
	output logic [15:0] memOut;
  
	//MAR MAR0(Buss, clk, reset, ldMAR, MARReg);	  
	//MDR MDR0(Buss, memOut, selMDR, clk, reset, ldMDR, mdrOut);

	logic [15:0] my_memory [0:65535];
	assign memOut = my_memory[MARReg]; 	   
	always @(posedge clk) 
	begin		  	 
		if (memWE && !reset)
			my_memory[MARReg] <= mdrOut;
			
	end
	
	initial begin
		for(int i = 0; i < 65535; i++) begin
			my_memory[i] = $urandom_range(0, 65535);
		end
	end
endmodule