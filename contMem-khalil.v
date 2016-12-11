`timescale 1ns / 1ps

module contMem(
	output [24:0] signals,
	input 
	);

	reg [24:0] my_memory [0:255];		

	initial
	begin
		$readmemb("lc3_instructions.v", my_memory);
	end
	assign signals = my_memory[MARReg];

endmodule
