// File Name: lc3_pkg.sv
// This file contains the package declaration which contains classes used by the testbench

`define SV_RAND_CHECK(r) \
 do begin \
	if (!(r)) begin \
		$display("%s:%0d: Randomization failed \"%s\"", \
			`__FILE__, `__LINE__, `"r`"); \
		$finish; \
	end \
 end while (0)

package lc3_pkg;
	typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;

	//typedef enum bit [3:0] {NOP=0, ADD, SUB, AND, NOT, RD, WR, BR, BRZ, RDI} Instruction; //, HALT=4'hF 
	//typedef enum bit [3:0] {ADD, AND, NOT, BR, JMP, JSR, JSRR, RET, LD, LDI, LDR, LEA, ST, STI, STR, TRAP, RTI} Instruction;
	
	class Transaction #(ADDRESS_WIDTH=8);
	
		opcode_type opcode;
		//rand bit [2:0] src1, src2, dst;
		rand bit [15:0] instruction;
		
		function new (bit [15:0] i=0);
			instruction = i;
			opcode = opcode_type'({28'h0, instruction[31:28]});
		endfunction
		
		function void post_randomize();
			opcode = opcode_type'({28'h0, instruction[31:28]});
		endfunction
	
	endclass: Transaction
	
	
	class Generator #(ADDRESS_WIDTH=8);
	
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt;
		Transaction #(ADDRESS_WIDTH) t;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) g2a);
			gen2agt = g2a;
		endfunction
		
		task run;
			while(1) begin
				t = new();
				`SV_RAND_CHECK(t.randomize());
				gen2agt.put(t);
			end
		endtask
	
	endclass: Generator
	
	
	class Agent #(ADDRESS_WIDTH=8);
	
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt, agt2drv;
		Transaction #(ADDRESS_WIDTH) t;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) g2a, a2d);
			gen2agt = g2a;
			agt2drv = a2d;
		endfunction
		
		task run;
			while(1) begin
				gen2agt.get(t);
				agt2drv.put(t);
			end
		endtask
		
	endclass: Agent	
	
	virtual class Driver_cbs #(ADDRESS_WIDTH=8);
		virtual task pre_tx(ref Transaction #(ADDRESS_WIDTH) t);
		// By default, callback does nothing
		endtask
		virtual task post_tx(ref Transaction #(ADDRESS_WIDTH) t, integer count);
		// By default, callback does nothing
		endtask
	endclass
	
	class Driver_cbs_coverage #(ADDRESS_WIDTH=8) extends Driver_cbs #(ADDRESS_WIDTH);
		Transaction #(ADDRESS_WIDTH) sample_t;
		covergroup CovOp;
			option.per_instance = 1;
			// Test that all opcodes have been used
			opcode: coverpoint sample_t.opcode;
			
			source11_9: coverpoint sample_t.instruction[11:9]{
				option.weight = 0;
			}
			source8_6: coverpoint sample_t.instruction[8:6]{
				option.weight = 0;
			}
			source2_0: coverpoint sample_t.instruction[2:0]{
				option.weight = 0;
			}
			destination: coverpoint sample_t.instruction[11:9]{
				option.weight = 0;
			}
			
			//op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			cross opcode, source11_9{
				option.weight = 2;
				ignore_bins ignore_no_src11_9 = binsof(opcode) intersect {op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			cross opcode, source8_6{
				option.weight = 2;
				ignore_bins ignore_no_src8_6 = binsof(opcode) intersect {op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			cross opcode, source2_0{
				option.weight = 2;
				ignore_bins ignore_no_src2_0 = binsof(opcode) intersect {op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a dst register have been tested with all possible dst registers
			cross opcode, destination{
				option.weight = 2;
				ignore_bins ignore_no_dst = binsof(opcode) intersect {op_br, op_jmp, op_jsr, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			//TODO: make more cover groups
		endgroup
		
		virtual task pre_tx(ref Transaction #(ADDRESS_WIDTH) t);
			sample_t = t;
			CovOp.sample;
		endtask
		
		virtual task post_tx(ref Transaction #(ADDRESS_WIDTH) t, integer count);
			if($get_coverage() == 100)begin
				$display("%g\tTest Coverage reached 100 percent!  Exiting test after %d instructions...", $time, count);
				$finish;
			end
		endtask
		
		
		
		function new();
			CovOp = new();
		endfunction
	endclass: Driver_cbs_coverage
	
	
	class Driver #(ADDRESS_WIDTH=8);
		virtual lc3_if#(ADDRESS_WIDTH).TB ports;
		mailbox #(Transaction #(ADDRESS_WIDTH)) agt2drv;
		Transaction #(ADDRESS_WIDTH) t;
		Driver_cbs #(ADDRESS_WIDTH) cbs[$];
		integer count;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) a2d, virtual lc3_if#(ADDRESS_WIDTH).TB p);
			agt2drv = a2d;
			ports = p;
			count = 0;
		endfunction
		
		task run;
			// Reset the DUT
			ports.reset <= 0;
			repeat (3) @ports.cb;
			ports.reset <= 1;
			ports.cb.memOut <= 0;
			repeat (2) @ports.cb;
			
			// Begin driving instructions
			while(1) begin
				count++;
				agt2drv.get(t);
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				ports.cb.memOut <= t.instruction;
				@ports.cb;
				if(t.opcode == op_add) begin
					//TODO: finish this and other opcodes
				end else begin
					$display("%g\tInstruction %d Received!", $time, t.opcode);
				end
				foreach(cbs[i]) begin
					cbs[i].post_tx(t, count);
				end
			end
		endtask
		
	endclass: Driver
	

	class Environment #(ADDRESS_WIDTH=8);
		virtual lc3_if#(ADDRESS_WIDTH).TB ports;
		Generator #(ADDRESS_WIDTH) gen;
		Agent #(ADDRESS_WIDTH) agt;
		Driver #(ADDRESS_WIDTH) drv;
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt, agt2drv;
		
		function new(virtual lc3_if#(ADDRESS_WIDTH).TB p);
			ports = p;
		endfunction
		
		function void build();
			//Initialize mailboxes
			gen2agt = new(1);
			agt2drv = new(1);
			
			//Initialize transactors
			gen = new(gen2agt);
			agt = new(gen2agt, agt2drv);
			drv = new(agt2drv, ports);
		endfunction
		
		task run();
			fork
				gen.run();
				agt.run();
				drv.run();
			join_any
		endtask
		
	endclass: Environment
	
endpackage