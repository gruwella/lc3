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

	integer num_instructions = 1000;

	//typedef enum bit [3:0] {NOP=0, ADD, SUB, AND, NOT, RD, WR, BR, BRZ, RDI} Instruction; //, HALT=4'hF 
	typedef enum bit [3:0] {ADD, AND, NOT, BR, JMP, JSR, JSRR, RET, LD, LDI, LDR, LEA, ST, STI, STR, TRAP, RTI} Instruction;
	
	class Transaction #(ADDRESS_WIDTH=8);
	
		rand Instruction opcode;
		rand bit [1:0] src, dst;
		rand bit [ADDRESS_WIDTH-1:0] addr;
		
		function new (Instruction op=NOP, bit [1:0] s=0, bit [1:0] d=0, bit [ADDRESS_WIDTH-1:0] a=0);
			opcode = op;
			src = s;
			dst = d;
			addr = a;
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
			opcode: coverpoint sample_t.opcode{
				ignore_bins ignore = {BR, BRZ};
			}
			
			source: coverpoint sample_t.src{
				option.weight = 0;
			}
			destination: coverpoint sample_t.dst{
				option.weight = 0;
			}
			address: coverpoint sample_t.addr{
				bins all_memory[] = {[0:$]};
				option.weight = 0;
			}
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			cross opcode, source{
				option.weight = 2;
				ignore_bins ignore_no_src = binsof(opcode) intersect {NOP, RD, RDI, BR, BRZ};
			}
			
			// Test that all opcodes with a dst register have been tested with all possible dst registers
			cross opcode, destination{
				option.weight = 2;
				ignore_bins ignore_no_dst = binsof(opcode) intersect {NOP, WR, BR, BRZ};
			}
			
			// Test that all opcodes with a src and dst register have been tested with all permutations of src and dst
			cross opcode, source, destination{
				option.weight = 2;
				ignore_bins ignore_no_src_dest = binsof(opcode) intersect {NOP, RD, WR, BR, BRZ, RDI};
			}
			
			cross opcode, address{
				option.weight = 2;
				ignore_bins ignore_non_wr = binsof(opcode) intersect {NOP, ADD, SUB, AND, NOT, BR, BRZ, RDI};
			}
			
			// Test that all opcodes have been run before and after all other opcodes
			coverpoint sample_t.opcode {
				bins nop_add = (NOP=>ADD);
				bins nop_sub = (NOP=>SUB);
				bins nop_and = (NOP=>AND);
				bins nop_not = (NOP=>NOT);
				bins nop_rd = (NOP=>RD);
				bins nop_wr = (NOP=>WR);
				bins nop_rdi = (NOP=>RDI);
				
				bins add_nop = (ADD=>NOP);
				bins add_sub = (ADD=>SUB);
				bins add_and = (ADD=>AND);
				bins add_not = (ADD=>NOT);
				bins add_rd = (ADD=>RD);
				bins add_wr = (ADD=>WR);
				bins add_rdi = (ADD=>RDI);
				
				bins sub_nop = (SUB=>NOP);
				bins sub_add = (SUB=>ADD);
				bins sub_and = (SUB=>AND);
				bins sub_not = (SUB=>NOT);
				bins sub_rd = (SUB=>RD);
				bins sub_wr = (SUB=>WR);
				bins sub_rdi = (SUB=>RDI);
				
				bins and_nop = (AND=>NOP);
				bins and_add = (AND=>ADD);
				bins and_sub = (AND=>SUB);
				bins and_not = (AND=>NOT);
				bins and_rd = (AND=>RD);
				bins and_wr = (AND=>WR);
				bins and_rdi = (AND=>RDI);
				
				bins not_nop = (NOT=>NOP);
				bins not_add = (NOT=>ADD);
				bins not_sub = (NOT=>SUB);
				bins not_and = (NOT=>AND);
				bins not_rd = (NOT=>RD);
				bins not_wr = (NOT=>WR);
				bins not_rdi = (NOT=>RDI);
				
				bins rd_nop = (RD=>NOP);
				bins rd_add = (RD=>ADD);
				bins rd_sub = (RD=>SUB);
				bins rd_and = (RD=>AND);
				bins rd_not = (RD=>NOT);
				bins rd_wr = (RD=>WR);
				bins rd_rdi = (RD=>RDI);
				
				bins wr_nop = (WR=>NOP);
				bins wr_add = (WR=>ADD);
				bins wr_sub = (WR=>SUB);
				bins wr_and = (WR=>AND);
				bins wr_not = (WR=>NOT);
				bins wr_rd = (WR=>RD);
				bins wr_rdi = (WR=>RDI);
				
				bins rdi_nop = (RDI=>NOP);
				bins rdi_add = (RDI=>ADD);
				bins rdi_sub = (RDI=>SUB);
				bins rdi_and = (RDI=>AND);
				bins rdi_not = (RDI=>NOT);
				bins rdi_rd = (RDI=>RD);
				bins rdi_wr = (RDI=>WR);
			}
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
		virtual risc_if#(ADDRESS_WIDTH).TB ports;
		mailbox #(Transaction #(ADDRESS_WIDTH)) agt2drv;
		Transaction #(ADDRESS_WIDTH) t;
		bit [7:0] inst;
		Driver_cbs #(ADDRESS_WIDTH) cbs[$];
		integer count;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) a2d, virtual risc_if#(ADDRESS_WIDTH).TB p);
			agt2drv = a2d;
			ports = p;
			count = 0;
		endfunction
		
		task run;
			// Reset the DUT
			ports.rst <= 0;
			repeat (3) @ports.cb;
			ports.rst <= 1;
			ports.cb.data_out <= 0;
			repeat (2) @ports.cb;
			
			// Begin driving instructions
			while(1) begin
				count++;
				agt2drv.get(t);
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				inst = {t.opcode, t.src, t.dst};
				ports.cb.data_out <= inst;
				@ports.cb;
				if(t.opcode == NOP || t.opcode == NOT || (t.opcode == BRZ && $root.risc_top.my_risc.Zflag == 0)) begin
					// Return to Fetch 1
					repeat (2) @ports.cb;
				end else if(t.opcode == ADD || t.opcode == SUB || t.opcode == AND) begin
					repeat (3) @ports.cb;
				end else if(t.opcode == RD || t.opcode == RDI) begin
					if(t.opcode == RD) begin
						// Move to Read 2
						@ports.cb;
						// Drive fake data
						ports.cb.data_out <= t.addr;
					end
					// Move to Read 1
					@ports.cb;
					// Drive Address
					ports.cb.data_out <= 8'hFF;
					// Move to Fetch 1
					repeat (2) @ports.cb;
				end else if(t.opcode == WR) begin
					// Move to Write 1
					@ports.cb;
					// Drive Address
					ports.cb.data_out <= t.addr;
					// Move to Write 2
					@ports.cb;
					// Move to Fetch 1
					repeat (2) @ports.cb;
				end else if(t.opcode == BR || (t.opcode == BRZ && $root.risc_top.my_risc.Zflag == 1)) begin
					// Move to Branch 1
					@ports.cb;
					// Drive Address
					ports.cb.data_out <= t.addr;
					// Move to Branch 2
					@ports.cb;
					// Drive fake data
					ports.cb.data_out <= 8'hFF;
					// Move to Fetch 1
					repeat (2) @ports.cb;
				end else begin
					$display("%g\tHalt Instruction Received!  Quitting...", $time);
					return;
				end
				foreach(cbs[i]) begin
					cbs[i].post_tx(t, count);
				end
			end
		endtask
		
	endclass: Driver
	

	class Environment #(ADDRESS_WIDTH=8);
		virtual risc_if#(ADDRESS_WIDTH).TB ports;
		Generator #(ADDRESS_WIDTH) gen;
		Agent #(ADDRESS_WIDTH) agt;
		Driver #(ADDRESS_WIDTH) drv;
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt, agt2drv;
		
		function new(virtual risc_if#(ADDRESS_WIDTH).TB p);
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