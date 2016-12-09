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
	
	class Transaction #(ADDRESS_WIDTH=8);
	
		opcode_type opcode;
		rand bit [2:0] src1, src2, dst;
		rand bit [15:0] instruction;
		
		function new (bit [15:0] i=0);
			instruction = i;
			opcode = opcode_type'({28'h0, instruction[31:28]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
		endfunction
		
		function void post_randomize();
			opcode = opcode_type'({28'h0, instruction[31:28]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
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
	
	virtual class Driver_cbs_expected #(ADDRESS_WIDTH=8);
		virtual task post_tx(ref Transaction #(ADDRESS_WIDTH) t, integer count);
			//TODO: somehow we package expected behavior here and send it to the Scoreboard using the save_expected function
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
	
	
	class State;
		logic [15:0] regs [8];
		logic [15:0] dst_val;
		logic [2:0] dst_reg;
		logic [15:0] pc;
		logic [15:0] mem_addr;
		logic [15:0] mem_val;
		
		function new();
/* 			for(i = 0; i < 8; i++) begin
				regs[i] = 
			end */
		endfunction
	endclass: State
	
	
	class Driver #(ADDRESS_WIDTH=8);
		virtual lc3_if#(ADDRESS_WIDTH).TB ports;
		mailbox #(Transaction #(ADDRESS_WIDTH)) agt2drv;
		Transaction #(ADDRESS_WIDTH) t;
		Driver_cbs #(ADDRESS_WIDTH) cbs[$];
		integer count;
		State s;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) a2d, virtual lc3_if#(ADDRESS_WIDTH).TB p);
			agt2drv = a2d;
			ports = p;
			count = 0;
		endfunction
		
		task run;
			// Reset the DUT
			ports.reset <= 1;
			repeat (3) @ports.cb;
			ports.reset <= 0;
			ports.cb.memOut <= 0;
			@ports.cb;
			
			// Begin driving instructions
			while(1) begin
				s = new();
				count++;
				agt2drv.get(t);
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				repeat(3) @ports.cb;
				ports.cb.memOut <= t.instruction;
				@ports.cb;
				s.pc = $root.lc3_top.my_lc3.my_dp.pc;
				s.regs[0] = $root.lc3_top.my_lc3.my_dp.r0;
				s.regs[1] = $root.lc3_top.my_lc3.my_dp.r1;
				s.regs[2] = $root.lc3_top.my_lc3.my_dp.r2;
				s.regs[3] = $root.lc3_top.my_lc3.my_dp.r3;
				s.regs[4] = $root.lc3_top.my_lc3.my_dp.r4;
				s.regs[5] = $root.lc3_top.my_lc3.my_dp.r5;
				s.regs[6] = $root.lc3_top.my_lc3.my_dp.r6;
				s.regs[7] = $root.lc3_top.my_lc3.my_dp.r7;
				if((t.opcode == op_ldi) || (t.opcode == op_sti)) begin // 8 clk cycles
					repeat(4) @ports.cb;
				end else if((t.opcode == op_ld) || (t.opcode == op_st) || (t.opcode == op_str) || (t.opcode == op_str)) begin // 6 clk cycles
					repeat(2) @ports.cb;
				end else if((t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_not) || (t.opcode == op_br) || (t.opcode == op_jmp) 
							|| (t.opcode == op_jsr) || (t.opcode == op_trap) || (t.opcode == op_rti) || (t.opcode == op_ioe)) begin // 4 clk cycles
					if(t.opcode == op_add) begin
						s.regs[t.dst] = s.regs[t.src1] + s.regs[t.src2];
						s.dst_val = s.regs[t.dst];
						s.dst_reg = t.dst;
					end
				end else begin
					$display("%g\tUnknown Instruction %d Received!", $time, t.opcode);
				end
				foreach(cbs[i]) begin
					cbs[i].post_tx(t, count);
				end
			end
		endtask
		
	endclass: Driver
	
	
	class Config;
		integer errors;
		function new();
			errors = 0;
		endfunction
	endclass: Config

	
	class Monitor #(ADDRESS_WIDTH=16);
		virtual lc3_if#(ADDRESS_WIDTH).MON ports;
		//TODO: Monitor callbacks?
		
		function new(input virtual lc3_if#(ADDRESS_WIDTH).MON p);
			ports = p;
		endfunction
		
		task run();
			Transaction t;
			forever begin
				
				//TODO: Check the DUT's correct functionality
			end
		endtask
		
	endclass: Monitor
	
	
	class Scoreboard;
		Config cfg;
		integer count;
		State expected[$];
		
		function new(Config c);
			cfg = c;
		endfunction
		
		function void save_expected(input State s);
			expected.push_back(s);
			//TODO: Save expected behavior somehow
		endfunction
		
		function void check_actual();
			//TODO: Verify expected behavior
			//TODO: use cfg to track number of errors
		endfunction
	endclass: Scoreboard
	
	
	class Environment #(ADDRESS_WIDTH=8);
		virtual lc3_if#(ADDRESS_WIDTH).TB tb_ports;
		virtual lc3_if#(ADDRESS_WIDTH).MON mon_ports;
		Generator #(ADDRESS_WIDTH) gen;
		Agent #(ADDRESS_WIDTH) agt;
		Driver #(ADDRESS_WIDTH) drv;
		Monitor #(ADDRESS_WIDTH) mon;
		Scoreboard sb;
		Config cfg;
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt, agt2drv;
		
		function new(virtual lc3_if#(ADDRESS_WIDTH).TB tp, virtual lc3_if#(ADDRESS_WIDTH).MON mp);
			tb_ports = tp;
			mon_ports = mp;
		endfunction
		
		function void build();
			//Initialize mailboxes
			gen2agt = new(1);
			agt2drv = new(1);
			
			//Initialize transactors
			gen = new(gen2agt);
			agt = new(gen2agt, agt2drv);
			drv = new(agt2drv, tb_ports);
			mon = new(mon_ports);
			cfg = new();
			sb = new(cfg);
		endfunction
		
		task run();
			fork
				gen.run();
				agt.run();
				drv.run();
				mon.run();
			join_any
		endtask
		
	endclass: Environment
	
endpackage