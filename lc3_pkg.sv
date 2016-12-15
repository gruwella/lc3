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
	typedef class Config;

		//LC3 Opcodes
	typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;
	
	// LC3 States
	typedef enum {start, fetch0, fetch1, fetch2, decode, ex_ld1, ex_ld2, ex_st1, ex_st2, ex_ldi1, ex_ldi2, ex_ldi3, ex_ldi4, ex_sti1, ex_sti2, ex_sti3, ex_sti4, ex_str1, ex_str2, ex_ldr1, ex_ldr2} state_type;
	
	class Transaction;
	
		opcode_type opcode;
		bit [2:0] src1, src2, dst;
		bit [15:0] imm5_val;
		bit imm5_bit;
		bit [15:0] pc_offset9;
		bit [15:0] pc_offset11;
		bit jsr_bit;
		bit n_flag, z_flag, p_flag;
		bit [15:0] trapvec8;
		bit [15:0] offset6;
		rand bit [15:0] instruction;
		rand bit rst; //TODO add constraint
		rand integer rst_cycle;
		
		constraint rst_d { rst dist {0:=90, 1:=10};} //TODO add reset back in
		constraint rst_c { rst_cycle >= 0; rst_cycle <= 8;}
		
		function new (bit [15:0] i=0);
			instruction = i;
/* 			case(instruction[31:28])
				4'h1 : opcode = op_add;
				4'h5 : opcode = op_and;
				4'h9 : opcode = op_not;
				4'hC : opcode = op_jmp;
				4'h4 : opcode = op_jsr;
				4'h2 : opcode = op_ld;
				4'h6 : opcode = op_ldr;
				4'hE : opcode = op_lea;
				4'hA : opcode = op_ldi;
				4'h3 : opcode = op_st;
				4'h7 : opcode = op_str;
				4'hB : opcode = op_sti;
				4'h8 : opcode = op_rti;
				4'hD : opcode = op_ioe;
				4'hF : opcode = op_trap;
				default : op_br;
			endcase */
			opcode = opcode_type'({28'h0, instruction[15:12]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = { {11{instruction[4]}}, instruction[4:0]};//{ {11{instruction[4]}}, instruction[4:0]}
			imm5_bit = instruction[5];
			pc_offset9 = { {7{instruction[8]}}, instruction[8:0]};
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = { {5{instruction[10]}}, instruction[10:0]};
			trapvec8 = { {8{0}}, instruction[7:0]};
			offset6 = { {10{instruction[5]}}, instruction[5:0]};
			//$display("T.opcode in new: 0x%h, instruction: 0x%h", opcode, instruction);
		endfunction
		
		
		
		function void post_randomize();
/* 			case(instruction[31:28])
				4'h1 : opcode = op_add;
				4'h5 : opcode = op_and;
				4'h9 : opcode = op_not;
				4'hC : opcode = op_jmp;
				4'h4 : opcode = op_jsr;
				4'h2 : opcode = op_ld;
				4'h6 : opcode = op_ldr;
				4'hE : opcode = op_lea;
				4'hA : opcode = op_ldi;
				4'h3 : opcode = op_st;
				4'h7 : opcode = op_str;
				4'hB : opcode = op_sti;
				4'h8 : opcode = op_rti;
				4'hD : opcode = op_ioe;
				4'hF : opcode = op_trap;
				default : op_br;
			endcase */
			opcode = opcode_type'({28'h0, instruction[15:12]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = { {11{instruction[4]}}, instruction[4:0]};//{ {11{instruction[4]}}, instruction[4:0]}
			imm5_bit = instruction[5];
			pc_offset9 = { {7{instruction[8]}}, instruction[8:0]};
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = { {5{instruction[10]}}, instruction[10:0]};
			trapvec8 = { {8{0}}, instruction[7:0]};
			offset6 = { {10{instruction[5]}}, instruction[5:0]};
			
			//$display("T.opcode in post_randomize: 0x%h, instruction: 0x%h", opcode, instruction);
		endfunction
	
	endclass: Transaction
	
	
	class Generator;
	
		mailbox #(Transaction) gen2agt;
		Transaction t;
		Config cfg;
		
		function new (ref mailbox #(Transaction) g2a, ref Config c);
			gen2agt = g2a;
			cfg = c;
		endfunction
		
		task run;
			//for(integer i = 0; i < cfg.num_instructions; i++) begin
			while(1) begin
				//$display("Creating new transaction in Generator");
				t = new();
				`SV_RAND_CHECK(t.randomize());
				//$display("T.opcode in Generator: 0x%h", t.opcode);
				gen2agt.put(t);
			end
			//$display("Returning from Generator.run");
		endtask
	
	endclass: Generator
	
	
	class Agent;
	
		mailbox #(Transaction) gen2agt, agt2drv;
		Transaction t;
		
		function new (ref mailbox #(Transaction) g2a, a2d);
			gen2agt = g2a;
			agt2drv = a2d;
		endfunction
		
		task run;
			while(1) begin
				gen2agt.get(t);
				agt2drv.put(t);
				//$display("Passed one transaction from Generator to Driver");
			end
		endtask
		
	endclass: Agent	
	
	virtual class Driver_cbs;
		virtual task pre_tx(ref Transaction t);
		// By default, callback does nothing
		endtask
		virtual task post_tx(ref Transaction t, integer count);
		// By default, callback does nothing
		endtask
	endclass
	
	class Driver_cbs_coverage extends Driver_cbs;
		Transaction sample_t;
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
			branch_offset: coverpoint sample_t.instruction[8]{
				option.weight = 0;
			}
			branch_condition: coverpoint sample_t.instruction[11:9]{
				option.weight = 0;
			}
			reset: coverpoint sample_t.rst{
				option.weight = 0;
			}
			reset_cycle: coverpoint sample_t.rst_cycle{
                bins cycle[] = {[1:8]};
				option.weight = 0;
			}
			// Ensures that each opcode is followed and preceded by all other opcodes
			diff_seq_test: coverpoint sample_t.opcode{
				bins opcode_seq[] = (op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap => op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap);
			}
			// Ensures that each opcode is followed and preceded by all other opcodes
			same_seq_test: coverpoint sample_t.opcode{
				bins same0 = (op_add => op_add);
				bins same1 = (op_and => op_and);
				bins same2 = (op_not => op_not);
				bins same3 = (op_br => op_br);
				bins same4 = (op_jmp => op_jmp);
				bins same5 = (op_jsr => op_jsr);
				bins same6 = (op_ld => op_ld);
				bins same7 = (op_ldr => op_ldr);
				bins same8 = (op_lea => op_lea);
				bins same9 = (op_ldi => op_ldi);
				bins same10 = (op_st => op_st);
				bins same11 = (op_str => op_str);
				bins same12 = (op_sti => op_sti);
				bins same13 = (op_rti => op_rti);
				bins same14 = (op_ioe => op_ioe);
				bins same15 = (op_trap => op_trap);
			}
			
			//op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap
			reset_all_cycles_of_instr: cross opcode, reset, reset_cycle{
				option.weight = 1;
				//bins rst_ld = binsof(opcode) intersect{op_ld};
				bins rst_all_cycle_ops = binsof(opcode) intersect{op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
				bins cycle = binsof(reset_cycle) intersect{[1:8]};
				bins rst_high = binsof(reset) intersect{1'b1};
			}
			reset_all_opcodes: cross opcode, reset{
				option.weight = 1;
				bins rst_all_ops = binsof(opcode) intersect{op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			branch_with_nzp: cross branch_condition, opcode{
				option.weight = 1;
				bins branching_nzp = binsof(opcode) intersect{op_br};
			}
			branch_pos_and_neg: cross branch_offset, opcode{
				option.weight = 1;
				bins branching_pos_neg = binsof(opcode) intersect{op_br};
			}
			// Test that all opcodes with a source register have been tested with all possible source registers
			opcode_src_11_9: cross opcode, source11_9{
				option.weight = 1;
				ignore_bins ignore_no_src11_9 = binsof(opcode) intersect {op_add, op_and, op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			opcode_src_8_6: cross opcode, source8_6{
				option.weight = 1;
				ignore_bins ignore_no_src8_6 = binsof(opcode) intersect {op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a source register have been tested with all possible source registers
			opcode_src_2_0: cross opcode, source2_0{
				option.weight = 1;
				ignore_bins ignore_no_src2_0 = binsof(opcode) intersect {op_not, op_br, op_jmp, op_jsr, op_ld, op_ldr, op_lea, op_ldi, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			// Test that all opcodes with a dst register have been tested with all possible dst registers
			opcode_dst: cross opcode, destination{
				option.weight = 1;
				ignore_bins ignore_no_dst = binsof(opcode) intersect {op_br, op_jmp, op_jsr, op_st, op_str, op_sti, op_rti, op_ioe, op_trap};
			}
			
			//TODO: make more cover groups
		endgroup
		
		virtual task pre_tx(ref Transaction t);
			sample_t = t;
			CovOp.sample;
		endtask
		
		virtual task post_tx(ref Transaction t, integer count);
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
		logic [15:0] pc;
		logic [15:0] mem_addr;
		logic [15:0] mem_val;
		logic [15:0] instr;
		logic n_flag, z_flag, p_flag;

        function new();
            pc = 0;
            mem_addr = 0;
            mem_val = 0;
            regs[0] = 0;
            regs[1] = 0;
            regs[2] = 0;
            regs[3] = 0;
            regs[4] = 0;
            regs[5] = 0;
            regs[6] = 0;
            regs[7] = 0;
			instr = 0;
			n_flag = 0;
			z_flag = 0;
			p_flag = 0;
        endfunction
		
		function void to_string();
			$display("PC: 0x%h", pc);
			$display("Mem Addr: 0x%h", mem_addr);
			$display("Mem Val: 0x%h", mem_val);
			$display("Registers: 0: 0x%h 1: 0x%h 2: 0x%h 3: 0x%h 4: 0x%h 5: 0x%h 6: 0x%h 7: 0x%h", regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
			$display("Flags: N=%b Z=%b P=%b", n_flag, z_flag, p_flag);
		endfunction
	endclass: State
	
	typedef class Scoreboard;
	
	class Driver;
		virtual test_if.TB2DUT tb_ports;
		mailbox #(Transaction) agt2drv;
		Transaction t;
		Driver_cbs cbs[$];
		integer count;
		State s;
		Scoreboard sb;
		mailbox #(State) driver_states;
		int temp;
		shortint signed_reg;
		
		function new (ref mailbox #(Transaction) a2d, virtual test_if.TB2DUT tbd, input Scoreboard sb, mailbox #(State) d);
			// Ports
			tb_ports = tbd;
			
			// Mailbox
			agt2drv = a2d;
			
			// Instruction count
			count = 0;
			
			driver_states = d;
			this.sb = sb;
			s = new();
		endfunction
		
		task run;
			//$display("Starting Driver run task");
			// Reset the DUT
			tb_ports.reset <= 1;
			repeat (3) @(posedge tb_ports.clk);
			tb_ports.reset <= 0;
			
			// Begin driving instructions
			while(1) begin
				if(sb.cfg.done == 1) break;
				s = new();
				agt2drv.get(t);
				//$display("Starting new transaction in Driver");
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				//$display("New Instruction: 0x%h", t.instruction[15:12]);
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 1) begin
					//$display("[%t]\t RESET,cycle_number = %d, instruction = 0x%h!", $realtime,t.rst_cycle,t.instruction);
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					//@tb_ports.clk;
					continue;
				end
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 2) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					//@tb_ports.clk;
					continue;
				end
				//$display("%g\tInserting instruction %x into address %x", $time, t.instruction, tb_ports.pc);
				$root.lc3_top.dut_mem.my_memory[tb_ports.pc] = t.instruction;
				sb.my_memory[tb_ports.pc] = t.instruction;
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 3) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					//@tb_ports.clk;
					continue;
				end
				@(posedge tb_ports.clk); 
				if(t.rst == 1 && t.rst_cycle == 4) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					//@tb_ports.clk;
					continue;
				end
				if($root.lc3_top.my_lc3.dut_id == 1) begin
					@(posedge tb_ports.clk); 
					if(t.rst == 1 && t.rst_cycle == 4) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
				end
				s.instr = t.instruction;
				s.pc = tb_ports.pc;
				s.regs[0] = tb_ports.r0;
				s.regs[1] = tb_ports.r1;
				s.regs[2] = tb_ports.r2;
				s.regs[3] = tb_ports.r3;
				s.regs[4] = tb_ports.r4;
				s.regs[5] = tb_ports.r5;
				s.regs[6] = tb_ports.r6;
				s.regs[7] = tb_ports.r7;
				s.n_flag = $root.lc3_top.my_lc3.n_flag;
				s.z_flag = $root.lc3_top.my_lc3.z_flag;
				s.p_flag = $root.lc3_top.my_lc3.p_flag;
				//$display("%Driver Sampling DUT in state %h\n", $time, $root.lc3_top.my_lc3.state_logic);
				//$display("Opcode: 0x%h", t.opcode);
				if((t.opcode == op_ldi) || (t.opcode == op_sti)) begin // 8 clk cycles
					if(t.opcode == op_sti) begin // store indirect
/* 						if(my_memory[s.pc + t.pc_offset9] == s.pc + t.pc_offset9) begin
							s.mem_addr = s.pc + t.pc_offset9;
						end else begin
							s.mem_addr = my_memory[s.pc + t.pc_offset9];
						end */
						//$display("Before: S.pc: %h t.pc_offset9: %h PC+Offset: %h mem[pc+off]: %h mem[mem[pc+off]]: %h", s.pc, t.pc_offset9, s.pc + t.pc_offset9, sb.my_memory[s.pc + t.pc_offset9], sb.my_memory[sb.my_memory[s.pc + t.pc_offset9]]);
						s.mem_addr = s.pc + t.pc_offset9;
						//s.mem_val = s.regs[t.dst];
						if(!t.rst || (t.rst && t.rst_cycle == 0))
						sb.my_memory[sb.my_memory[s.mem_addr]] = s.regs[t.dst];
						//$display("After: S.pc: %h t.pc_offset9: %h PC+Offset: %h mem[pc+off]: %h mem[mem[pc+off]]: %h", s.pc, t.pc_offset9, s.pc + t.pc_offset9, sb.my_memory[s.pc + t.pc_offset9], sb.my_memory[sb.my_memory[s.pc + t.pc_offset9]]);
						
						
/* 						s.mem_addr = my_memory[s.pc + t.pc_offset9];
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						my_memory[s.mem_addr] = s.mem_val; */
						//$display("************************************tb pc: 0x%h tb pcoffset9: 0x%h", s.pc, t.pc_offset9);
					end else if(t.opcode == op_ldi) begin // load indirect
						s.regs[t.dst] = sb.my_memory[sb.my_memory[s.pc + t.pc_offset9]];
						s.mem_addr = s.pc + t.pc_offset9;
						//s.mem_val = my_memory[s.pc + t.pc_offset9];
/* 						s.dst_val = my_memory[my_memory[s.pc + t.pc_offset9]];
						s.dst_reg = t.dst; */
					end
					@(posedge tb_ports.clk);
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 7) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
					@(posedge tb_ports.clk);			
					if(t.rst == 1 && t.rst_cycle == 8) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
				end else if((t.opcode == op_ld) || (t.opcode == op_st) || (t.opcode == op_str) || (t.opcode == op_ldr)) begin // 6 clk cycles
					if(t.opcode == op_st) begin // store
						s.mem_addr = s.pc + t.pc_offset9;
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						if(!t.rst || (t.rst && (t.rst_cycle > 6 || t.rst_cycle < 1)))
						sb.my_memory[s.mem_addr] = s.mem_val;
					end else if(t.opcode == op_str) begin // store register
						s.mem_addr = s.regs[t.src1] + t.offset6;
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						if(!t.rst || (t.rst && (t.rst_cycle > 6 || t.rst_cycle < 1)))
						sb.my_memory[s.mem_addr] = s.mem_val;
					end else if(t.opcode == op_ld) begin // load
						s.regs[t.dst] = sb.my_memory[s.pc + t.pc_offset9];
						s.mem_addr = s.pc + t.pc_offset9;
/* 						s.dst_val = my_memory[s.pc + t.pc_offset9];
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_ldr) begin // load register
						//$display("LDR Instruction: t.dst: 0x%h t.src1: 0x%h t.offset6: 0x%h src+offset: 0x%h mem[src+offset]: 0x%h", t.dst, t.src1, t.offset6, t.src1+t.offset6, my_memory[s.regs[t.src1] + t.offset6]);
						s.regs[t.dst] = sb.my_memory[s.regs[t.src1] + t.offset6];
						s.mem_addr = s.regs[t.src1] + t.offset6;
/* 						s.dst_val = my_memory[s.src1 + t.offset6];
						s.dst_reg = t.dst; */
					end
					@(posedge tb_ports.clk);
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
				end else if(t.opcode == op_trap) begin // 5 clk cycles (just trap)
					s.regs[7] = s.pc;
					s.pc = t.trapvec8;
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						//@tb_ports.clk;
						continue;
					end
				end else if((t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_not) || (t.opcode == op_br) || (t.opcode == op_jmp) 
							|| (t.opcode == op_jsr) || (t.opcode == op_rti) || (t.opcode == op_ioe) || (t.opcode == op_lea)) begin // 4 clk cycles
					if(t.opcode == op_add) begin //add instruction
						if(t.imm5_bit == 1) begin
							s.regs[t.dst] = s.regs[t.src1] + t.imm5_val;
/* 							s.dst_val = s.regs[t.dst];
							s.dst_reg = t.dst; */
						end else begin
							s.regs[t.dst] = s.regs[t.src1] + s.regs[t.src2];
/* 							s.dst_val = s.regs[t.dst];
							s.dst_reg = t.dst; */
						end
					end else if(t.opcode == op_and) begin //and instruction
						if(t.imm5_bit == 1) begin
							s.regs[t.dst] = s.regs[t.src1] & t.imm5_val;
/* 							s.dst_val = s.regs[t.dst];
							s.dst_reg = t.dst; */
						end else begin
							s.regs[t.dst] = s.regs[t.src1] & s.regs[t.src2];
/* 							s.dst_val = s.regs[t.dst];
							s.dst_reg = t.dst; */
						end
					end else if(t.opcode == op_not) begin //not instruction
						//$display("SRC: %h", s.regs[t.src1]);
						s.regs[t.dst] = ~(s.regs[t.src1]);
						//$display("DST: %h", s.regs[t.dst]);
/* 						s.dst_val = s.regs[t.dst];
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_br) begin //branch instruction
						if((t.n_flag & tb_ports.n_flag) || (t.z_flag & tb_ports.z_flag) || (t.p_flag & tb_ports.p_flag)) begin
							s.pc = s.pc + t.pc_offset9;
						end
					end else if(t.opcode == op_jmp) begin // jump
						s.pc = s.regs[t.src1];
					end else if(t.opcode == op_jsr) begin // jump sub routine
						temp = s.regs[t.src1];
						s.regs[7] = s.pc;
						if(t.jsr_bit == 1) begin
							s.pc = s.pc + t.pc_offset11;
						end else begin
							s.pc = temp;
						end
					end else if(t.opcode == op_lea) begin // load effective address
						s.regs[t.dst] = s.pc + t.pc_offset9;
/* 						s.dst_val = s.pc + t.pc_offset9;
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_rti || t.opcode == op_ioe) begin
						//$display("Found opcode rti or ioe");
						// do nothing
					end else begin
						$display("Invalid opcode");
					end
				end else begin
					$display("%g\tUnknown Instruction %d Received!", $time, t.opcode);
				end
				if((t.opcode == op_lea) || (t.opcode == op_not) || (t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_ld) || (t.opcode == op_ldr) || (t.opcode == op_ldi)) begin
					//$display("Value = 0x%h", s.regs[t.dst]);
					signed_reg = s.regs[t.dst];
					if(signed_reg < 0) begin
						s.n_flag = 1;
						s.z_flag = 0;
						s.p_flag = 0;
					end else if(signed_reg > 0) begin
						s.n_flag = 0;
						s.z_flag = 0;
						s.p_flag = 1;
					end else begin
						s.n_flag = 0;
						s.z_flag = 1;
						s.p_flag = 0;
					end
				end
				foreach(cbs[i]) begin
					cbs[i].post_tx(t, count);
				end
				driver_states.put(s);
				count++;
			end
		endtask
		
	endclass: Driver
	
	
	class Config;
		integer errors;
		integer num_instructions;
		integer done;
		function new(integer ni=10);
			errors = 0;
			done = 0;
			num_instructions = ni;
		endfunction
	endclass: Config

	
	class Monitor;
		virtual test_if.TB2DUT ports;
		//TODO: Monitor callbacks?
		State s;
		Scoreboard sb;
		Transaction t;
		int resetting;
		int state_int;
		
		function new(input virtual test_if.TB2DUT p, input Scoreboard scb);
			ports = p;
			sb = scb;
			resetting = 1;
		endfunction
		
		task run();
			forever begin
				//$display("0Resetting is %d and State_logic is %d and fetch0 is %d", resetting, $root.lc3_top.my_lc3.state_logic, fetch0);
				if(sb.cfg.done == 1) break;
				if(ports.reset == 1) begin
					//$display("0Resetting is %d and State_logic is %d and fetch0 is %d", resetting, $root.lc3_top.my_lc3.state_logic, fetch0);
					resetting = 1;
					@(posedge ports.clk);
					continue;
				end else begin
					//$display("1Resetting is %d and State_logic is %d and fetch0 is %d", resetting, $root.lc3_top.my_lc3.state_logic, fetch0);
					if($root.lc3_top.my_lc3.state_logic == start) begin
						@(posedge ports.clk);
					end
					@(posedge ports.clk);
					resetting = 0;
				end
				if($root.lc3_top.my_lc3.state_logic == 1 && resetting == 0) begin
					//$display("2Resetting is %d and State_logic is %d and fetch0 is %d", resetting, $root.lc3_top.my_lc3.state_logic, fetch0);
					t = new($root.lc3_top.my_lc3.ir);
					//$display("IR: 0x%h", $root.lc3_top.my_lc3.ir);
					s = new();
					resetting = 0;
					s.pc = ports.pc;
					s.regs[0] = ports.r0;
					s.regs[1] = ports.r1;
					s.regs[2] = ports.r2;
					s.regs[3] = ports.r3;
					s.regs[4] = ports.r4;
					s.regs[5] = ports.r5;
					s.regs[6] = ports.r6;
					s.regs[7] = ports.r7;
					s.n_flag = $root.lc3_top.my_lc3.n_flag;
					s.z_flag = $root.lc3_top.my_lc3.z_flag;
					s.p_flag = $root.lc3_top.my_lc3.p_flag;
					//$display("opcode is: 0x%d", t.opcode);
					if(t.opcode == op_st) begin
						s.mem_addr = s.pc + t.pc_offset9;
						s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end else if(t.opcode == op_str) begin
						s.mem_addr = s.regs[t.src1] + t.offset6;
						s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end else if(t.opcode == op_sti) begin
						//$display("********************************dut pc: 0x%h dut pcoffset9: 0x%h", s.pc, t.pc_offset9);
						s.mem_addr = s.pc + t.pc_offset9;
/* 						($root.lc3_top.dut_mem.my_memory[s.pc + t.pc_offset9] == s.regs[t.dst]) begin
							s.mem_addr = s.pc + t.pc_offset9;
						end else begin
							s.mem_addr = $root.lc3_top.dut_mem.my_memory[s.pc + t.pc_offset9];
						end */
						//s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end else if(t.opcode == op_ld) begin
						s.mem_addr = s.pc + t.pc_offset9;
					end else if(t.opcode == op_ldr) begin
						s.mem_addr = s.regs[t.src1] + t.offset6;
					end else if(t.opcode == op_ldi) begin
						s.mem_addr = s.pc + t.pc_offset9;
					end
					sb.check_actual(s);
					//$display("%gMonitor Sampling DUT\n", $time);
				end else begin
					//$display("3Resetting is %d and State_logic is %d and fetch0 is %d", resetting, $root.lc3_top.my_lc3.state_logic, fetch0);
				end
			end
		endtask
		
	endclass: Monitor
	
	
	class Scoreboard;
		Config cfg;
		integer before_errors;
		mailbox #(State) driver_states;
		State e;
		integer count;
		integer myint;
		integer error;
		bit [15:0] my_memory [0:65535];
		
		function new(ref Config c, mailbox #(State) d);
			driver_states = d;
			cfg = c;
			count = 0;
			error = 0;
			for(int i = 0; i < 65535; i++) begin
				my_memory[i] = $root.lc3_top.dut_mem.my_memory[i];
				//$display("Addr: %h Value: %h", i, my_memory[i]);
			end
		endfunction
		
		task check_actual(ref State a);
			$timeformat(-9, 3, "ns", 8);
			driver_states.get(e);
			before_errors = cfg.errors;
/* 			for(int i = 0; i < 65535; i++) begin
				if(my_memory[i] != $root.lc3_top.dut_mem.my_memory[i]) begin
					error = 1;
					$display("[%t]\tError: Memory Value does not match at address 0x%h after instr 0x%h! Expected 0x%h Actual 0x%h", $realtime, i, e.instr, my_memory[i], $root.lc3_top.dut_mem.my_memory[i]);
				end
			end */
			//if((a.pc != e.pc) || (a.mem_addr != e.mem_addr) || (a.mem_val != e.mem_val) || (a.regs[0] != e.regs[0]) || (a.regs[1] != e.regs[1]) 
			//		|| (a.regs[2] != e.regs[2]) || (a.regs[3] != e.regs[3]) || (a.regs[4] != e.regs[4]) || (a.regs[5] != e.regs[5]) 
			//		|| (a.regs[6] != e.regs[6]) || (a.regs[7] != e.regs[7]) || (a.p_flag != e.p_flag) || (a.n_flag != e.n_flag) || (a.z_flag != e.z_flag)) begin
				$display("Instruction %d: 0x%h", count, e.instr);
				$display("Expected:");
				e.to_string();
				$display("Actual:");
				a.to_string();
				if(a.pc != e.pc) begin
					error = 1;
					$display("[%t]\tError: PC does not match!", $realtime);
				end
				if(a.mem_addr != e.mem_addr) begin
					error = 1;
					$display("[%t]\tError: Memory Address does not match!", $realtime);
				end
				if(a.mem_val != e.mem_val) begin
					error = 1;
					$display("[%t]\tError: Memory Value does not match!", $realtime);
				end
				for(integer i = 0; i < 8; i++) begin
					if(a.regs[i] != e.regs[i]) begin
						error = 1;
						$display("[%t]\tError: R%d does not match!", $realtime, i);
					end
				end
				if(a.n_flag != e.n_flag) begin
					error = 1;
					$display("[%t]\tError: N-Flag does not match!", $realtime);
				end
				if(a.z_flag != e.z_flag) begin
					error = 1;
					$display("[%t]\tError: Z-Flag does not match!", $realtime);
				end
				if(a.p_flag != e.p_flag) begin
					error = 1;
					$display("[%t]\tError: P-Flag does not match!", $realtime);
				end
				if(error == 0) begin
					$display("[%t]\tSuccess: expected and actual values match!", $realtime);
				end else begin
					cfg.errors++;
					error = 0;
				end
				$display(" ");
				if(e.instr[15:12] == 4'h6) begin
					$display("Reg index: %h Address: %h Value in dut mem: %h Value in golden mem %h", a.instr[8:6], a.regs[a.instr[8:6]], $root.lc3_top.dut_mem.my_memory[a.regs[a.instr[8:6]]], my_memory[a.instr[8:6]]);
				end
			//end
			count++;
			if(count >= cfg.num_instructions) begin
				cfg.done = 1;
			end
		endtask
	endclass: Scoreboard
	
	
	class Environment;
		virtual test_if.TB2DUT tb_ports;
		Generator gen;
		Agent agt;
		Driver drv;
		Monitor mon;
		Scoreboard sb;
		Config cfg;
		mailbox #(Transaction ) gen2agt, agt2drv;
		mailbox #(State) driver_states;
		
		function new(virtual test_if.TB2DUT tbd);
			tb_ports = tbd;
		endfunction
		
		function void build();
			//Initialize mailboxes
			gen2agt = new(1);
			agt2drv = new(1);
			driver_states = new(5);
			
			//Initialize transactors
			cfg = new();
			sb = new(cfg, driver_states);
			//$display("My Integer is: %d", sb.myint);
			gen = new(gen2agt, cfg);
			agt = new(gen2agt, agt2drv);
			drv = new(agt2drv, tb_ports, sb, driver_states);
			mon = new(tb_ports, sb);
		endfunction
		
		task run();
			fork
				gen.run();
				agt.run();
				drv.run();
				mon.run();
			join_any
            $display("Finishing Env.run");
		endtask
		
		function void wrap_up();
			$display("Instructions run = %d", drv.count);
			$display("TOTAL ERRORS: %d", cfg.errors);
		endfunction
		
	endclass: Environment
	
	virtual class Component ;  			// tests come from this class
		pure virtual task run_test(); 
	endclass: Component
	
	
	virtual class Wrapper ;				//abstract common class for proxy class
		pure virtual function string get_type_name();
		pure virtual function Component create_object(string name);
	endclass: Wrapper


	class Factory ;							//sigleton class.. holds associative from string to proxy class
	   static Wrapper type_names[string];
	   static Factory inst;
	   
	   static function Factory get();
		  if (inst == null) inst = new();
		  return inst;
	   endfunction

	   static function void register(Wrapper c);
          $display("Registering class: %s", c.get_type_name());
		  type_names[c.get_type_name()] = c;
	   endfunction

	   static function Component get_test();
		  string name;
		  Wrapper test_wrapper;
		  Component test_component;
			if (!$value$plusargs("TESTNAME=%s", name)) begin
				$display("FATAL +TESTNAME not found");
				$finish;
			end
		  $display("%m found TESTNAME=%s", name);
		  test_wrapper = Factory::type_names[name];
		  $cast(test_component, test_wrapper.create_object(name));
		  return test_component;
	   endfunction

	   static function void printFactory();
		  $display("Factory:");
		  foreach (type_names[s]) $display("The name: %s maps to the class: %p", s, type_names[s]);
	   endfunction
	endclass: Factory
	
	class Registry #(type T, string Tname) extends Wrapper;			//proxy class parameterized
		typedef Registry #(T,Tname) this_type;
		local static this_type me = get();
		T test;

		static function this_type get();
			if (me == null) begin
				Factory f = Factory::get();  // Build factory
				me = new();
				void'(f.register(me));
			end
			return me;
		endfunction

		virtual function string get_type_name();
			return Tname;
		endfunction

		virtual function Component create_object(string name);
			test = new(name);
			return test;
		endfunction

	endclass:Registry
	
	class TestBasic extends Component;
		typedef Registry #(TestBasic, "TestBasic") type_id;
		Environment env;
		Driver_cbs_coverage dcv;
		virtual test_if.TB2DUT tbdut_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
		endfunction
		
		virtual task run_test();
			$display("Starting TestBasic run_test");
			tbdut_if = $root.lc3_top.tbdut_if.TB2DUT;
			env = new(tbdut_if);
			env.build();
			$display("Built Environment in TestBasic");
			env.cfg.num_instructions = 1000000;
			begin
				dcv = new();
				env.drv.cbs.push_back(dcv);
			end
			env.run();
			$display("Finished Running in TestBasic");
			env.wrap_up();
		endtask
	endclass: TestBasic
	
endpackage
