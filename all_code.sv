`ifndef ASSERT_MACROS
`define ASSERT_MACROS

`define assert_clk_xrst(arg) \
	assert property (@(posedge clk) disable iff (reset) arg)
	
`define assert_clk(arg) \
	assert property (@(posedge clk) arg)
	
`endif



// File Name: risc_top.sv
// This file is the top level of a system that tests a simple RISC processor.

`default_nettype none
`timescale 1ns / 1ns
module lc3_top;

	bit clk;
	// Clock generator
	initial begin
		clk = 0;
		forever #50 clk = ~clk;
	end
	
	//Interfaces
	test_if tbdut_if_0(clk);
	test_if tbdut_if_1(clk);
	
	// Test bench
	lc3_tb my_tb();
	
	// Memory
	memory dut_mem_0(.clk(clk), .memWE(tbdut_if_0.memwe), .memOut(tbdut_if_0.memOut), .reset(tbdut_if_0.reset), .mdrOut(tbdut_if_0.mdr), .MARReg(tbdut_if_0.mar));
	memory dut_mem_1(.clk(clk), .memWE(tbdut_if_1.memwe), .memOut(tbdut_if_1.memOut), .reset(tbdut_if_1.reset), .mdrOut(tbdut_if_1.mdr), .MARReg(tbdut_if_1.mar));
	
	// DUT
	ammon_lc3 my_lc3_0(.clk(tbdut_if_0.clk),
        .reset(tbdut_if_0.reset), 
        .memwe(tbdut_if_0.memwe), 
        .mdr(tbdut_if_0.mdr), 
        .mar(tbdut_if_0.mar), 
        .memOut(tbdut_if_0.memOut));
	
	khalil_LC3 my_lc3_1(.clk(tbdut_if_1.clk),
        .reset(tbdut_if_1.reset), 
        .memwe(tbdut_if_1.memwe), 
        .mdr(tbdut_if_1.mdr), 
        .mar(tbdut_if_1.mar), 
        .memOut(tbdut_if_1.memOut));
		
endmodule



// File Name: risc_tb.v
// This file tests the RISC processor with constrained random testing.

`default_nettype none
`timescale 1ns / 100ps
program automatic lc3_tb();
	
	import lc3_pkg::*;
	
	Component test;
	initial begin
		test = Factory::get_test();
		test.run_test();
	end
	
endprogram



// File Name: lc3_if.sv
// This file contains the interface used with the LC3 processor.

interface test_if (input bit clk);
	logic reset;
 	logic memwe;
	logic [15:0] mdr, memOut;
	logic [15:0] mar;
	logic [15:0] pc, r0, r1, r2, r3, r4, r5, r6, r7;
	logic n_flag, z_flag, p_flag;
	
	modport TB2DUT(output reset, input clk, memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag, memOut);
	modport DUT2TB(input clk, reset, memOut, output memwe, mdr, mar, pc, r0, r1, r2, r3, r4, r5, r6, r7, n_flag, z_flag, p_flag);
	modport DUT2MEM(input clk, reset, mar, mdr, memwe, memOut);
endinterface



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
		rand bit rst;
		rand integer rst_cycle;
		
		constraint rst_d { rst dist {0:=90, 1:=10};}
		constraint rst_c { rst_cycle >= 0; rst_cycle <= 8;}
		
		function new (bit [15:0] i=0);
			instruction = i;
			opcode = opcode_type'({28'h0, instruction[15:12]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = { {11{instruction[4]}}, instruction[4:0]};
			imm5_bit = instruction[5];
			pc_offset9 = { {7{instruction[8]}}, instruction[8:0]};
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = { {5{instruction[10]}}, instruction[10:0]};
			trapvec8 = { {8{0}}, instruction[7:0]};
			offset6 = { {10{instruction[5]}}, instruction[5:0]};
		endfunction
		
		
		
		function void post_randomize();
			opcode = opcode_type'({28'h0, instruction[15:12]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = { {11{instruction[4]}}, instruction[4:0]};
			imm5_bit = instruction[5];
			pc_offset9 = { {7{instruction[8]}}, instruction[8:0]};
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = { {5{instruction[10]}}, instruction[10:0]};
			trapvec8 = { {8{0}}, instruction[7:0]};
			offset6 = { {10{instruction[5]}}, instruction[5:0]};
		endfunction
	
	endclass: Transaction
	
	
	class Generator;
	
		mailbox #(Transaction) gen2drv;
		Transaction t;
		Config cfg;
		
		function new (ref mailbox #(Transaction) g2d, ref Config c);
			gen2drv = g2d;
			cfg = c;
		endfunction
		
		task run;
			while(1) begin
				t = new();
				`SV_RAND_CHECK(t.randomize());
				gen2drv.put(t);
			end
		endtask
	
	endclass: Generator
	
	
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
			// Ensures that each opcode is followed and preceded by itself
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
		mailbox #(Transaction) gen2drv;
		Transaction t;
		Driver_cbs cbs[$];
		integer count;
		State s;
		Scoreboard sb;
		mailbox #(State) driver_states;
		int temp;
		shortint signed_reg;
		
		function new (ref mailbox #(Transaction) g2d, virtual test_if.TB2DUT tbd, input Scoreboard sb, mailbox #(State) d);
			// Ports
			tb_ports = tbd;
			
			// Mailbox
			gen2drv = g2d;
			
			// Instruction count
			count = 0;
			
			driver_states = d;
			this.sb = sb;
			s = new();
		endfunction
		
		task run;
			// Reset the DUT
			tb_ports.reset <= 1;
			repeat (3) @(posedge tb_ports.clk);
			tb_ports.reset <= 0;
			
			// Begin driving instructions
			while(1) begin
				if(sb.cfg.done == 1) break;
				s = new();
				gen2drv.get(t);
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 1) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					continue;
				end
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 2) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					continue;
				end
				if(sb.cfg.dut_id == 0) begin
					$root.lc3_top.dut_mem_0.my_memory[$root.lc3_top.my_lc3_0.pc] = t.instruction;
					sb.my_memory[$root.lc3_top.my_lc3_0.pc] = t.instruction;
				end else begin
					$root.lc3_top.dut_mem_1.my_memory[$root.lc3_top.my_lc3_1.pc] = t.instruction;
					sb.my_memory[$root.lc3_top.my_lc3_1.pc] = t.instruction;
				end
				@(posedge tb_ports.clk);
				if(t.rst == 1 && t.rst_cycle == 3) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					continue;
				end
				@(posedge tb_ports.clk); 
				if(t.rst == 1 && t.rst_cycle == 4) begin
					tb_ports.reset <= 1;
					@(posedge tb_ports.clk);
					tb_ports.reset <= 0;
					continue;
				end
				if(sb.cfg.dut_id == 1) begin
					@(posedge tb_ports.clk); 
					if(t.rst == 1 && t.rst_cycle == 4) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						continue;
					end
				end
				s.instr = t.instruction;
				if(sb.cfg.dut_id == 0) begin
					s.pc = $root.lc3_top.my_lc3_0.pc;
					s.regs[0] = $root.lc3_top.my_lc3_0.r0_out;
					s.regs[1] = $root.lc3_top.my_lc3_0.r1_out;
					s.regs[2] = $root.lc3_top.my_lc3_0.r2_out;
					s.regs[3] = $root.lc3_top.my_lc3_0.r3_out;
					s.regs[4] = $root.lc3_top.my_lc3_0.r4_out;
					s.regs[5] = $root.lc3_top.my_lc3_0.r5_out;
					s.regs[6] = $root.lc3_top.my_lc3_0.r6_out;
					s.regs[7] = $root.lc3_top.my_lc3_0.r7_out;
					s.n_flag = $root.lc3_top.my_lc3_0.n_flag;
					s.z_flag = $root.lc3_top.my_lc3_0.z_flag;
					s.p_flag = $root.lc3_top.my_lc3_0.p_flag;
				end else if(sb.cfg.dut_id == 1) begin
					s.pc = $root.lc3_top.my_lc3_1.pc;
					s.regs[0] = $root.lc3_top.my_lc3_1.r0_out;
					s.regs[1] = $root.lc3_top.my_lc3_1.r1_out;
					s.regs[2] = $root.lc3_top.my_lc3_1.r2_out;
					s.regs[3] = $root.lc3_top.my_lc3_1.r3_out;
					s.regs[4] = $root.lc3_top.my_lc3_1.r4_out;
					s.regs[5] = $root.lc3_top.my_lc3_1.r5_out;
					s.regs[6] = $root.lc3_top.my_lc3_1.r6_out;
					s.regs[7] = $root.lc3_top.my_lc3_1.r7_out;
					s.n_flag = $root.lc3_top.my_lc3_1.n_flag;
					s.z_flag = $root.lc3_top.my_lc3_1.z_flag;
					s.p_flag = $root.lc3_top.my_lc3_1.p_flag;
				end
				if((t.opcode == op_ldi) || (t.opcode == op_sti)) begin // 8 clk cycles
					if(t.opcode == op_sti) begin // store indirect
						s.mem_addr = s.pc + t.pc_offset9;
						if(!t.rst || (t.rst && t.rst_cycle == 0))
						sb.my_memory[sb.my_memory[s.mem_addr]] = s.regs[t.dst];
					end else if(t.opcode == op_ldi) begin // load indirect
						s.regs[t.dst] = sb.my_memory[sb.my_memory[s.pc + t.pc_offset9]];
						s.mem_addr = s.pc + t.pc_offset9;
					end
					@(posedge tb_ports.clk);
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 7) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						continue;
					end
					@(posedge tb_ports.clk);			
					if(t.rst == 1 && t.rst_cycle == 8) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
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
					end else if(t.opcode == op_ldr) begin // load register
						s.regs[t.dst] = sb.my_memory[s.regs[t.src1] + t.offset6];
						s.mem_addr = s.regs[t.src1] + t.offset6;
					end
					@(posedge tb_ports.clk);
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
						continue;
					end
					@(posedge tb_ports.clk);				
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						@(posedge tb_ports.clk);
						tb_ports.reset <= 0;
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
						continue;
					end
				end else if((t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_not) || (t.opcode == op_br) || (t.opcode == op_jmp) 
							|| (t.opcode == op_jsr) || (t.opcode == op_rti) || (t.opcode == op_ioe) || (t.opcode == op_lea)) begin // 4 clk cycles
					if(t.opcode == op_add) begin //add instruction
						if(t.imm5_bit == 1) begin
							s.regs[t.dst] = s.regs[t.src1] + t.imm5_val;
						end else begin
							s.regs[t.dst] = s.regs[t.src1] + s.regs[t.src2];
						end
					end else if(t.opcode == op_and) begin //and instruction
						if(t.imm5_bit == 1) begin
							s.regs[t.dst] = s.regs[t.src1] & t.imm5_val;
						end else begin
							s.regs[t.dst] = s.regs[t.src1] & s.regs[t.src2];
						end
					end else if(t.opcode == op_not) begin //not instruction
						s.regs[t.dst] = ~(s.regs[t.src1]);
					end else if(t.opcode == op_br) begin //branch instruction
						if(sb.cfg.dut_id == 0) begin
							if((t.n_flag & $root.lc3_top.my_lc3_0.n_flag) || (t.z_flag & $root.lc3_top.my_lc3_0.z_flag) || (t.p_flag & $root.lc3_top.my_lc3_0.p_flag)) begin
								s.pc = s.pc + t.pc_offset9;
							end
						end else begin
							if((t.n_flag & $root.lc3_top.my_lc3_1.n_flag) || (t.z_flag & $root.lc3_top.my_lc3_1.z_flag) || (t.p_flag & $root.lc3_top.my_lc3_1.p_flag)) begin
								s.pc = s.pc + t.pc_offset9;
							end
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
					end else if(t.opcode == op_rti || t.opcode == op_ioe) begin
						// do nothing
					end else begin
						$display("Invalid opcode");
					end
				end else begin
					$display("%g\tUnknown Instruction %d Received!", $time, t.opcode);
				end
				if((t.opcode == op_lea) || (t.opcode == op_not) || (t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_ld) || (t.opcode == op_ldr) || (t.opcode == op_ldi)) begin
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
		integer dut_id;
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
				if(sb.cfg.done == 1) break;
				if(ports.reset == 1) begin
					resetting = 1;
					@(posedge ports.clk);
					continue;
				end else begin
					if(sb.cfg.dut_id == 0) begin
						state_int = $root.lc3_top.my_lc3_0.state_logic;
					end else begin
						state_int = $root.lc3_top.my_lc3_1.state_logic;
					end
					if(state_int == start) begin
						@(posedge ports.clk);
					end
					@(posedge ports.clk);
					resetting = 0;
				end
				if(sb.cfg.dut_id == 0) begin
					state_int = $root.lc3_top.my_lc3_0.state_logic;
				end else begin
					state_int = $root.lc3_top.my_lc3_1.state_logic;
				end
				if(state_int == 1 && resetting == 0) begin
					s = new();
					resetting = 0;
					if(sb.cfg.dut_id == 0) begin
						t = new($root.lc3_top.my_lc3_0.ir);
						s.pc = $root.lc3_top.my_lc3_0.pc;
						s.regs[0] = $root.lc3_top.my_lc3_0.r0_out;
						s.regs[1] = $root.lc3_top.my_lc3_0.r1_out;
						s.regs[2] = $root.lc3_top.my_lc3_0.r2_out;
						s.regs[3] = $root.lc3_top.my_lc3_0.r3_out;
						s.regs[4] = $root.lc3_top.my_lc3_0.r4_out;
						s.regs[5] = $root.lc3_top.my_lc3_0.r5_out;
						s.regs[6] = $root.lc3_top.my_lc3_0.r6_out;
						s.regs[7] = $root.lc3_top.my_lc3_0.r7_out;
						s.n_flag = $root.lc3_top.my_lc3_0.n_flag;
						s.z_flag = $root.lc3_top.my_lc3_0.z_flag;
						s.p_flag = $root.lc3_top.my_lc3_0.p_flag;
					end else if(sb.cfg.dut_id == 1) begin
						t = new($root.lc3_top.my_lc3_1.ir);
						s.pc = $root.lc3_top.my_lc3_1.pc;
						s.regs[0] = $root.lc3_top.my_lc3_1.r0_out;
						s.regs[1] = $root.lc3_top.my_lc3_1.r1_out;
						s.regs[2] = $root.lc3_top.my_lc3_1.r2_out;
						s.regs[3] = $root.lc3_top.my_lc3_1.r3_out;
						s.regs[4] = $root.lc3_top.my_lc3_1.r4_out;
						s.regs[5] = $root.lc3_top.my_lc3_1.r5_out;
						s.regs[6] = $root.lc3_top.my_lc3_1.r6_out;
						s.regs[7] = $root.lc3_top.my_lc3_1.r7_out;
						s.n_flag = $root.lc3_top.my_lc3_1.n_flag;
						s.z_flag = $root.lc3_top.my_lc3_1.z_flag;
						s.p_flag = $root.lc3_top.my_lc3_1.p_flag;
					end
					if(t.opcode == op_st) begin
						s.mem_addr = s.pc + t.pc_offset9;
						if(sb.cfg.dut_id == 0) begin
							s.mem_val = $root.lc3_top.dut_mem_0.my_memory[s.mem_addr];
						end else begin
							s.mem_val = $root.lc3_top.dut_mem_1.my_memory[s.mem_addr];
						end
					end else if(t.opcode == op_str) begin
						s.mem_addr = s.regs[t.src1] + t.offset6;
						if(sb.cfg.dut_id == 0) begin
							s.mem_val = $root.lc3_top.dut_mem_0.my_memory[s.mem_addr];
						end else begin
							s.mem_val = $root.lc3_top.dut_mem_1.my_memory[s.mem_addr];
						end
					end else if(t.opcode == op_sti) begin
						s.mem_addr = s.pc + t.pc_offset9;
					end else if(t.opcode == op_ld) begin
						s.mem_addr = s.pc + t.pc_offset9;
					end else if(t.opcode == op_ldr) begin
						s.mem_addr = s.regs[t.src1] + t.offset6;
					end else if(t.opcode == op_ldi) begin
						s.mem_addr = s.pc + t.pc_offset9;
					end
					sb.check_actual(s);
				end else begin
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
			
			if(cfg.dut_id == 0) begin
				for(int i = 0; i < 65535; i++) begin
					my_memory[i] = $root.lc3_top.dut_mem_0.my_memory[i];
				end
			end else begin
				for(int i = 0; i < 65535; i++) begin
					my_memory[i] = $root.lc3_top.dut_mem_1.my_memory[i];
				end
			end
		endfunction
		
		task check_actual(ref State a);
			$timeformat(-9, 3, "ns", 8);
			driver_states.get(e);
			before_errors = cfg.errors;
			if((a.pc != e.pc) || (a.mem_addr != e.mem_addr) || (a.mem_val != e.mem_val) || (a.regs[0] != e.regs[0]) || (a.regs[1] != e.regs[1]) 
					|| (a.regs[2] != e.regs[2]) || (a.regs[3] != e.regs[3]) || (a.regs[4] != e.regs[4]) || (a.regs[5] != e.regs[5]) 
					|| (a.regs[6] != e.regs[6]) || (a.regs[7] != e.regs[7]) || (a.p_flag != e.p_flag) || (a.n_flag != e.n_flag) || (a.z_flag != e.z_flag)) begin
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
			end
			count++;
			if(count >= cfg.num_instructions) begin
				cfg.done = 1;
			end
		endtask
	endclass: Scoreboard
	
	
	class Environment;
		virtual test_if.TB2DUT tb_ports;
		Generator gen;
		Driver drv;
		Monitor mon;
		Scoreboard sb;
		Config cfg;
		mailbox #(Transaction ) gen2drv;
		mailbox #(State) driver_states;
		
		function new(virtual test_if.TB2DUT tbd);
			tb_ports = tbd;
		endfunction
		
		function void build();
			//Initialize mailboxes
			gen2drv = new(1);
			driver_states = new(5);
			
			//Initialize transactors
			cfg = new();
			sb = new(cfg, driver_states);
			gen = new(gen2drv, cfg);
			drv = new(gen2drv, tb_ports, sb, driver_states);
			mon = new(tb_ports, sb);
		endfunction
		
		task run();
			fork
				gen.run();
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
	
	class TestAmmonCov extends Component;
		typedef Registry #(TestAmmonCov, "TestAmmonCov") type_id;
		Environment env;
		Driver_cbs_coverage dcv;
		virtual test_if.TB2DUT tbdut_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
		endfunction
		
		virtual task run_test();
			tbdut_if = $root.lc3_top.tbdut_if_0.TB2DUT;
			env = new(tbdut_if);
			env.build();
			$display("Built Environment in TestBasic");
			env.cfg.num_instructions = 1000000;
			env.cfg.dut_id = 0;
			begin
				dcv = new();
				env.drv.cbs.push_back(dcv);
			end
			env.run();
			$display("Finished Running in TestBasic");
			env.wrap_up();
		endtask
	endclass: TestAmmonCov
	
	class TestKhalilCov extends Component;
		typedef Registry #(TestKhalilCov, "TestKhalilCov") type_id;
		Environment env;
		Driver_cbs_coverage dcv;
		virtual test_if.TB2DUT tbdut_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
		endfunction
		
		virtual task run_test();
			tbdut_if = $root.lc3_top.tbdut_if_1.TB2DUT;
			env = new(tbdut_if);
			env.build();
			$display("Built Environment in TestBasic");
			env.cfg.num_instructions = 1000000;
			env.cfg.dut_id = 1;
			begin
				dcv = new();
				env.drv.cbs.push_back(dcv);
			end
			env.run();
			$display("Finished Running in TestBasic");
			env.wrap_up();
		endtask
	endclass: TestKhalilCov
	
	class TestAmmon extends Component;
		typedef Registry #(TestAmmon, "TestAmmon") type_id;
		Environment env;
		virtual test_if.TB2DUT tbdut_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
		endfunction
		
		virtual task run_test();
			tbdut_if = $root.lc3_top.tbdut_if_0.TB2DUT;
			env = new(tbdut_if);
			env.build();
			$display("Built Environment in TestBasic");
			env.cfg.num_instructions = 1000000;
			env.cfg.dut_id = 0;
			env.run();
			$display("Finished Running in TestBasic");
			env.wrap_up();
		endtask
	endclass: TestAmmon
	
	class TestKhalil extends Component;
		typedef Registry #(TestKhalil, "TestKhalil") type_id;
		Environment env;
		virtual test_if.TB2DUT tbdut_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
		endfunction
		
		virtual task run_test();
			tbdut_if = $root.lc3_top.tbdut_if_1.TB2DUT;
			env = new(tbdut_if);
			env.build();
			$display("Built Environment in TestBasic");
			env.cfg.num_instructions = 1000000;
			env.cfg.dut_id = 1;
			env.run();
			$display("Finished Running in TestBasic");
			env.wrap_up();
		endtask
	endclass: TestKhalil
	
endpackage


// File Name:  ammon_lc3.sv
// This module is where all the control signals of the LC3 are generated.  It
// instantiates the datapath module and passes all control signals to it.

`timescale 1ns / 1ns
`default_nettype none
`include "assert_macros.sv"

typedef enum {start, fetch0, fetch1, fetch2, decode, ex_ld1, ex_ld2, ex_st1, ex_st2, ex_ldi1, ex_ldi2, ex_ldi3, ex_ldi4, ex_sti1, ex_sti2, ex_sti3, ex_sti4, ex_str1, ex_str2, ex_ldr1, ex_ldr2, ex_trap1} state_type;
typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;

module ammon_lc3 (clk, reset, memwe, mdr, mar, memOut);
	input logic clk, reset;
	
	output logic [15:0] mar, mdr; 
	input logic [15:0] memOut;
	output logic memwe;
	logic n_flag, z_flag, p_flag;
	logic [15:0] pc, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out;

	// Signal Declarations
	logic [15:0] ir;
	logic [31:0] opcode; 
	logic ir_n, ir_z, ir_p; 
	logic take_branch; 
	state_type state, next_state; // State_type described in the "lc3_pkg" package
	int state_logic;
	assign state_logic = state;
	int dut_id = 0;

	// IR subsignals
	assign opcode = {28'h0, ir[15:12]};
	assign ir_n = ir[11];
	assign ir_z = ir[10];
	assign ir_p = ir[9];

	// Control Signals
	logic en_pc, en_alu, en_marmux, en_mdr, regwe, sel_marmux, 
	load_pc, load_ir, flagwe, load_mar, sel_mdr, 
	load_mdr, sel_eab1;
	logic [1:0] alu_ctl, sel_pc, sel_eab2;
	logic [2:0] dr, sr1, sr2;

	// Control Signal Integers
	integer en_pc_next, en_alu_next, en_marmux_next, en_mdr_next, regwe_next, sel_marmux_next, 
		load_pc_next, load_ir_next, flagwe_next, load_mar_next, memwe_next, sel_mdr_next, 
		load_mdr_next, sel_eab1_next;
	integer alu_ctl_next, sel_pc_next, sel_eab2_next;
	integer dr_next, sr1_next, sr2_next;
	
	ERR_RESET_SHOULD_SET_ALL_REGISTERS_TO_ZERO:
		`assert_clk(reset |-> pc==16'h0 && ir==16'h0 && mdr==16'h0 && mar==16'h0 && n_flag==0 && z_flag==0 && p_flag==0);
	
	ERR_FLAGS_SHOULD_NOT_BE_HIGH_AT_THE_SAME_TIME:
		`assert_clk(!((n_flag && p_flag) || (n_flag && z_flag) || (z_flag && p_flag)));
	
	ERR_MEMWE_IS_HIGH_DURING_NON_STORE_INSTRUCTION:
		`assert_clk((opcode != op_st) && (opcode != op_str) && (opcode != op_sti) |-> !memwe);
	
	ERR_MEMWE_IS_HIGH_FOR_MORE_THAN_ONE_CLK_CYCLE:
		`assert_clk(memwe |-> ##1 !memwe);
	
	ERR_MORE_THAN_ONE_BUS_DRIVER:
		`assert_clk($onehot0({en_pc, en_marmux, en_mdr, en_alu}));


	////////// Control Logic ///////////

	// State Register
	always_ff @ (posedge clk or posedge reset) begin
		if(reset == 1'b1) begin
			state <= start;
		end else begin
			state <= next_state;
		end
	end

	// Next State Logic
	always_comb begin
		unique case(state)
			start: next_state = fetch0;
			fetch0: next_state = fetch1;
			fetch1: next_state = fetch2;
			fetch2: next_state = decode;
			decode: if(opcode == op_ld) begin //load
				next_state = ex_ld1;
			end else if(opcode == op_ldr) begin //load register
				next_state = ex_ldr1;
			end else if(opcode == op_ldi) begin //load indirect
				next_state = ex_ldi1;
			end else if(opcode == op_st) begin //store
				next_state = ex_st1;
			end else if(opcode == op_sti) begin //store indirect
				next_state = ex_sti1;
			end else if(opcode == op_str) begin //store register
				next_state = ex_str1;
			end else if(opcode == op_trap) begin //store register
				next_state = ex_trap1;
			end else begin
				next_state = fetch0;
			end
			ex_ld1: next_state = ex_ld2;
			ex_st1: next_state = ex_st2;
			ex_ldi1: next_state = ex_ldi2;
			ex_ldi2: next_state = ex_ldi3;
			ex_ldi3: next_state = ex_ldi4;
			ex_sti1: next_state = ex_sti2;
			ex_sti2: next_state = ex_sti3;
			ex_sti3: next_state = ex_sti4;
			ex_ldr1: next_state = ex_ldr2;
			ex_str1: next_state = ex_str2;
			default: next_state = fetch0;
		endcase
	end

	// Control Assignments
	assign take_branch = ((ir_n & n_flag) || (ir_z & z_flag) || (ir_p & p_flag)) ? 1'b1 : 1'b0;
	assign alu_ctl = alu_ctl_next;
	assign dr = dr_next;
	assign sr1 = sr1_next;
	assign sr2 = sr2_next;
	assign sel_eab1 = sel_eab1_next;
	assign sel_eab2 = sel_eab2_next;
	assign sel_pc = sel_pc_next;
	assign en_alu = en_alu_next;
	assign regwe = regwe_next;
	assign flagwe = flagwe_next;
	assign en_marmux = en_marmux_next;
	assign sel_marmux = sel_marmux_next;
	assign en_pc = en_pc_next;
	assign load_pc = load_pc_next;
	assign load_ir = load_ir_next;
	assign load_mar = load_mar_next;
	assign load_mdr = load_mdr_next;
	assign sel_mdr = sel_mdr_next;
	assign memwe = memwe_next;
	assign en_mdr = en_mdr_next;
  
	// Output Logic
	always_comb begin
		alu_ctl_next = 2'b00;
		dr_next = ir[11:9];
		sr1_next = ir[8:6];
		sr2_next = ir[2:0];
		sel_eab1_next = 1'b0;
		sel_eab2_next = 2'b00;
		sel_pc_next = 2'b00;
		en_alu_next = 1'b0;
		regwe_next = 1'b0;
		flagwe_next = 1'b0;
		en_marmux_next = 1'b0;
		sel_marmux_next = 1'b0;
		en_pc_next = 1'b0;
		load_pc_next = 1'b0;
		load_ir_next = 1'b0;
		load_mar_next = 1'b0;
		load_mdr_next = 1'b0;
		sel_mdr_next = 1'b0;
		memwe_next = 1'b0;
		en_mdr_next = 1'b0;
		unique case(state)
			fetch0: begin
				en_pc_next = 1'b1;
				load_mar_next = 1'b1;
			end
			fetch1: begin
				load_pc_next = 1'b1;
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			fetch2: begin
				load_ir_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			decode: begin
				if(opcode == op_and) begin //and
					en_alu_next = 1'b1;
					regwe_next = 1'b1;
					flagwe_next = 1'b1;
					alu_ctl_next = 2'b10;
				end else if(opcode == op_add) begin //add
					en_alu_next = 1'b1;
					regwe_next = 1'b1;
					flagwe_next = 1'b1;
					alu_ctl_next = 2'b01;
				end else if(opcode == op_not) begin //not
					en_alu_next = 1'b1;
					regwe_next = 1'b1;
					flagwe_next = 1'b1;
					alu_ctl_next = 2'b11;
				end else if(opcode == op_br) begin //branch
					if(take_branch == 1'b1) begin
						load_pc_next = 1'b1;
						sel_pc_next = 2'b01;
						sel_eab2_next = 2'b10;
					end
				end else if((opcode == op_ld) || (opcode == op_st) || (opcode == op_ldi) || (opcode == op_sti)) begin //load, store, load indirect, store indirect
					en_marmux_next = 1'b1;
					load_mar_next = 1'b1;
					sel_eab2_next = 2'b10;
				end else if((opcode == op_str) || (opcode == op_ldr)) begin //store register
					en_marmux_next = 1'b1;
					load_mar_next = 1'b1;
					sel_eab2_next = 2'b01;
					sel_eab1_next = 1'b1;
				end else if(opcode == op_lea) begin //load effective address
					sel_eab2_next = 2'b10;
					regwe_next = 1'b1;
					flagwe_next = 1'b1;
					en_marmux_next = 1'b1;
				end else if(opcode == op_jmp) begin //jump, return
					load_pc_next = 1'b1;
					sel_pc_next = 2'b01;
					sel_eab1_next = 1'b1;
				end else if(opcode == op_jsr) begin //jump sub-routine
					if(ir[11] == 1'b1) begin // JSR
						regwe_next = 1'b1;
						en_pc_next = 1'b1;
						load_pc_next = 1'b1;
						sel_pc_next = 2'b01;
						dr_next = 3'b111;
						sel_eab2_next = 2'b11;
					end else begin // JSRR
						regwe_next = 1'b1;
						en_pc_next = 1'b1;
						load_pc_next = 1'b1;
						sel_pc_next = 2'b01;
						dr_next = 3'b111;
						sel_eab1_next = 1'b1;
					end
				end else if(opcode == op_trap) begin //trap
					regwe_next = 1'b1;
					dr_next = 3'b111;
					en_pc_next = 1'b1;
				end else if(opcode == op_rti || opcode == op_ioe) begin // RTI or IOE
					// Do nothing
				end else begin //any other opcode, will be ignored.
					//$display("%g\tInstruction 0x%h is not supported and will be ignored", $time, opcode);
				end
			end
			ex_ld1: begin
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			ex_ld2: begin
				regwe_next = 1'b1;
				flagwe_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			ex_st1: begin
				en_alu_next = 1'b1;
				load_mdr_next = 1'b1;
				sr1_next = ir[11:9];
			end
			ex_st2: begin
				memwe_next = 1'b1;
			end
			ex_ldi1: begin
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			ex_ldi2: begin
				load_mar_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			ex_ldi3: begin
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			ex_ldi4: begin
				regwe_next = 1'b1;
				flagwe_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			ex_sti1: begin
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			ex_sti2: begin
				load_mar_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			ex_sti3: begin
				en_alu_next = 1'b1;
				load_mdr_next = 1'b1;
				sr1_next = ir[11:9];
			end
			ex_sti4: begin
				memwe_next = 1'b1;
			end
			ex_str1: begin
				en_alu_next = 1'b1;
				load_mdr_next = 1'b1;
				sr1_next = ir[11:9];
			end
			ex_str2: begin
				memwe_next = 1'b1;
			end
			ex_ldr1: begin
				load_mdr_next = 1'b1;
				sel_mdr_next = 1'b1;
			end
			ex_ldr2: begin
				regwe_next = 1'b1;
				flagwe_next = 1'b1;
				en_mdr_next = 1'b1;
			end
			ex_trap1: begin
				en_marmux_next = 1'b1;
				sel_marmux_next = 1'b1;
				load_pc_next = 1'b1;
				sel_pc_next = 2'b10;
			end
			default: begin
				//assert none of the control signals
			end
		endcase
	end

	ammon_lc3_datapath my_dp (.*);
  
endmodule

module ammon_lc3_datapath(clk, reset, alu_ctl, sr1, sr2, dr, sel_pc, sel_eab2, en_alu, regwe, 
		en_marmux, sel_marmux, sel_eab1, en_pc, load_pc, load_ir, load_mar, load_mdr, 
		sel_mdr, memwe, en_mdr, flagwe, ir, n_flag, z_flag, p_flag, mar, mdr, memOut, pc, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out);

	//Ports
	input logic clk, reset, en_alu, regwe, en_marmux, sel_marmux, sel_eab1, en_pc, load_pc, 
		load_ir, load_mar, load_mdr, sel_mdr, memwe, en_mdr, flagwe;
	input logic [1:0] alu_ctl, sel_pc, sel_eab2;
	input logic [2:0] sr1, sr2, dr;
	output logic [15:0] ir;
	output logic n_flag, z_flag, p_flag;
	
	output logic [15:0] mar, mdr; 
	input logic [15:0] memOut; 
	output logic [15:0] pc;
	output logic [15:0] r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out;

	//Internal Signals
	wire [15:0] buss;
	logic [5:0] immediate;
	logic [15:0] rb, ra, eab_out, marmux_out;
	logic [15:0] eab_arg1, eab_arg2, eab_ir11, eab_ir9, eab_ir6;
	logic [15:0] alu_out;
	logic [15:0] arg_a, arg_b;
	logic [15:0] r0_in, r1_in, r2_in, r3_in, r4_in, r5_in, r6_in, r7_in;
	logic [15:0] marmux_ir8;
	logic [15:0] pc_next;
	logic [15:0] ir_next;

	////////////// Datapath Description ///////////////////

	assign immediate = ir[5:0];

	// EAB
	assign eab_ir11 = {{5{ir[10]}}, ir[10:0]};
	assign eab_ir9 = {{7{ir[8]}}, ir[8:0]};
	assign eab_ir6 = {{10{ir[5]}}, ir[5:0]};
	assign eab_arg1 = (sel_eab1 == 0) ? pc : ra;
	assign eab_arg2 = (sel_eab2 == 0) ? 0:
		(sel_eab2 == 2'b01) ? eab_ir6:
		(sel_eab2 == 2'b10) ? eab_ir9: eab_ir11;
	assign eab_out = eab_arg1 + eab_arg2;

	//MARMux
	assign marmux_ir8 = {{8{0}}, ir[7:0]};
	assign marmux_out = (sel_marmux == 0) ? eab_out : marmux_ir8;

	//PC
	assign pc_next = (sel_pc == 0) ? pc + 1:
		(sel_pc == 2'b01) ? eab_out: buss;
	always_ff @ (posedge clk or posedge reset) begin
		if(reset) begin
			pc <= 0;
		end else if(load_pc == 1'b1) begin
			pc <= pc_next;
		end else begin
			pc <= pc;
		end
	end

	//IR
	assign ir_next = (load_ir == 1'b1) ? buss : ir;
	always_ff @ (posedge clk or posedge reset) begin
		if(reset) begin
			ir <= 0;
		end else begin
			ir <= ir_next;
		end
	end

	//NZP Flags
	always_ff @ (posedge clk or posedge reset) begin
		if(reset) begin
			n_flag <= 1'b0;
			z_flag <= 1'b0;
			p_flag <= 1'b0;
		end else if(flagwe == 1'b1)
		begin
			if(buss[15] == 1'b1) begin
				n_flag <= 1'b1;
				z_flag <= 0;
				p_flag <= 0;
			end else if(buss == 0) begin
				n_flag <= 0;
				z_flag <= 1'b1;
				p_flag <= 0;
			end else begin
				n_flag <= 0;
				z_flag <= 0;
				p_flag <= 1'b1;
			end
		end else begin
			n_flag <= n_flag;
			z_flag <= z_flag;
			p_flag <= p_flag;
		end
	end

	//ALU
	assign arg_a = ra;
	assign arg_b = (immediate[5] == 1'b1) ? { {11{immediate[4]}}, immediate[4:0]} : rb;
	assign alu_out = (alu_ctl == 2'b00) ? arg_a:
		(alu_ctl == 2'b01) ? arg_a + arg_b:
		(alu_ctl == 2'b10) ? arg_a & arg_b: ~arg_a;

	//RegFile
	assign r0_in = (dr == 3'b000) ? buss : r0_out;
	assign r1_in = (dr == 3'b001) ? buss : r1_out;
	assign r2_in = (dr == 3'b010) ? buss : r2_out;
	assign r3_in = (dr == 3'b011) ? buss : r3_out;
	assign r4_in = (dr == 3'b100) ? buss : r4_out;
	assign r5_in = (dr == 3'b101) ? buss : r5_out;
	assign r6_in = (dr == 3'b110) ? buss : r6_out;
	assign r7_in = (dr == 3'b111) ? buss : r7_out;
	assign ra = (sr1 == 3'b000) ? r0_out:
		(sr1 == 3'b001) ? r1_out:
		(sr1 == 3'b010) ? r2_out:
		(sr1 == 3'b011) ? r3_out:
		(sr1 == 3'b100) ? r4_out:
		(sr1 == 3'b101) ? r5_out:
		(sr1 == 3'b110) ? r6_out: r7_out;
	assign rb = (sr2 == 3'b000) ? r0_out:
		(sr2 == 3'b001) ? r1_out:
		(sr2 == 3'b010) ? r2_out:
		(sr2 == 3'b011) ? r3_out:
		(sr2 == 3'b100) ? r4_out:
		(sr2 == 3'b101) ? r5_out:
		(sr2 == 3'b110) ? r6_out: r7_out;
	
	
	register_t r0 (r0_out, clk, r0_in, reset, regwe);
	register_t r1 (r1_out, clk, r1_in, reset, regwe);
	register_t r2 (r2_out, clk, r2_in, reset, regwe);
	register_t r3 (r3_out, clk, r3_in, reset, regwe);
	register_t r4 (r4_out, clk, r4_in, reset, regwe);
	register_t r5 (r5_out, clk, r5_in, reset, regwe);
	register_t r6 (r6_out, clk, r6_in, reset, regwe);
	register_t r7 (r7_out, clk, r7_in, reset, regwe);

	//Tri-state Bus Drivers
	tristate_driver pc_tsd (pc, buss, en_pc);
	tristate_driver marmux_tsd (marmux_out, buss, en_marmux);
	tristate_driver alu_tsd (alu_out, buss, en_alu);
	tristate_driver mdr_tsd (mdr, buss, en_mdr);

	//Memory
	//memory mymem(memOut, clk, reset, memwe, mdr, mar);
  
	MAR MAR0(buss, clk, reset, load_mar, mar);	  
	MDR MDR0(buss, memOut, sel_mdr, clk, reset, load_mdr, mdr);
  
endmodule

module MAR(Buss, clk, reset, ldMAR, MAR);
	input logic [15:0] Buss;
	input logic clk, reset, ldMAR;
	output logic [15:0] MAR;
 
	always @(posedge clk or posedge reset) 
		if (reset == 1'b1) 										   
			MAR = 0; 
		else if (ldMAR)
			MAR = Buss;								 					
endmodule				

module MDR(Buss, memOut, selMDR, clk, reset, ldMDR, MDR);
	input logic [15:0] Buss, memOut;
	input logic clk, reset, ldMDR, selMDR;
	output logic [15:0] MDR;

	always @(posedge clk or posedge reset) 
		if (reset == 1'b1) 										   
			MDR = 0; 
		else if (ldMDR)
			MDR = selMDR?memOut:Buss;		
endmodule

module register_t(dout, clk, din, reset, load);
	input logic clk, reset, load; 
	input logic [15:0] din;
	output logic [15:0] dout;

	always @(posedge clk)
		if (reset) dout <= 16'd0; 
		else if (load) dout <= din;
endmodule

module tristate_driver ( din, dout, ctrl );
	input logic [15:0] din;
	input logic ctrl;
	output logic [15:0] dout;
	assign dout = (ctrl)? din:(16'bZZZZZZZZZZZZZZZZ);
endmodule



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
	
	ERR_RESET_SHOULD_SET_ALL_REGISTERS_TO_ZERO:
		`assert_clk(reset |-> pc==16'h0 && ir==16'h0 && mdr==16'h0 && mar==16'h0 && n_flag==0 && z_flag==0 && p_flag==0);
	
	ERR_FLAGS_SHOULD_NOT_BE_HIGH_AT_THE_SAME_TIME:
		`assert_clk(!((n_flag && p_flag) || (n_flag && z_flag) || (z_flag && p_flag)));
	
	ERR_MEMWE_IS_HIGH_DURING_NON_STORE_INSTRUCTION:
		`assert_clk((Control.opCode != 4'b0011) && (Control.opCode != 4'b0111) && (Control.opCode != 4'b1011) |-> !memwe);
	
	ERR_MEMWE_IS_HIGH_FOR_MORE_THAN_ONE_CLK_CYCLE:
		`assert_clk(memwe |-> ##1 !memwe);
	
	ERR_MORE_THAN_ONE_BUS_DRIVER:
		`assert_clk($onehot0({enaPC, enaMARM, enaMDR, enaALU}));



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