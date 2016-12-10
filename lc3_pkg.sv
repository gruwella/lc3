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
		bit [2:0] src1, src2, dst;
		bit [4:0] imm5_val;
		bit imm5_bit;
		bit [8:0] pc_offset9;
		bit [10:0] pc_offset11;
		bit jsr_bit;
		bit n_flag, z_flag, p_flag;
		bit [7:0] trapvec8;
		bit [5:0] offset6;
		rand bit [15:0] instruction;
		rand bit rst; //TODO add constraint
		rand integer rst_cycle;
		
		constraint rst_d { rst dist {0:=98, 1:=2};}
		constraint rst_c { rst_cycle >= 0; rst_cycle <= 8;}
		
		function new (bit [15:0] i=0);
			instruction = i;
			opcode = opcode_type'({28'h0, instruction[31:28]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = instruction[4:0];
			imm5_bit = instruction[5];
			pc_offset9 = instruction[8:0];
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = instruction[10:0];
			trapvec8 = instruction[7:0];
			offset6 = instruction[5:0];
		endfunction
		
		function void post_randomize();
			opcode = opcode_type'({28'h0, instruction[31:28]});
			src1 = instruction[8:6];
			src2 = instruction[2:0];
			dst = instruction[11:9];
			imm5_val = instruction[4:0];
			imm5_bit = instruction[5];
			pc_offset9 = instruction[8:0];
			n_flag = instruction[11];
			z_flag = instruction[10];
			p_flag = instruction[9];
			jsr_bit = n_flag;
			pc_offset11 = instruction[10:0];
			trapvec8 = instruction[7:0];
			offset6 = instruction[5:0];
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
	
	
	class State;
		logic [15:0] regs [8];
/* 		logic [15:0] dst_val;
		logic [2:0] dst_reg; */
		logic [15:0] pc;
		logic [15:0] mem_addr;
		logic [15:0] mem_val;
	endclass: State
	
	typedef class Scoreboard;
	
	class Driver #(ADDRESS_WIDTH=8);
		virtual test_if #(ADDRESS_WIDTH).TB2DUT tb_ports;
		virtual mem_if #(ADDRESS_WIDTH).TB2MEM dut_mem_ports;
		mailbox #(Transaction #(ADDRESS_WIDTH)) agt2drv;
		Transaction #(ADDRESS_WIDTH) t;
		Driver_cbs #(ADDRESS_WIDTH) cbs[$];
		integer count;
		State s;
		bit [15:0] my_memory [0:255];
		Scoreboard sb;
		
		function new (mailbox #(Transaction #(ADDRESS_WIDTH)) a2d, virtual test_if #(ADDRESS_WIDTH).TB2DUT tbd, virtual mem_if #(ADDRESS_WIDTH).TB2MEM dutm, Scoreboard sb);
			// Ports
			tb_ports = tbd;
			dut_mem_ports = dutm;
			
			// Mailbox
			agt2drv = a2d;
			
			// Instruction count
			count = 0;
			
			sb = this.sb;
		endfunction
		
		task connect_signals;
			forever begin
				dut_mem_ports.memwe <= tb_ports.memwe;
				dut_mem_ports.mdr <= tb_ports.mdr;
				dut_mem_ports.mar <= tb_ports.mar;
				@dut_mem_ports.clk; //TODO: make sure this is needed
			end
		endtask
		
		task run;
			// Reset the DUT
			tb_ports.reset <= 1;
			dut_mem_ports.reset <= 1;
			repeat (3) @tb_ports.cb;
			tb_ports.reset <= 0;
			dut_mem_ports.reset <= 0;
			tb_ports.memOut <= 0;
			//@tb_ports.cb;
			
			// Begin driving instructions
			while(1) begin
				s = new();
				agt2drv.get(t);
				foreach(cbs[i]) begin
					cbs[i].pre_tx(t);
				end
				@tb_ports.cb;
				if(t.rst == 1 && t.rst_cycle == 1) begin
					tb_ports.reset <= 1;
					dut_mem_ports.reset <= 1;
					@tb_ports.cb;
					tb_ports.reset <= 0;
					dut_mem_ports.reset <= 0;
					//@tb_ports.cb;
					continue;
				end
				@tb_ports.cb;
				if(t.rst == 1 && t.rst_cycle == 2) begin
					tb_ports.reset <= 1;
					dut_mem_ports.reset <= 1;
					@tb_ports.cb;
					tb_ports.reset <= 0;
					dut_mem_ports.reset <= 0;
					//@tb_ports.cb;
					continue;
				end
				@tb_ports.cb;
				if(t.rst == 1 && t.rst_cycle == 3) begin
					tb_ports.reset <= 1;
					dut_mem_ports.reset <= 1;
					@tb_ports.cb;
					tb_ports.reset <= 0;
					dut_mem_ports.reset <= 0;
					//@tb_ports.cb;
					continue;
				end
				tb_ports.memOut <= t.instruction;
				@tb_ports.cb;
				if(t.rst == 1 && t.rst_cycle == 4) begin
					tb_ports.reset <= 1;
					dut_mem_ports.reset <= 1;
					@tb_ports.cb;
					tb_ports.reset <= 0;
					dut_mem_ports.reset <= 0;
					//@tb_ports.cb;
					continue;
				end
				tb_ports.memOut <= dut_mem_ports.memOut;
				s.pc = tb_ports.pc;
				s.regs[0] = tb_ports.r0;
				s.regs[1] = tb_ports.r1;
				s.regs[2] = tb_ports.r2;
				s.regs[3] = tb_ports.r3;
				s.regs[4] = tb_ports.r4;
				s.regs[5] = tb_ports.r5;
				s.regs[6] = tb_ports.r6;
				s.regs[7] = tb_ports.r7;
				if((t.opcode == op_ldi) || (t.opcode == op_sti)) begin // 8 clk cycles
					if(t.opcode == op_sti) begin // store indirect
						s.mem_addr = my_memory[s.pc + t.pc_offset9];
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						my_memory[s.mem_addr] = s.mem_val;
					end else if(t.opcode == op_ldi) begin // load indirect
						s.regs[t.dst] = my_memory[my_memory[s.pc + t.pc_offset9]];
/* 						s.dst_val = my_memory[my_memory[s.pc + t.pc_offset9]];
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_ldr) begin // load register
						s.regs[t.dst] = my_memory[t.src1 + t.offset6];
/* 						s.dst_val = my_memory[s.src1 + t.offset6];
						s.dst_reg = t.dst; */
					end
					@tb_ports.cb;
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
					@tb_ports.cb;					
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
					@tb_ports.cb;					
					if(t.rst == 1 && t.rst_cycle == 7) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
					@tb_ports.cb;					
					if(t.rst == 1 && t.rst_cycle == 8) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
				end else if((t.opcode == op_ld) || (t.opcode == op_st) || (t.opcode == op_str) || (t.opcode == op_ldr)) begin // 6 clk cycles
					if(t.opcode == op_st) begin // store
						s.mem_addr = s.pc + t.pc_offset9;
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						my_memory[s.mem_addr] = s.mem_val;
					end else if(t.opcode == op_str) begin // store register
						s.mem_addr = t.src1 + t.offset6;
						s.mem_val = s.regs[t.dst]; // This is actually a source register
						my_memory[s.mem_addr] = s.mem_val;
					end else if(t.opcode == op_ld) begin // load
						s.regs[t.dst] = my_memory[s.pc + t.pc_offset9];
/* 						s.dst_val = my_memory[s.pc + t.pc_offset9];
						s.dst_reg = t.dst; */
					end
					@tb_ports.cb;
					if(t.rst == 1 && t.rst_cycle == 5) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
					@tb_ports.cb;					
					if(t.rst == 1 && t.rst_cycle == 6) begin
						tb_ports.reset <= 1;
						dut_mem_ports.reset <= 1;
						@tb_ports.cb;
						tb_ports.reset <= 0;
						dut_mem_ports.reset <= 0;
						//@tb_ports.cb;
						continue;
					end
				end else if((t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_not) || (t.opcode == op_br) || (t.opcode == op_jmp) 
							|| (t.opcode == op_jsr) || (t.opcode == op_trap) || (t.opcode == op_rti) || (t.opcode == op_ioe)) begin // 4 clk cycles
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
						s.regs[t.dst] = ~(s.regs[t.src1]);
/* 						s.dst_val = s.regs[t.dst];
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_br) begin //branch instruction
						if((t.n_flag & tb_ports.n_flag) || (t.z_flag & tb_ports.z_flag) || (t.p_flag & tb_ports.p_flag)) begin
							s.pc = s.pc + t.pc_offset9;
						end
					end else if(t.opcode == op_jmp) begin // jump
						s.pc = s.regs[t.src1];
					end else if(t.opcode == op_jsr) begin // jump sub routine
						s.regs[7] = s.pc;
						if(t.jsr_bit == 1) begin
							s.pc = s.pc + t.pc_offset11;
						end else begin
							s.pc = t.src1;
						end
					end else if(t.opcode == op_lea) begin // load effective address
						s.regs[t.dst] = s.pc + t.pc_offset9;
/* 						s.dst_val = s.pc + t.pc_offset9;
						s.dst_reg = t.dst; */
					end else if(t.opcode == op_trap) begin // trap
						s.regs[7] = s.pc;
						s.pc = t.trapvec8;
					end else if(t.opcode == op_rti || t.opcode == op_ioe) begin
						$display("Found opcode rti or ioe");
						// do nothing
					end else begin
						$display("Invalid opcode");
					end
				end else begin
					$display("%g\tUnknown Instruction %d Received!", $time, t.opcode);
				end
				foreach(cbs[i]) begin
					cbs[i].post_tx(t, count);
				end
				sb.save_expected(s);
				count++;
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
		virtual test_if#(ADDRESS_WIDTH).TB2DUT ports;
		//TODO: Monitor callbacks?
		State s;
		Scoreboard sb;
		Transaction #(ADDRESS_WIDTH) t;
		
		function new(input virtual test_if#(ADDRESS_WIDTH).TB2DUT p, Scoreboard scb);
			ports = p;
			sb = scb;
		endfunction
		
		task run();
			forever begin
				if(ports.reset == 1) continue;
				@ports.cb;
				if(ports.reset == 1) continue;
				@ports.cb;
				if(ports.reset == 1) continue;
				@ports.cb;
				if(ports.reset == 1) continue;
				@ports.cb;
				t = new($root.lc3_top.my_lc3.ir);
				s = new();
				if((t.opcode == op_ldi) || (t.opcode == op_sti)) begin // 8 clk cycles
					if(ports.reset == 1) continue;
					@ports.cb;
					if(ports.reset == 1) continue;
					@ports.cb;
					if(ports.reset == 1) continue;
					@ports.cb;
					if(ports.reset == 1) continue;
					@ports.cb;
					if(t.opcode == op_sti) begin
						s.mem_addr = $root.lc3_top.dut_mem.my_memory[s.pc + t.pc_offset9];
						s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end
				end else if((t.opcode == op_ld) || (t.opcode == op_st) || (t.opcode == op_str) || (t.opcode == op_ldr)) begin // 6 clk cycles
					if(ports.reset == 1) continue;
					@ports.cb;
					if(ports.reset == 1) continue;
					@ports.cb;
					if(t.opcode == op_st) begin
						s.mem_addr = s.pc + t.pc_offset9;
						s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end else if(t.opcode == op_str) begin
						s.mem_addr = t.src1 + t.offset6;
						s.mem_val = $root.lc3_top.dut_mem.my_memory[s.mem_addr];
					end
				end else if((t.opcode == op_add) || (t.opcode == op_and) || (t.opcode == op_not) || (t.opcode == op_br) || (t.opcode == op_jmp) 
							|| (t.opcode == op_jsr) || (t.opcode == op_trap) || (t.opcode == op_rti) || (t.opcode == op_ioe)) begin // 4 clk cycles
							
				end else begin
					//Illegal opcode
				end
				s.pc = tb_ports.pc;
				s.regs[0] = tb_ports.r0;
				s.regs[1] = tb_ports.r1;
				s.regs[2] = tb_ports.r2;
				s.regs[3] = tb_ports.r3;
				s.regs[4] = tb_ports.r4;
				s.regs[5] = tb_ports.r5;
				s.regs[6] = tb_ports.r6;
				s.regs[7] = tb_ports.r7;
				sb.check_actual(s);
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
		endfunction
		
		function void check_actual(State a);
			State e = expected.pop_front();
			if(a.pc != e.pc) begin
				cfg.errors++;
				$display("%g\tError: PC does not match!  Expected: 0x%h  Actual: 0x%h", $time, e.pc, a.pc);
			end else if(a.mem_addr != e.mem_addr) begin
				cfg.errors++;
				$display("%g\tError: Memory Address does not match!  Expected: 0x%h  Actual: 0x%h", $time, e.mem_addr, a.mem_addr);
			end else if(a.mem_val != e.mem_val) begin
				cfg.errors++;
				$display("%g\tError: Memory Value does not match!  Expected: 0x%h  Actual: 0x%h", $time, e.mem_val, a.mem_val);
			end else begin
				for(integer i = 0; i < 8; i++) begin
					if(a.regs[i] != e.regs[i]) begin
						cfg.errors++;
						$display("%g\tError: R%d does not match!  Expected: 0x%h  Actual: 0x%h", $time, i, e.regs[i], a.regs[i]);
					end
				end
			end
		endfunction
	endclass: Scoreboard
	
	
	class Environment #(ADDRESS_WIDTH=8);
		virtual test_if #(ADDRESS_WIDTH).TB2DUT tb_ports;
		virtual mem_if #(ADDRESS_WIDTH).TB2MEM dut_mem_ports;
		Generator #(ADDRESS_WIDTH) gen;
		Agent #(ADDRESS_WIDTH) agt;
		Driver #(ADDRESS_WIDTH) drv;
		Monitor #(ADDRESS_WIDTH) mon;
		Scoreboard sb;
		Config cfg;
		mailbox #(Transaction #(ADDRESS_WIDTH)) gen2agt, agt2drv;
		
		function new(virtual test_if #(ADDRESS_WIDTH).TB2DUT tbd, virtual mem_if #(ADDRESS_WIDTH).TB2MEM dutm);
			tb_ports = tbd;
			dut_mem_ports = dutm;
		endfunction
		
		function void build();
			//Initialize mailboxes
			gen2agt = new(1);
			agt2drv = new(1);
			
			//Initialize transactors
			sb = new(cfg);
			gen = new(gen2agt);
			agt = new(gen2agt, agt2drv);
			drv = new(agt2drv, tb_ports, dut_mem_ports, sb);
			mon = new(tb_ports, sb);
			cfg = new();
		endfunction
		
		task run();
			fork
				gen.run();
				agt.run();
				drv.run();
				drv.connect_signals();
				mon.run();
			join_any
		endtask
		
		function void wrap_up();
			$display("Instructions run = %d", drv.count);
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
	
	class TestBasic #(ADDRESS_WIDTH=16) extends Component;
		typedef Registry #(TestBasic, "TestBasic") type_id;
		Environment #(ADDRESS_WIDTH) env;
		Driver_cbs_coverage #(ADDRESS_WIDTH) dcv;
		virtual test_if #(ADDRESS_WIDTH).TB2DUT tbdut_if;
		virtual mem_if #(ADDRESS_WIDTH).TB2MEM dutmem_if;
		string name;
	   
		function new(string n);
			name = n;
			$display("%m");
			env = new();
		endfunction
		
		virtual task run_test();
			tbdut_if = $root.lc3_top.dut_mem_if.TB2DUT;
			dutmem_if = $root.lc3_top.dut_mem_if.TB2MEM;
			env = new(tbdut_if, dutmem_if);
			env.build();
			begin
				dcv = new();
				env.drv.cbs.push_back(dcv);
			end
			env.run();
			env.wrap_up();
		endtask
	endclass: TestBasic
	
endpackage