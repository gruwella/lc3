// File Name:  ammon_lc3.sv
// This module is where all the control signals of the LC3 are generated.  It
// instantiates the datapath module and passes all control signals to it.

`timescale 1ns / 1ns
`default_nettype none

typedef enum {start, fetch0, fetch1, fetch2, decode, ex_ld1, ex_ld2, ex_st1, ex_st2, ex_ldi1, ex_ldi2, ex_ldi3, ex_ldi4, ex_sti1, ex_sti2, ex_sti3, ex_sti4, ex_str1, ex_str2, ex_ldr1, ex_ldr2, ex_trap1} state_type;
typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;

module ammon_lc3 (clk, reset, memwe, mdr, mar, memOut, pc, n_flag, z_flag, p_flag, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out);
	input logic clk, reset;
	
	output logic [15:0] mar, mdr; 
	input logic [15:0] memOut;
	output logic memwe;
	output logic n_flag, z_flag, p_flag;
	output logic [15:0] pc, r0_out, r1_out, r2_out, r3_out, r4_out, r5_out, r6_out, r7_out;

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
					$display("%g\tInstruction 0x%h is not supported and will be ignored", $time, opcode);
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

/* module memory(mdrOut, Buss, clk, reset, ldMAR, ldMDR, selMDR, memWE);
	input logic [15:0] Buss;
	input logic clk, reset, ldMAR, selMDR, ldMDR, memWE;
	output logic [15:0] mdrOut;				 
	wire [15:0] MARReg, memOut;
  
	MAR MAR0(Buss, clk, reset, ldMAR, MARReg);	  
	MDR MDR0(Buss, memOut, selMDR, clk, reset, ldMDR, mdrOut);

	logic [15:0] my_memory [0:255];
	assign memOut = my_memory[MARReg]; 	   
	always @(posedge clk) 
	begin		  	 
		if (memWE)
			my_memory[MARReg] <= mdrOut;
	end
endmodule */

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
