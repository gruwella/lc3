`include "assert_macros.sv"

typedef enum {op_add=32'h1, op_and=32'h5, op_not=32'h9, op_br=32'h0, op_jmp=32'hC, op_jsr=32'h4, op_ld=32'h2, op_ldr=32'h6, op_lea=32'hE, op_ldi=32'hA, op_st=32'h3, op_str=32'h7, op_sti=32'hB, op_rti=32'h8, op_ioe=32'hD, op_trap=32'hF} opcode_type;

module lc3_asserts (
	// Put wires to test here (all inputs) 
	input logic clk, reset, n_flag, z_flag, p_flag, memwe, en_pc, en_marmux, en_mdr, en_alu,
	input logic [15:0] pc, ir, mdr, mar, opcode);
	
	/*input logic [7:0] dout, 
	input logic       full, empty,
	input logic       write, read, clk, rst_n,
	input logic [7:0] din,
	input logic [3:0] wptr, rptr,
	input logic [4:0] cnt);*/
	
	// Put assert statements here
	
	ERR_RESET_SHOULD_SET_ALL_REGISTERS_TO_ZERO:
		`assert_clk(reset |-> pc==16'h0 && ir==16'h0 && mdr==16'h0 && mar==16'h0 && n_flag==0 && z_flag==0 && p_flag==0);
	
	ERR_FLAGS_SHOULD_NOT_BE_HIGH_AT_THE_SAME_TIME:
		`assert_clk(!((n_flag && p_flag) || (n_flag && z_flag) || (z_flag && p_flag)));
		
	ERR_FLAGS_UPDATED_WHEN_THEY_SHOULDNT_HAVE:
		`assert_clk_xrst((opcode != op_add) && (opcode != op_and) && (opcode != op_not) && (opcode != op_ld) && (opcode != op_ldr) && (opcode != op_ldi) |-> ##1 $stable(p_flag) && $stable(n_flag) && $stable(z_flag));
	
	ERR_MEMWE_IS_HIGH_DURING_NON_STORE_INSTRUCTION:
		`assert_clk((opcode != op_st) && (opcode != op_str) && (opcode != op_sti) |-> !memwe);
	
	ERR_MEMWE_IS_HIGH_FOR_MORE_THAN_ONE_CLK_CYCLE:
		`assert_clk(memwe |-> !memwe);
	
	ERR_MORE_THAN_ONE_BUS_DRIVER:
		`assert_clk($onehot0({en_pc, en_marmux, en_mdr, en_alu}));
	
	
	
	/*ERR_FIFO_RESET_SHOULD_CAUSE_EMPTY1_FULL0_RPTR0_WPTR0_CNT0:
		`assert_clk(!rst_n |-> (empty==1 && full==0 && rptr==0 && wptr==0 && cnt==0));
		
	ERR_FIFO_SHOULD_BE_FULL:
		`assert_clk(cnt>=16 |-> full);
		
	ERR_FIFO_SHOULD_NOT_BE_FULL:
		`assert_clk(cnt<16  |-> !full);
		
	ERR_FIFO_DID_NOT_GO_FULL: // one element more to store. we write without reading
		`assert_clk_xrst(cnt==15 && !read && write |-> ##1 full);
		
	ERR_FIFO_SHOULD_BE_EMPTY:
		`assert_clk(cnt==0 |-> empty);
		
	ERR_FIFO_SHOULD_NOT_BE_EMPTY:
		`assert_clk(cnt>0 |-> !empty);
		
	ERR_FIFO_DID_NOT_GO_EMPTY: // one element. we read it and do not write another one
		`assert_clk_xrst(cnt==1 && read && !write |-> ##1 empty);
	
	
	// Second set of assertions
	
	ERR_FIFO_FULL_WRITE_CAUSED_FULL_FLAG_TO_CHANGE:
		`assert_clk_xrst(full && write && !read |-> ##1 full);	
	ERR_FIFO_FULL_WRITE_CAUSED_WPTR_TO_CHANGE:
		`assert_clk_xrst(full && write && !read |-> ##1 $stable(wptr));	
	ERR_FIFO_EMPTY_READ_CAUSED_EMPTY_FLAG_TO_CHANGE:
		`assert_clk_xrst(empty && read && !write |-> ##1 empty);
	ERR_FIFO_EMPTY_READ_CAUSED_RPTR_TO_CHANGE:
		`assert_clk_xrst(empty && read && !write |-> ##1 $stable(rptr));	
	ERR_FIFO_WORD_COUNTER_IS_NEGATIVE:
		`assert_clk(cnt>=0);
	ERR_FIFO_READWRITE_ILLEGAL_FIFO_FULL_OR_EMPTY:
		`assert_clk_xrst(read && write |-> ##1 !full && !empty);*/
	
	
	
		
endmodule
