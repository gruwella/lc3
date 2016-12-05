`include "assert_macros.sv"

module lc3_asserts (
	// Put wires to test here (all inputs)
	input logic clk);
	
	/*input logic [7:0] dout, 
	input logic       full, empty,
	input logic       write, read, clk, rst_n,
	input logic [7:0] din,
	input logic [3:0] wptr, rptr,
	input logic [4:0] cnt);*/
	
	// Put assert statements here
	
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
