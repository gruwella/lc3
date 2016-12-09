package classes_final_project;
    `define SV_RAND_CHECK(r) \
do begin \
if(!(r)) begin \
$display("%s:%0d: Randomization failed \" %s \"", \
`__FILE__, `__LINE__, `"r`"); \
$finish; \
end \
end while(0)

 // Define the Opcodes
  parameter ADD 	     = 4'b0001;
  parameter AND 	     = 4'b0101;
  parameter NOT 	     = 4'b1001;
  parameter BR		      = 4'b0000;
  parameter JMP_RET   = 4'b1100;
  parameter JSR_JSRR  = 4'b0100;
  parameter LD        = 4'b0010;
  parameter LDI       = 4'b1010;
  parameter LDR       = 4'b0110;
  parameter LEA       = 4'b1110;
  parameter ST        = 4'b0011;
  parameter STI       = 4'b1011;
  parameter STR       = 4'b0111;
  parameter TRAP      = 4'b1111;
  parameter RTI       = 4'b1000;
  parameter reserved  = 4'b1101;



///////////////////////////////////////////////////////////////////////////////////////////////////////
class Transaction(#ADD_WIDTH = 8);
  
   rand logic [3:0] opcode;
   rand logic [1:0] dst;
   rand logic [1:0] src;
   rand logic [ADD_WIDTH - 1:0] addr;
   
   logic n,z,p;
   
  
          constraint op_const {
                                opcode dist {
                                      ADD :=1,
                                      AND :=1,
                                      NOT :=1,
                                      BR  :=1,
                                      JMP_RET := 1,
                                      JSR_JSRR := 1,
                                      LD   :=1,
                                      LDI  :=1,
                                      LDR  :=1,
                                      LEA  :=1,
                                      ST   :=1,
                                      STI  :=1,
                                      STR  :=1,
                                      TRAP :=1,
                                      RTI :=1
                                       
                                };
                              } 
  
  
  
  
endclass
  
////////////////////////////////////////////////////////////////////////////////////////////////////////  
  class Generator(#ADD_WIDTH = 8);
    Transaction(.ADD_WIDTH(#ADD_WIDTH)) tr;
    mailbox #(Transaction #(.ADD_WIDTH(#ADD_WIDTH))) mbx;
    
    function new( input mailbox #(Transaction #(.ADD_WIDTH(#ADD_WIDTH))) mbx);
              this.mbx = mbx;
    endfunction
    
    task run();
          forever begin
            tr = new();
            `SV_RAND_CHECK(tr.randomize);
            mbx.put(tr);
          end
    endtask
  
  task wrap_up();
    endtask
  
  endclass
////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Agent (#ADD_WIDTH = 8);
    mailbox #(Transaction #(.ADD_WIDTH(#ADD_WIDTH))) gen2agt,agt2drv;
    Transaction(.ADD_WIDTH(#ADD_WIDTH)) tr;
    
    function new( input mailbox #(Transaction #(.ADD_WIDTH(#ADD_WIDTH))) gen2drv,agt2drv);
              this.gen2agt = gen2agt;
              this.agt2drv = agt2drv;
    endfunction

task run();
          forever begin
            gen2agt.peek(tr);
            gen2agt.put(tr);
            agt2drv.get(tr);
          end
          
endtask

  task wrap_up();
    endtask
endclass
//////////////////////////////////////////////////////////////////////////////////////////

virtual class Driver_cbs;


                virtual task pre_tx(ref Transaction tr);
                            // By default, callback does nothing
                endtask


                virtual task post_tx(ref Transaction tr);
                          // By default, callback does nothing
                endtask


endclass


/////////////////////////////////////////////////////////////////////////////////

virtual class Driver_cbs_scoreboard extends Driver_cbs;


                virtual task pre_tx(ref Transaction tr);
                            // By default, callback does nothing
                endtask


                virtual task post_tx(ref Transaction tr);
                          // By default, callback does nothing
                endtask


endclass
////////////////////////////////////////////////////////////////////////////////////////////////
virtual class Driver_cbs_monitor extends Driver_cbs;


                virtual task pre_tx(ref Transaction tr);
                            // By default, callback does nothing
                endtask


                virtual task post_tx(ref Transaction tr);
                          // By default, callback does nothing
                endtask


endclass
////////////////////////////////////////////////////////////////////////////////////////////
class Enviroment;
 
 // Config cfg;
  Generator (.ADD_WIDTH(#ADD_WIDTH)) gen;
  Agent (.ADD_WIDTH(#ADD_WIDTH)) agt;
  //Driver(.ADD_WIDTH(#ADD_WIDTH)) drv;
  mailbox #(Transaction #(.ADD_WIDTH(#ADD_WIDTH))) gen2agt,agt2drv;
  
  
  function void build();
            gen2agt = new(1);
            agt2drv = new(1);
            gen = new(gen2agt);
            agt = new(gen2agt, agt2drv);
            drv = new(agt2drv);

  endfunction
  
  
  
  task run();
    //we need a function to reset the driver in the driver class 
        fork
            gen.run();
            agt.run();
            drv.run();
        join
  endtask

  task wrap_up();
          fork
                gen.wrap_up();
                agt.wrap_up();
                drv.wrap_up();

          join
  endtask  
  
  
  
endclass
//////////////////////////////////////////////////////////////////////////////////////

class golden_lc3;
  
  
  
  
  
endclass
  
endpackage
