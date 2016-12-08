package classes_final_project;
    `define SV_RAND_CHECK(r) \
do begin \
if(!(r)) begin \
$display("%s:%0d: Randomization failed \" %s \"", \
`__FILE__, `__LINE__, `"r`"); \
$finish; \
end \
end while(0)



///////////////////////////////////////////////////////////////////////////////////////////////////////
class Transaction(#ADD_WIDTH = 8);
  
   rand logic [3:0] opcode;
   rand logic [1:0] dst;
   rand logic [1:0] src;
   rand logic [ADD_WIDTH - 1:0] addr;
  
          constraint op_const {
                                opcode dist {
                                      NOP :=1,
                                      ADD :=1,
                                      SUB :=1,
                                      AND :=1,
                                      NOT :=1,
                                      RD := 1,
                                      WR := 1, 
                                      RDI :=1 
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

/////////////////////////////////////////////////////////////////////////////////
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


  
endpackage
