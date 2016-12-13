add wave -position insertpoint  \
sim:/lc3_top/tbdut_if/clk \
sim:/lc3_top/tbdut_if/reset \
sim:/lc3_top/my_lc3/pc \
sim:/lc3_top/my_lc3/my_dp/load_ir \
sim:/lc3_top/my_lc3/ir \
sim:/lc3_top/my_lc3/opcode \
sim:/lc3_top/my_lc3/mar \
sim:/lc3_top/tbdut_if/memOut \
sim:/lc3_top/my_lc3/my_dp/load_mdr \
sim:/lc3_top/my_lc3/my_dp/sel_mdr \
sim:/lc3_top/my_lc3/mdr \
sim:/lc3_top/my_lc3/my_dp/en_mdr \
sim:/lc3_top/my_lc3/state \
sim:/lc3_top/my_lc3/next_state 
run 2000ns
