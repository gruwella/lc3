add wave -position insertpoint  \
sim:/lc3_top/tbdut_if_0/clk \
sim:/lc3_top/tbdut_if_0/reset \
sim:/lc3_top/my_lc3_0/pc \
sim:/lc3_top/my_lc3_0/my_dp/load_ir \
sim:/lc3_top/my_lc3_0/ir \
sim:/lc3_top/my_lc3_0/opcode \
sim:/lc3_top/my_lc3_0/mar \
sim:/lc3_top/tbdut_if_0/memOut \
sim:/lc3_top/my_lc3_0/mdr \
sim:/lc3_top/my_lc3_0/state \
sim:/lc3_top/my_lc3_0/next_state \
sim:/lc3_top/my_lc3_0/my_dp/r0_out \
sim:/lc3_top/my_lc3_0/my_dp/r1_out \
sim:/lc3_top/my_lc3_0/my_dp/r2_out \
sim:/lc3_top/my_lc3_0/my_dp/r3_out \
sim:/lc3_top/my_lc3_0/my_dp/r4_out \
sim:/lc3_top/my_lc3_0/my_dp/r5_out \
sim:/lc3_top/my_lc3_0/my_dp/r6_out \
sim:/lc3_top/my_lc3_0/my_dp/r7_out \
sim:/lc3_top/my_lc3_0/my_dp/buss
add wave /lc3_top/my_lc3_0/ERR_RESET_SHOULD_SET_ALL_REGISTERS_TO_ZERO /lc3_top/my_lc3_0/ERR_FLAGS_SHOULD_NOT_BE_HIGH_AT_THE_SAME_TIME /lc3_top/my_lc3_0/ERR_MEMWE_IS_HIGH_DURING_NON_STORE_INSTRUCTION /lc3_top/my_lc3_0/ERR_MEMWE_IS_HIGH_FOR_MORE_THAN_ONE_CLK_CYCLE /lc3_top/my_lc3_0/ERR_MORE_THAN_ONE_BUS_DRIVER
run -all 
