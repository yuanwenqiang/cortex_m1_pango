vlib work
vmap work work
vmap  usim usim
vlog +define+SIM_ON \
+incdir+../bench/src/m1_core \
+incdir+../bench/src/logical/cmsdk_apb_watchdog/verilog \
+incdir+../bench/src/logical/cmsdk_apb_i2c/verilog \
+incdir+../pnr/ipcore/DDR3/example_design/bench/mem \
-f file_list.f \
-l vlog.log
vsim -novopt -suppress 3486,3680,3781  m1_test -L work -L usim -l sim.log
do wave.do
run -all