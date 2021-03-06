#set the working dir, where all compiled verilog goes
vlib work

#compile all veriolog modules in mux.v to working dir 
#could also have multiple veriolog files
vlog datapathN.v
vlog ram.v
vlog ram_p.v

#load simulation using mux as the top level simulation module
vsim -L altera_mf_ver datapathN

#log all signals and add some signals to waveform window
log -r {/*}

# add wave {/*} would add all items in top level simulation module
add wave -r /*

force {clk} 0 0ns, 1 {0.001ns} -r 0.002ns

force {enableShift} 1
force {drawB} 1
force {drawC} 0 
force {reset} 1
force {enableX} 0
force {enableCountXC} 0
force {countUp} 0
force {countDown} 0
run 20ns


#reset


force {enableShift} 0
force {reset} 0
force {enableX} 1

run 40ns

force {drawB} 0
force {drawC} 1
force {enableCountXC} 1
force {countUp} 1
force {countDown} 0
run 10ns

force {drawB} 0
force {drawC} 1
force {enableCountXC} 1
force {countUp} 0
force {countDown} 1
run 10ns

force {drawB} 1
force {drawC} 0
force {enableCountXC} 1
force {countUp} 0
force {countDown} 1
run 20ns
