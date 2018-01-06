#set the working dir, where all compiled verilog goes
vlib work

#compile all veriolog modules in mux.v to working dir 
#could also have multiple veriolog files
vlog toplevelY.v

#load simulation using mux as the top level simulation module
vsim -L altera_mf_ver toplevelY

#log all signals and add some signals to waveform window
log {/*}

# add wave {/*} would add all items in top level simulation module
add wave -r /*

force {CLOCK_50} 0 0ns, 1 {0.001ns} -r 0.002ns

force {start} 0
run 30ns


force {start} 1
force {user} 1
run 3000ns

force {user} 0
run 3000ns
force {user} 1
run 3000ns
force {user} 0
run 6000ns
