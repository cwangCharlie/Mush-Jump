#set the working dir, where all compiled verilog goes
vlib work

#compile all veriolog modules in mux.v to working dir 
#could also have multiple veriolog files
vlog datapathN.v

#load simulation using mux as the top level simulation module
vsim datapathN

#log all signals and add some signals to waveform window
log {/*}

# add wave {/*} would add all items in top level simulation module
add wave -r /*

force {clk} 0 0ns, 1 {0.001ns} -r 0.002ns

force {enableShift} 1
force {drawB} 1
force {reset} 1
run 30ns


#reset


force {enableShift} 1
force {reset} 0
force {enableX} 1

run 90000000ns
