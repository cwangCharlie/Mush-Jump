
#set the working dir, where all compiled verilog goes
vlib work

#compile all veriolog modules in mux.v to working dir 
#could also have multiple veriolog files
vlog control.v

#load simulation using mux as the top level simulation module
vsim control

#log all signals and add some signals to waveform window
log {/*}

# add wave {/*} would add all items in top level simulation module
add wave -r /*

force {clk} 0 0ns, 1 {0.001ns} -r 0.002ns

force {startG} 0
force {doneP} 0
force {doneC} 0
force {userInput} 0
force {ground} 1
run 1000ns


force {startG} 1

force {doneP} 0

force {doneC} 0

force {userInput} 0

force {ground} 1

run 1000ns


force {startG} 1

force {doneP} 1

force {doneC} 0

force {userInput} 0

force {ground} 1

run 1000ns


force {startG} 1

force {doneP} 0

force {doneC} 1

force {userInput} 1

force {ground} 1

run 1000ns


force {startG} 1

force {doneP} 0

force {doneC} 1

force {userInput} 0

force {ground} 0

run 1000ns
