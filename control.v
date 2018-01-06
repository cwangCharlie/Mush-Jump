module control(
    input clk,
    input clock25,
    input startG,
	 input doneP,

	input doneC,
	input userInput,
	input ground,

    output reg  resetn,
    output reg  drawB,
	 output reg enableX,
	 output reg plot,
    output reg  enableI,

output reg drawC,
output reg enableCountXC,
output reg countUp,
output reg countDown

    );

    reg [5:0] current_state, next_state; 
    
    localparam  
    			BEGIN_state        = 5'd0,
	          	RESET_state        = 5'd1,
	            	DRAW_BACKGROUND    = 5'd2,
			PRINT_SCREEN       = 5'd3,

			CHECK_GROUND 	= 5'd4,
			DRAW_PLAYER 	= 5'd5,
			INCREMENT_P 	= 5'd6,
			WAIT_FOR_USER 	= 5'd7,
			INCREMENT_Y_P 	= 5'd8,
			SHIFT_DOWN 	= 5'd9,

			WAIT_FOR_COMPLETE  = 5'd10,
			INCREMENT_BACKG	 = 5'd11;
				
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                BEGIN_state: next_state = startG ? RESET_state : BEGIN_state; // Loop in current state until value is input
                // // Loop in current state until go signal goes low
                RESET_state: next_state = DRAW_BACKGROUND; // Load the background
		DRAW_BACKGROUND: next_state = PRINT_SCREEN;
		PRINT_SCREEN: next_state = doneP ? CHECK_GROUND : DRAW_BACKGROUND; // goes for incrmentation
		


		CHECK_GROUND : next_state = ground ? DRAW_PLAYER : SHIFT_DOWN;
		DRAW_PLAYER : next_state = INCREMENT_P;
		INCREMENT_P : next_state = doneC ? WAIT_FOR_USER : DRAW_PLAYER;
		WAIT_FOR_USER: next_state = userInput ? INCREMENT_Y_P : WAIT_FOR_USER;

		INCREMENT_Y_P : next_state = WAIT_FOR_COMPLETE;
		SHIFT_DOWN : next_state = WAIT_FOR_COMPLETE;





                WAIT_FOR_COMPLETE: next_state = clock25 ? INCREMENT_BACKG : WAIT_FOR_COMPLETE;
		INCREMENT_BACKG: next_state = DRAW_BACKGROUND;
            default:     next_state = BEGIN_state;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
         resetn = 1'b0;
			drawB = 1'b0;
			enableX = 1'b0;
			enableI = 1'b0;
			plot = 1'b0;

			enableCountXC = 1'b0;
			countUp = 1'b0;
			countDown = 1'b0;
			drawC = 1'b0;
        case (current_state)
         BEGIN_state: begin
         end
			RESET_state:begin
			resetn = 1'b1;
			end
			DRAW_BACKGROUND: begin
			drawB = 1'b1;
			drawC = 1'b0;
			plot =1'b1;
			end
			PRINT_SCREEN:begin
			enableX = 1'b1;
			end
			CHECK_GROUND: begin
			plot = 1'b0;
			enableX= 1'b0;
			end

			DRAW_PLAYER: begin
			drawC = 1'b1;
			drawB = 1'b0;
			plot =1'b1;
			end

			INCREMENT_P: begin
			enableCountXC = 1'b1;
			end

			WAIT_FOR_USER: begin
			enableCountXC = 1'b0;
			end

			INCREMENT_Y_P: begin
			countUp = 1'b1;
			countDown = 1'b0;
			end

			SHIFT_DOWN : begin
			countUp = 1'b0;
			countDown = 1'b1;
			end

			WAIT_FOR_COMPLETE: begin
			drawB = 1'b0;
			drawC = 1'b0;
			plot = 1'b0;
			end

			INCREMENT_BACKG: begin
			enableI = 1'b1;
			end
        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
            current_state <= next_state;
    end // state_FFS
endmodule

module rateDivider(clock, reset, rateRatio, Enable);
    input clock , reset;//will be the 50mhz
    input [14:0] rateRatio; //stores max
    output Enable;
    reg[14:0]RateDivider = 15'b0; //counts down


    always@(posedge clock)
    begin
        if(reset) //when the reset is 1
            RateDivider <= rateRatio;
        else if(RateDivider == 15'b0000000000000000000000000) //load in max when counter has reached 0
            RateDivider <= rateRatio;
        else
            RateDivider <= RateDivider - 1; //count down if enable is on
    end

    assign Enable = (RateDivider == 15'b0000000000000000000000000)?1:0; //when counter reaches 0 then assign enable to be 1
endmodule