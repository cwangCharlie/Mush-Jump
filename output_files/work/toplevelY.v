module toplevel(input CLOCK_50,
					 input start,
					 input user,
					 output [7:0] x, 
					 output [6:0] y,
					 output [2:0] colour,
					 output plot);
	
		//clock 25 signals
    wire clock25;
    wire [14:0]resetRatio;
    
    assign resetRatio = 15'b0101111101011110000011111; //12.5million - 1
	 
	wire done;
	wire reset;
	wire shift;
	wire draw;
	wire enableX;

	wire doneC;
	wire check;
	wire enableC;
	wire countUp;
	wire countDown;
	wire ground;

	wire [7:0] xB;
	wire [6:0] yB;
	wire [7:0] xCoord;
	wire [6:0] yCoord;

    reg[7:0] x1;
    reg[6:0] y1;
	
rateDivider R25(CLOCK_50, reset,resetRatio,clock25);
	
datapath d1( .clk(CLOCK_50),
			 .reset(reset),
			 .enableShift(shift),
			 .doneP(done),
			 .drawS(draw),
			 .enableX(enableX),
			 .colour(colour),
			 .countx(xB),
			 .county(yB),

			 .countUp(countUp),
			 .countDown(countDown),
			 .drawB(drawB),
			 .drawC(drawC),
			 .doneC(doneC),
			 .enableCountXC(enableC),
			 .xcoordC(xCoord),
			 .ycoordC(yCoord),
			 .ground(ground)
			);
			
control c1( .clk(CLOCK_50),
			.clock25(clock25),
			.startG(start),


			.doneC(doneC),
			.userInput(user),
			.ground(ground),

			.drawC(drawC),
			.enableCountXC(enableC),
			.countUp(countUp),
			.countDown(countDown),


			.doneP(done),
			.enableX(enableX),
			.resetn(reset),
			.drawB(draw),
			.plot(plot),
			.enableI(shift)
			);

always@(posedge CLOCK_50) begin
	if(drawB) begin
		x1<=xB;
		y1<=yB;
	end
	if(drawC) begin
		x1<=xCoord;
		y1<=yCoord;

	end
end

assign x = x1;
assign y = y1;

endmodule


module datapath(
	input enableShift,
	input clk,
	input reset,
	input drawS,
	input enableX,
	input enableCountXC,
	input drawB,
	input drawC,
	input countUp,
	input countDown,

	output reg doneP,
	output reg doneC,
	output [2:0] colour,
	output [6:0] ycoordC,
	output [7:0] xcoordC,
	output reg [7:0] countx,
	output reg [6:0] county,
	output reg ground
	);
	

	reg [3:0] countxC;
	reg [3:0] countyC;
	reg [10:0]xInitial;
	reg [5:0] yInitial;
	reg looped;
	wire [17:0] address;
 
//counter for the background 
always@(posedge clk) begin
	

//for background

	if(reset||doneP) begin // counter through 
		county <= 0;
		countx <=0;
		doneP <= 0;
	end
	else if(county == 7'd119 && countx == 8'd159) begin
		doneP <=1'b1;
	end
	else if(countx != 8'd159 && enableX) begin
		countx <= countx + 1;
	end
	else if(countx == 8'd159)begin 
		county <=county +1;
		countx <= 0;
	end
 
end

//increment the x initial
always @(posedge clk) begin
//for background 
	if (reset || looped) begin
		// reset 
		xInitial <= 0;
		looped<=1'b0;
	end
	else if(xInitial == 11'd1840) begin
		looped <= 1'b1;
	end
	else if(enableShift) begin // if in the increment state
		xInitial <= xInitial +1;
	end


//for the character
end
assign address =(xInitial+countx)+(county*(11'd2000));

ram u0(.address(address),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(color)
			);
			
//increment the y coordinate of the character
 always @(posedge clk) begin

	if (reset) begin
		// reset 
		yInitial <= 0;
		ground <= 1;		//on the ground when its 1
	end
	else if(yInitial == 0)begin
		ground <= 1;
	end 
	else if(countUp) begin // if in the increment state
		yInitial <= yInitial +1;
		ground<= 0;

	end
	else if(countDown && ground == 0) begin // if in the increment state
		yInitial <= yInitial - 1;
	end
	
	
end


//counter for the character address 

always@(posedge clk) begin
	
	if(reset||doneC) begin // counter through 
		countyC <=0;
		countxC <=0;
		doneC <= 0;
	end
	else if(countyC == 4'd15 && countxC == 4'd14) begin
		doneC <=1'b1;
	end
	else if(countxC != 4'd14 && enableCountXC) begin
		countxC <= countxC + 1;
	end
	else if(countxC == 4'd15)begin 
		countyC <=countyC +1;
		countxC <= 0;
	end

end

assign ycoordC = yInitial + 5'd24;
assign xcoordC = 5'd20;
assign addressC = (countyC * 4'd15)+countxC;
	
ram_p u1(.address(addressC),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorC)
			);


selectColour s0(drawB,color,colorC,colour);

endmodule

module selectColour(sel,color,colorC,colour);
	input sel;
	input [2:0]color;
	input [2:0]colorC;
	output [2:0]colour;
    reg[2:0] x;

	always@(*) begin
	case(sel)
		2'b00:x = color;
		2'b01:x = colorC;
		default: x = 2'b00;
	endcase
end
assign colour =x;
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