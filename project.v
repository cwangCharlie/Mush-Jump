module project
	(
		KEY, 
		CLOCK_50,
		
		HEX5, 
		HEX4, 
		HEX2, 
		HEX0,
		HEX1,
		HEX3,
		LEDR,
		SW,
		JUMP,

		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		
		
		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		FPGA_I2C_SDAT,

		// Outputs
		AUD_XCK,
		AUD_DACDAT,

		FPGA_I2C_SCLK
	);

	input CLOCK_50;
	input [3:0] KEY;
	output [9:0] LEDR;

	
	

	wire done;
	wire reset;
	wire shift;
	wire draw;
	wire enableX;
	output [6:0] HEX5, HEX4, HEX2, HEX0, HEX1, HEX3;

	output reg JUMP;
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire plot;
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(1'b1),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(plot),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "startscreen.mif";
		
	always@(*) begin
	JUMP = 1'b0;
	if(left_channel_audio_in > 32'd20000000)begin
		JUMP = 1'b1;
	end 
	
	end 
	
	assign LEDR[0] = JUMP;
	assign LEDR[9:2] = left_channel_audio_in [31:25];
	assign LEDR[1] = audio_in_available;

input		[7:0]	SW;

input				AUD_ADCDAT;


// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;



/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

reg [18:0] delay_cnt;
wire [18:0] delay;


assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in;
assign right_channel_audio_out	= right_channel_audio_in;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[2]),

	.clear_audio_in_memory		(SW[5]), // clear memory
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(SW[6]), //clear memory 
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[2])
);
 
	 
	 
	 wire [7:0] points;
	toplevel u0(.CLOCK_50(CLOCK_50),
					.start(~KEY[1]),
					.user(JUMP),
					.restart(~KEY[3]),
					.ChooseLevel(SW[0]),
					.score(points),
					.x(x),
					.y(y),
					.colour(colour),
					.plot(plot),
    );
	 
	 hex_decoder H2(
        .hex_digit(points[3:0]), 
        .segments(HEX0)
        );
        
    hex_decoder H4(
        .hex_digit(points[7:4]), 
        .segments(HEX1)
        );
	 hex_decoder H5(
        .hex_digit(left_channel_audio_out[11:8]), 
        .segments(HEX2)
        );
    hex_decoder H0(
        .hex_digit(left_channel_audio_out[15:12]), 
        .segments(HEX3)
        );
	hex_decoder H1(
        .hex_digit(left_channel_audio_out[19:16]), 
        .segments(HEX4)
        );
	hex_decoder H3(
        .hex_digit(left_channel_audio_out[23:20]), 
        .segments(HEX5)
        );

endmodule

module toplevel(input CLOCK_50,
					 input start,
					 input user,
					 input restart,
					 input ChooseLevel,
					 output [7:0] score,
					 output [7:0] x, 
					 output [6:0] y,
					 output [2:0] colour,
					 output plot
					 );
	
		//clock 25 signals
	 
    wire Enable;
    wire [24:0]resetRatio;
	 wire [24:0]resetRatio2;
	 reg [24:0]choice;
    
    assign resetRatio = 25'b0001111101011110000011111; //12.5million - 1
	assign resetRatio2 = 25'b0000011101011110000011111; 
	
	always@(*)begin
	if(ChooseLevel) begin
		choice = resetRatio2;
		
	end else begin
		choice = resetRatio;
	end 
	end
	 
	wire done;
	wire reset;
	wire shift;
	wire enableX;
	
	wire doneC;
	wire enableC;
	wire countUp;
	wire countDown;
	wire ground;
	wire top;
	wire checkConnect;
	wire checkContinousI;
	wire checkBlcok;
	wire checkB;
	wire enableCY;
	wire donecheck;
	wire checkCButtom;
   wire doneCB;
	wire enableXT;
	wire enableCB;
	wire doneS;
	wire enableXS;
	wire doneE;
	wire enableXE;
	wire drawS;
	wire drawE;
	
	
	wire SCORE;
	wire doneSC;
	wire enableSC;
	wire checkScore;
	
	

	
rateDivider R25(CLOCK_50, reset,choice,Enable);
	
datapath d1( .clk(CLOCK_50),
			 .reset(reset),
			 .enableShift(shift),
			 .doneP(done),
			 .enableX(enableX),
			 .colourO(colour),
			 .X(x),
			 .Y(y),
			 .Edisconnect(checkConnect),
			 .checkBlock(checkBlcok),
			 .enableCY(enableCY),
			 .checkCButtom(checkCButtom),
			 .doneCB(doneCB),
			 .enableSC(enableSC),
			 .checkScore(checkScore),
			 
			 .doneS(doneS),
			 .enableXS(enableXS),
			 .doneE(doneE),
			 .enableXE(enableXE),
			 .drawS(drawS),
			 .drawE(drawE),

			 .top(top),
			 .continousI(checkContinousI),
			 .countUp(countUp),
			 .countDown(countDown),
			 .drawB(drawB),
			 .drawC(drawC),
			 .doneC(doneC),
			 .enableCountXC(enableC),
			 .ground(ground),
			 .checkB(checkB),
			 .donecheck(donecheck),
			 .enableXT(enableXT),
			 .enableCB(enableCB),
			 .points(score),
			 .SCORE(SCORE),
			 .doneSC(doneSC)
			);
			
control c1( .clk(CLOCK_50),
			.clock25(Enable),
			.startG(start),
			.restart(restart),
			.enableXT(enableXT),
			.enableCB(enableCB),
			
			.SCORE(SCORE),
			.doneSC(doneSC),

			.doneS(doneS),
			.enableXS(enableXS),
			.doneE(doneE),
			.enableXE(enableXE),
			.drawS(drawS),
			.drawE(drawE),
			
			.doneC(doneC),
			.jump(user),
			.checkG(ground),
			.top(top),
			.continousI(checkContinousI),

			.drawC(drawC),
			.enableCountXC(enableC),
			.countUp(countUp),
			.countDown(countDown),
			.checkB(checkB),
			.donecheck(donecheck),


			.checkBlock(checkBlcok),
			.enableCY(enableCY),
			.doneP(done),
			.enableX(enableX),
			.resetn(reset),
			.drawB(drawB),
			.plot(plot),
			.Edisconnect(checkConnect),
			.enableI(shift),
			.checkCButtom(checkCButtom),
			.doneCB(doneCB),
			
			.enableSC(enableSC),
			.checkScore(checkScore)
			);


endmodule

module datapath(
	input enableShift,
	input clk,
	input reset,
	input enableX,
	input enableCountXC,
	input drawB,
	input drawC,
	input countUp,
	input countDown,
	input Edisconnect,
	input checkBlock,
	input enableCY,
	input enableXT,
	input enableCB,
	
	input enableXE,
	input enableXS,
	input drawE,
	input drawS,
	
	input enableSC,
	input checkScore,
	
	output [2:0] colourO,
	output reg doneP,
	output reg doneC,
	output reg top, //CHECK TOP
	output reg continousI,
	output [6:0] Y,
	output [7:0] X,
	output reg checkB,
	output reg donecheck,
	output reg ground,
	output reg checkCButtom,
	output reg doneCB,
	
	output reg doneE,
	output reg doneS,
	
	output [7:0] points,
	output reg SCORE,
	output reg doneSC
	);
	
	assign points = pointsF + pointsB;
	reg [3:0] countxC;
	reg [3:0] countyC;
	reg [10:0]xInitial;
	reg [7:0] yInitial;
	reg looped;
	wire [17:0] address;
	wire [17:0] addressCheck;
	wire [17:0] addressBC;
	wire [7:0] addressC;
	wire [6:0] ycoordC;
	wire [7:0] xcoordC;
	reg [7:0] countx;
	reg [6:0] county;
	reg [7:0] countxE;
	reg [6:0] countyE;
	reg [7:0] countxS;
	reg [6:0] countyS;
	wire [14:0] addressS;
	wire [14:0] addressE;
 
//counter for the background 
always@(posedge clk) begin
	//for background
if(reset||doneP) begin // counter through 
		county <= 0;
		countx <=0;
		doneP <= 0;
end

else if(drawB) begin
	
	if(county == 7'd119 && countx == 8'd159) begin 
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
end

//increment the x initial
always @(posedge clk) begin
//for background 
if (reset || looped) begin
		// reset 
		xInitial <= 0;
		looped<=1'b0;
end
else if(drawB) begin
	
        if(xInitial == 11'd1840) begin
		looped <= 1'b1;
	end
	else if(enableShift) begin // if in the increment state
		xInitial <= xInitial +1;
	end

end
//for the character
end
assign address =(xInitial+countx)+(county*(11'd2000));

wire [2:0] color;
newbackground u0(.address(address),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(color)
			);
			
//increment the y coordinate of the character
 always @(posedge clk) begin
if (reset) begin
		// reset 
		yInitial <= 7'd95;
		ground <= 1'b1;		//on the ground when its 1
		top <= 1'b0;
		continousI <= 1'b0;
		
end
else if(drawC) begin

	if((Edisconnect && ground == 1'b1) || (Edisconnect && ground == 1'b0))begin
		continousI <= ~continousI;
	end 
	else if(yInitial == 7'd0 && continousI == 1'b1) begin
	// reached the top of the screen
		top <= 1;
		ground <= 1'b0;
	end 
	else if(countDown && ground == 1'b0 && checkCButtom == 1'b0) begin // if in the increment state
		yInitial <= yInitial + 1;
		top <= 1'b0;
	end
	else if(countUp) begin // if in the increment state
		yInitial <= yInitial - 1;
		ground<= 1'b0;

	end
	else if(yInitial == 7'd95)begin
		ground <= 1;
	end 
	
end
end


//counter for the character address 

always@(posedge clk) begin
if(reset||doneC) begin // counter through 
		countyC <=4'b0;
		countxC <=4'b0;
		doneC <= 1'b0;
end
else if(drawC) begin

	if(countxC == 4'd14 && countyC == 4'd15) begin
		doneC <=1'b1;
	end
	else if(enableCountXC && countxC != 4'd14) begin
		countxC <= countxC + 1;
	end
	else if(countxC == 4'd14)begin 
		countyC <=countyC +1;
		countxC <= 4'b0;
	end
       
	
	
end
end

assign ycoordC = yInitial + countyC;
assign xcoordC = countxC;
assign addressC = (countyC * 4'd15)+countxC;

wire [2:0] colorC;
player u1(.address(addressC),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorC)
			);



selectColour s0(.clk(clk),
					.sel(drawB),
					.drawC(drawC),
					.drawE(drawE),
					.drawS(drawS),
					
					.color(color),
					.colorC(colorC),
					.colorE(colorE),
					.colorS(colorS),
					.colour(colourO),
					.xB(countx),
					.yB(county),
					.xE(countxE),
					.yE(countyE),
					.xS(countxS),
					.yS(countyS),
					.xC(xcoordC),
					.yC(ycoordC),
					.x(X),
					.y(Y)
					);

// check for the score collision on the Y-coordinates 
reg [4:0] countCYS;
wire [2:0] colorCOIN;

//INPUT SIGNALS
//enableSC,
//checkScore

//OUTPUT SIGNALS
//SCORE,
//doneSC,

// implementing the counter for check background 
always@(posedge clk) begin
	if(reset) begin // counter through 
		countCYS <= 5'b0;
		doneSC <= 1'b0;
		//points<= 8'b0;
		
	end
   else if(enableSC) begin
		   
			countCYS <= countCYS +1;
		end
		
	else if(checkScore) begin
		if(countCYS == 5'd16) begin
			countCYS <= 5'd0;
			doneSC <= 1'b1;
		end
		else if(countCYS != 5'd16) begin
			doneSC <= 1'b0;
		end
		if (colorCOIN != 3'b000) begin
			SCORE <= 1'b0;
		end
	   if(colorCOIN == 3'b110)begin
			SCORE <= 1'b1;
			//points <= points +1;
		end 
		
	
		
end
end	
				
assign addressCheckSCORE =(xInitial+5'd15)+((yInitial+5'd14)*(11'd2000));	
newbackground WC(.address(addressCheckSCORE),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorCOIN)
			);
					
					
	


	
reg [4:0] countCY;
wire [2:0] colorB;
reg [7:0] pointsF;

// implementing the counter for check background 
always@(posedge clk) begin
	if(reset) begin // counter through 
		countCY <= 5'b0;
		donecheck <= 1'b0;
		pointsF <= 8'b0;
		
	end
   else if(enableCY) begin
		   
			countCY <= countCY +1;
		end
		
	else if(checkBlock) begin
		if(countCY == 5'd16) begin
			countCY <= 5'd0;
			donecheck <= 1'b1;
		end
		else if(countCY != 5'd16) begin // modified 
			donecheck <= 1'b0;
		end
		if(colorB == 3'b000 && ground == 1'b0)begin
			pointsF <= (pointsF - 5'd2) + 1;
		end 
		if (colorB != 3'b010) begin
			checkB <= 1'b0;
		end
	   if(colorB == 3'b10)begin
			checkB <= 1'b1;
		end 
		
	
		
end
end	
				
assign addressCheck =(xInitial+5'd15)+((yInitial+5'd14)*(11'd2000));	
newbackground w1(.address(addressCheck),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorB)
			);
			
			
reg [4:0] countCX;
wire [2:0] colorCButtom;
reg [7:0] pointsB;

// implementing the counter for check background 
always@(posedge clk) begin
		
	if(reset) begin // counter through 
		countCX <= 5'b0;
		doneCB <= 1'b0;
		pointsB <= 8'b0;
	end
   else if(enableXT) begin
			countCX <= countCX +1;
		end
		
	else if(enableCB) begin
		if(countCX == 5'd16) begin
			countCX <= 5'd0;
			doneCB <= 1'b1;
		end
		else if(countCX != 5'd16) begin
			doneCB <= 1'b0;
		end
		if(colorB == 3'b000 && ground == 1'b0)begin
			pointsB <= (pointsB - 5'd16) + 1;
		end 
		
		if(colorCButtom != 3'b010) begin
			checkCButtom <= 1'b0;
		end 
	   else if(colorCButtom == 3'b010)begin
			checkCButtom <= 1'b1;
			
		end 
		
		
end
end	
				
assign addressBC =(xInitial+countCX)+((yInitial+5'd16)*(11'd2000));	
newbackground w2(.address(addressBC),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorCButtom)
			);
			

wire [2:0] colorS;
//counter for the startscreen 
always@(posedge clk) begin
	//for background
if(reset||doneS) begin // counter through 
		countyS <= 0;
		countxS <=0;
		doneS <= 0;
end

else if(drawS) begin
	
	if(countyS == 7'd119 && countxS == 8'd159) begin 
		doneS <=1'b1;
	end
	else if(countxS != 8'd159 && enableXS) begin
		countxS <= countxS + 1;
	end
	else if(countxS == 8'd159)begin 
		countyS <=countyS +1;
		countxS <= 0;
	end
 end 
end

assign addressS = (countxS + countyS*(8'd160));

startscreen s1 (.address(addressS),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorS)
			);

			
wire [2:0] colorE;
//counter for the endscreen 
always@(posedge clk) begin
	//for background
if(reset||doneE) begin // counter through 
		countyE <= 0;
		countxE <=0;
		doneE <= 0;
end

else if(drawE) begin
	
	if(countyE == 7'd119 && countxE == 8'd159) begin 
		doneE <=1'b1;
	end
	else if(countxE != 8'd159 && enableXE) begin
		countxE <= countxE + 1;
	end
	else if(countxE == 8'd159)begin 
		countyE <=countyE +1;
		countxE <= 0;
	end
 end 
end
assign addressE = (countxE + countyE*(8'd160));

gameover s2 (.address(addressE),
			.clock(clk),
			.data(3'd0),
			.wren(1'b0),
			.q(colorE)
			);			
					

endmodule 

module selectColour(clk,sel,drawC,drawE,drawS,color,colorC,colorE,colorS, xB, yB,xE,yE,xS,yS,xC,yC,colour, x, y);
	input sel, clk,drawC,drawE,drawS;
	input [7:0] xB; 
	input [6:0] yB;
	input [7:0] xC; 
	input [6:0] yC;
	input [7:0] xE; 
	input [6:0] yE;
	input [7:0] xS; 
	input [6:0] yS;
	input [2:0]color;
	input [2:0]colorC;
	input [2:0]colorE;
	input [2:0]colorS;
	output reg [2:0]colour;
	output reg [7:0] x; 
	output reg [6:0] y;

	always@(posedge clk) begin 
	if(sel) begin 
			x <= xB;
			y<= yB;
			colour <= color;
	end
	if(drawE) begin 
			x <= xE;
			y<= yE;
			colour <= colorE;
	end
	if(drawS) begin 
			x <= xS;
			y<= yS;
			colour <= colorS;
	end
	if(drawC) begin
		x<= xC;
		y<= yC;
		colour<=colorC;
	end 
end
endmodule 

module rateDivider(clock, reset, rateRatio, Enable);
    input clock , reset;//will be the 50mhz
    input [24:0] rateRatio; //stores max
    output Enable;
    reg[24:0]RateDivider = 25'b0; //counts down


    always@(posedge clock)
    begin
        if(reset) //when the reset is 1
            RateDivider <= rateRatio;
        else if(RateDivider == 25'b0000000000000000000000000) //load in max when counter has reached 0
            RateDivider <= rateRatio;
        else
            RateDivider <= RateDivider - 1; //count down if enable is on
    end

    assign Enable = (RateDivider == 25'b0000000000000000000000000)?1:0; //when counter reaches 0 then assign enable to be 1
endmodule

module control(
    input clk,
    input clock25,
    input startG,
	 input doneP,
	 input doneC,
	 input checkG,
	 input jump,
	 input top,
	 input continousI,
	 input checkB,
	 input donecheck,
	 input restart,
	 input checkCButtom,
	 input doneCB,
	 input doneS,
	 input doneE,
	 
	 input SCORE,
	 input doneSC,
	 
    output reg  resetn,
    output reg  drawB,
	 output reg enableX,
	 output reg plot,
    output reg  enableI,
	 output reg drawC,
	 output reg countUp,
	 output reg countDown,
	 output reg enableCountXC,
	 output reg Edisconnect,
	 output reg checkBlock,
	 output reg enableCY,
	 output reg enableXT,
	 output reg enableCB,
	 output reg enableXS,
	 output reg enableXE,
	 output reg drawE,
	 output reg drawS,
	 
	 output reg enableSC,
	 output reg checkScore //enable check signal 
	 
	 
    );

    reg [5:0] current_state, next_state; 
    
    localparam  
    			BEGIN_state        = 5'd0,
            RESET_state        = 5'd1,
            DRAW_BACKGROUND    = 5'd2,
				PRINT_SCREEN       = 5'd3,
				
				CHECK_TOP			 = 5'd4, // check if the character is on the top
				CHECK_CON			 = 5'd5, // check for continous input 
				CHECK_GROUND 		 = 5'd6,
				
				
				
				USER_INPUT			 = 5'd7,
				JUMP					 = 5'd8,
				NOT_JUMP 			 = 5'd9,
				GROUND 				 = 5'd10,
				IN_AIR				 = 5'd11,
				CHECK_BUTTOM		 = 5'd12,
				X_INCREMENT			 = 5'd13,
				
				DRAW_PLAYER 		 = 5'd14,
				INCREMENT_P			 = 5'd15,
				DROP_CHARACTER 	 = 5'd16,
				
				
				WAIT_FOR_COMPLETE  = 5'd17,
				CHECK_COLLISION	 = 5'd18,
				INCREMENT_CY		 = 5'd19,
				
				INCREMENT_BACKG	 = 5'd20,
				
				
				CHECK_FOR_FINISH	 = 5'd21,
				ENDGAME				 = 5'd22,
				GAMEOVER 			 = 5'd23,
				DRAW_GAMEOVER		 = 5'd24,
				INCREMENT_START 	 = 5'd25,
				INCREMENT_CHECKS 	 = 5'd26,
				CHECK_SCORE			 = 5'd27;
				
				
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                BEGIN_state: next_state =INCREMENT_START; // Loop in current state until value is input
                // // Loop in current state until go signal goes low
                	// Load the background
					INCREMENT_START: next_state = doneS ? RESET_state : BEGIN_state;
					RESET_state: next_state = startG ? DRAW_BACKGROUND : RESET_state; 	
						
					 DRAW_BACKGROUND: next_state = PRINT_SCREEN;
					 PRINT_SCREEN: next_state = doneP ? USER_INPUT : DRAW_BACKGROUND; // goes for incrmentation
					 
					 
					 USER_INPUT: next_state = jump ? CHECK_TOP : NOT_JUMP;
					 CHECK_TOP: next_state = top ? DRAW_PLAYER : CHECK_CON;
					 CHECK_CON: next_state = continousI ? JUMP : CHECK_GROUND;
					 CHECK_GROUND: next_state = checkG ? GROUND : IN_AIR;
					 JUMP: next_state = DRAW_PLAYER;
					 NOT_JUMP: next_state = checkG ? GROUND : IN_AIR;
					 GROUND : next_state = DRAW_PLAYER;
					 IN_AIR : next_state = CHECK_BUTTOM;
					 CHECK_BUTTOM: next_state = checkCButtom ? DRAW_PLAYER : X_INCREMENT; 
					 X_INCREMENT: next_state = doneCB ? DROP_CHARACTER : CHECK_BUTTOM;
					 DROP_CHARACTER: next_state = DRAW_PLAYER;
					 DRAW_PLAYER : next_state = INCREMENT_P;
					 INCREMENT_P : next_state = doneC ? WAIT_FOR_COMPLETE : DRAW_PLAYER;

					 
                WAIT_FOR_COMPLETE: next_state = clock25 ? CHECK_COLLISION : WAIT_FOR_COMPLETE;
					 CHECK_COLLISION: next_state = checkB ? GAMEOVER : INCREMENT_CY;
					 GAMEOVER: next_state = DRAW_GAMEOVER;
					 DRAW_GAMEOVER: next_state = doneE ? ENDGAME : GAMEOVER;
					 ENDGAME: next_state = restart ? BEGIN_state : ENDGAME;
					 INCREMENT_CY: next_state = donecheck ? CHECK_SCORE : CHECK_COLLISION;
					 CHECK_SCORE : next_state = SCORE ? INCREMENT_BACKG : INCREMENT_CHECKS;
					 INCREMENT_CHECKS : next_state = doneSC ? INCREMENT_BACKG : CHECK_SCORE;
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
			
			drawC = 1'b0;
			countUp = 1'b0;
			countDown = 1'b0;
			checkBlock = 1'b0;
			enableCountXC = 1'b0;
			Edisconnect = 1'b0;
			enableCY = 1'b0;
			enableCB = 1'b0;
			enableXT = 1'b0;
			drawS = 1'b0;
			enableXS = 1'b0;
			drawE = 1'b0;
			enableXE = 1'b0;
			
			enableSC = 1'b0;
			checkScore =1'b0;
			
			
        case (current_state)
         BEGIN_state: begin
			drawS = 1'b1;
			plot = 1'b1;
         end
			INCREMENT_START: begin
			drawS = 1'b1;
			enableXS = 1'b1;
			end
			RESET_state:begin
			resetn = 1'b1;
			end
			DRAW_BACKGROUND: begin
			drawB = 1'b1;
			plot =1'b1;
			end
			PRINT_SCREEN:begin
			drawB = 1'b1;
			enableX = 1'b1;
			end
			
			CHECK_TOP: begin
			
			end 
			CHECK_CON: begin
			
			end 
			CHECK_GROUND: begin
				
			end 
			USER_INPUT: begin
			drawB = 1'b1;
			end 
			
			JUMP: begin
			drawC = 1'b1;
			countUp = 1'b1;
			end 
			NOT_JUMP: begin
			drawC = 1'b1;
			Edisconnect = 1'b1;
			end
			GROUND:begin
			drawC = 1'b1;
			Edisconnect = 1'b1;
			end
			IN_AIR: begin
			
			end
			CHECK_BUTTOM: begin
			enableCB = 1'b1;
			end 
			X_INCREMENT: begin
			enableXT = 1'b1;
			enableCB = 1'b1;
			end 
			DROP_CHARACTER: begin
			drawC =1'b1;
			countDown = 1'b1;
			end 
			DRAW_PLAYER: begin
			drawC = 1'b1;
			plot = 1'b1;
			end
			INCREMENT_P: begin
			drawC = 1'b1;
			enableCountXC = 1'b1;
			end
			
			WAIT_FOR_COMPLETE: begin

			end
			CHECK_COLLISION: begin
			checkBlock =1'b1;
			end 
			INCREMENT_CY: begin
			enableCY = 1'b1;
			end 
			INCREMENT_CHECKS: begin
			//add
			enableSC = 1'b1;
			end 
			CHECK_SCORE: begin
			//add 
			checkScore =1'b1;
			end
			
			INCREMENT_BACKG: begin
			drawB = 1'b1;
			enableI = 1'b1;
			end
			
			GAMEOVER: begin
			drawE = 1'b1;
			plot = 1'b1;
			end
			DRAW_GAMEOVER: begin
			drawE = 1'b1;
			enableXE = 1'b1;
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

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule
