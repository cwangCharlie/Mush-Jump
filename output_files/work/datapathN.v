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
if(drawB && !drawC) begin
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
end

//increment the x initial
always @(posedge clk) begin
//for background 
if(drawB && !drawC) begin
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

end
//for the character
end
assign address =(xInitial+countx)+(county*(11'd2000));

//ram u0(.address(address),
//			.clock(clk),
//			.data(3'd0),
//			.wren(1'b0),
//			.q(color)
//			);
			
//increment the y coordinate of the character
 always @(posedge clk) begin
if(!drawB && drawC) begin
	if (reset) begin
		// reset 
		yInitial <= 1'b0;
		ground <= 1'b1;		//on the ground when its 1
	end
	else if(yInitial == 1'b0)begin
		ground <= 1;
	end 
	else if(countUp) begin // if in the increment state
		yInitial <= yInitial +1;
		ground<= 1'b0;

	end
	else if(countDown && ground == 1'b0) begin // if in the increment state
		yInitial <= yInitial - 1;
	end
	
end
end


//counter for the character address 

always@(posedge clk) begin
if(!drawB && drawC) begin
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
end

assign ycoordC = yInitial + 5'd24;
assign xcoordC = 5'd20;
assign addressC = (countyC * 4'd15)+countxC;
	
//ram_p u1(.address(addressC),
//			.clock(clk),
//			.data(3'd0),
//			.wren(1'b0),
//			.q(colorC)
//			);


//selectColour s0(drawB,color,colorC,colour);

endmodule