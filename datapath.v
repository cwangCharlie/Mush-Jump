module datapath(
	input enableShift,
	input CLOCK_50,
	input reset,
	input drawS,
	input enableX,
	output reg doneP,
	output [2:0] color,
	output reg [7:0] countx,
	output reg [6:0] county
	);
	
	
	reg [10:0]xInitial;
	reg looped;
	wire [10:0] address;
 
//counter 
always@(posedge CLOCK_50) begin
	
if (drawS) begin

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
always @(posedge CLOCK_50) begin

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

assign address = (xInitial+countx)+(county*(11'd2000));

ram u0(.address(address),
			.clock(CLOCK_50),
			.data(3'd0),
			.wren(1'b0),
			.q(color)
			);
			
endmodule