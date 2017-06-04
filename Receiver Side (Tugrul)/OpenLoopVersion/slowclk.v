`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:44:33 12/10/2016 
// Design Name: 
// Module Name:    slowclk 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module slowclk(
   input clock,
	input reset,
   output reg new_clock
   );
	
	
	reg [21:0] clock_counter = 0;
	 
	always @(posedge clock) begin
		if (reset) begin
			clock_counter <= 0;
			new_clock <= 0;
			end
		else if (clock_counter == 2700000) begin // really fuckin slow
			new_clock <= ~new_clock;
			clock_counter <= 0;
		end
		else clock_counter <= clock_counter + 1;
	end


endmodule