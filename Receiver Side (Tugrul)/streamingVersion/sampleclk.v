`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:32:34 12/09/2016 
// Design Name: 
// Module Name:    sampleclk 
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
module sampleclk(
   input clock,
	input reset,
   output reg new_clock
   );
	
	
	reg [7:0] clock_counter = 0;
	 
	always @(posedge clock) begin
		if (reset) begin
			clock_counter <= 0;
			new_clock <= 0;
			end
		else if (clock_counter == 6) begin // 27MHz / 500kHz / 2 == 27 cycles
			new_clock <= ~new_clock;
			clock_counter <= 0;
		end
		else clock_counter <= clock_counter + 1;
	end


endmodule
