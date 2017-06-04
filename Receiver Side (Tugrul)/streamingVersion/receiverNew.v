`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:20:02 12/01/2016 
// Design Name: 
// Module Name:    transmitterNew 
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
module receiverNew(
    input clk,
    input reset,
	 input [70:0] seventy_one_bit_packet,  //Will include the crc tail. So we need make sure to strip that off
    input crc_done,
	 input crc_good,
	 input memory_full,
	 input receive_en,
	 output reg [1:0] state,
	 output reg [35:0] data_to_recorder,
	 output reg [35:0] data_to_zbt,
	 output reg [18:0] zbt_address,
	 output reg [18:0] resend_address,
	 output reg write_enable
    );
		

	parameter idle = 0;
	parameter receiveData = 1;

	
	//reg [5:0] counter; 
	//reg [4:0] crc_counter;
	reg [18:0] last_good_address=0;

	always @(posedge clk) begin
	
		if (reset) begin
			state <= idle;
		end
		
		case (state)  
		
			idle: begin 
				state <= idle;
				write_enable <= 0;
				if (receive_en) begin 
					state <= receiveData;
				end
			end 
				
			receiveData: begin
				
				
				if (crc_done) begin
					if (crc_good) begin 
						zbt_address <= seventy_one_bit_packet[70:52];
						data_to_zbt <= seventy_one_bit_packet[51:16];
						last_good_address <= seventy_one_bit_packet[70:52]; //Save this last beautiful address;
						write_enable<=1;
						state<= idle;

						end 
						
					if (~crc_good) begin //else 
						resend_address <= last_good_address + 1; //CHECK THIS LINE'S VALIDITY!
						state<= idle;
						end 
			end
			
			end
			
			default: state <= idle;

		endcase
		
	end 

endmodule
