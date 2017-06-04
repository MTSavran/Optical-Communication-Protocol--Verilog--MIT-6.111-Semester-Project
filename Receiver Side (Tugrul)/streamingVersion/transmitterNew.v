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
module transmitterNew(
    input clk,
    input reset,
    input transmit_en,
    input [35:0] to_transmitter_data,
    input [18:0] transmit_address,
    input [15:0] crc_tail,
    input crc_done,
	 output reg crc_start,
	 output reg serial_data,
	 output reg [1:0] state,
	 output reg deserial_on,
	 output reg [5:0] counter,
	 output reg [5:0] crc_counter,
	 output reg [58:0] packet
    );
		
	reg [15:0] crc_checksum;
	parameter idle = 0;
	parameter transmitData = 1;
	parameter transmitCRC = 2;
	
	initial state = 0;
	
	//reg [5:0] counter; 
	//reg [4:0] crc_counter;
	

	always @(posedge clk) begin
	
		if (reset) begin
			state <= idle;
			crc_checksum <= 16'hFFFF;
			serial_data <= 0; 
			crc_counter <= 0;
			crc_start <= 0;
			counter <= 0;
			deserial_on <= 0;
			
		end
		
		case (state)  
		
			idle: begin 
				if (transmit_en) begin 
					crc_start <= 1;
					state <= transmitData;
					packet <= {4'b1001,transmit_address,to_transmitter_data};
				end
				else begin
					crc_checksum <= 16'hFFFF;
					serial_data <= 0; 
					crc_counter <= 0;
					crc_start <= 0;
					counter <= 0;
					deserial_on <= 0;
				end
			end 
				
			transmitData: begin
				
				if (counter!=59) begin 
					crc_start <= 0;
					deserial_on <= 1;
					serial_data <= packet[58];
					packet <= {packet[57:0],1'b0};
					counter <= counter + 1;
					if (crc_done) crc_checksum <= crc_tail;end 
				else begin 
					serial_data <= crc_checksum[15];
					crc_checksum <= {crc_checksum[14:0],1'b0};
					crc_counter <= crc_counter + 1;
					if (crc_counter == 16) begin
						crc_counter <= 0;
						counter <= 0;
						deserial_on <= 0;
						state <= idle;end	
				end 
						
			end
			
			default: state <= idle;

		endcase
		
	end 

endmodule
