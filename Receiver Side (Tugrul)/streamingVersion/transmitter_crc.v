`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:13:53 10/04/2016 
// Design Name: 
// Module Name:    CRC 
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


module transmitter_crc(
    input clock,
    input start,
	 input reset,
    input [35:0] tr_data,
	 input [18:0] tr_address,
    output reg done,
    output reg [15:0] r,
	 output reg [5:0] count 
    );
	
	
	reg temp;
	reg [54:0] packet;
	reg state;
	parameter S_IDLE = 0;
	parameter S_CRC = 1;
	initial state = 0;
	
	always @(posedge clock) begin
	
		if (reset) begin
			done <= 0;
			r[15:0] <= 16'hFFFF;
			count <= 0;
		end
	
		case (state)
		
		S_IDLE: begin
		
			if (~start) begin
				done <= 0;
				r[15:0] <= 16'hFFFF;
				count <= 0;
			end
		
			else if (start) begin // CRC initilized, reset and begin calculation
				temp <= tr_address[18]; //data[50];
				packet <= {tr_address[17:0],tr_data[35:0],1'b0}; // {data[49:0],1'b0};
				r[15:0] <= 16'hFFFF;
				count <= 0;
				done <= 0;
				state <= S_CRC;
			end
			
		end
		
		S_CRC: begin
		
			// begin when start is deasserted, break when done
			temp <= packet[54];
			packet <= {packet[53:0],1'b0};
			r[0]  <=  temp ^ r[15];
			r[1]  <=  r[0];
			r[2]  <=  r[1] ^ temp ^ r[15];
			r[3]  <=  r[2];
			r[4]  <=  r[3];
			r[5]  <=  r[4];
			r[6]  <=  r[5];
			r[7]  <=  r[6];
			r[8]  <=  r[7];
			r[9]  <=  r[8];
			r[10] <=  r[9];
			r[11] <=  r[10];
			r[12] <=  r[11];
			r[13] <=  r[12];
			r[14] <=  r[13];
			r[15] <=  r[14] ^ temp ^ r[15];
			
			count <= count + 1; // increment counter
			done <= (count == 54); // reached packet_length_th bit
			state <= (count == 54) ? S_IDLE : S_CRC;
		
		end
		
		default: state <= S_IDLE;
		
		endcase
		
	end

endmodule
