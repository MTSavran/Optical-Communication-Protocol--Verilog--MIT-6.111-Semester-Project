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

//THIS CRC RECEIVES SERIAL DATA 
//IT TAKES IN 19_BIT_ADDRESS + 36_BIT_DATA + 16_BIT_CRC = 71BITS SERIALLY 
module receiver_crc(
    input clock,
    input start,
	 input serial_data_in,
    output reg done,
    output reg [15:0] r,
	 output reg [6:0] count,
	 output reg crc_good,
	 output reg receiver_flag
    );
	//reg [5:0] count  = 0; // initialize counter to zero //////////// uncomment later
	reg temp;
	reg state;
	parameter S_IDLE = 0;
	parameter S_CRC = 1;
	
	initial begin
		done <= 0;
		r[15:0] <= 16'hFFFF;
		count <= 0;
		crc_good <= 0;
		receiver_flag <= 0;
	end
	
	always @(posedge clock) begin
	
		case (state)
		
		S_IDLE: begin
		
			if (~start) begin
				done <= 0;
				r[15:0] <= 16'hFFFF;
				count <= 0;
				crc_good <= 0;
				receiver_flag <= 0;
			end
		
			else if (start) begin // CRC initilized, reset and begin calculation
				temp <= serial_data_in;
				r[15:0] <= 16'hFFFF;
				count <= 0;
				done <= 0;
				state <= S_CRC;
				crc_good <= 0;
				receiver_flag <= 0;
			end
			
		end
		
		S_CRC: begin
		
			
			temp <= serial_data_in;
			//packet <= {packet[53:0],1'b0};
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
			if (count == 70) receiver_flag <= 1;
			if (count == 71) begin
				done <= 1;
				state <= S_IDLE;
				crc_good <= (r == 0); 
				receiver_flag <= 0;
			end
			else state <= S_CRC;
		
		end
		
		default: state <= S_IDLE;
		
		endcase
		
	end

endmodule
