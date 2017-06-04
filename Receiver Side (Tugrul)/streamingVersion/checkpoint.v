`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:15:20 11/30/2016 
// Design Name: 
// Module Name:    checkpoint 
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
module checkpoint(
	 input sample_clk,
	 input reset,
	 input signal,
	 input crc_done,
	 output reg crc_start,
	 output reg deserial_on =0,
	 output reg [1:0] countHeader =0,
	 output reg [1:0] state,
	 output reg clk_state,
	 output reg [2:0] parallel_header,
	 output reg [1:0] bit_counter,
	 output reg [1:0] nth_sample,
	 output reg transmission_clk
    );
	 
	  
	 reg start_parallel;
	 reg [8:0] process_counter;
	 reg clock_divide_count;
	 
	 parameter S_IDLE = 0;
	 parameter S_CHECK_HEADER = 1;
	 parameter S_PROCESSING_DATA = 2;
	 
	 parameter S_WAIT = 0;
	 parameter S_CLK = 1;
	 
	 initial begin
		transmission_clk <= 0;
		start_parallel <= 0; 
		crc_start <= 0;
		countHeader <= 0;
		parallel_header <= 0;
		deserial_on <= 0;
		process_counter <= 0;
		bit_counter <= 0;
		nth_sample <= 0;
		state <= 0;
		clock_divide_count <= 0;
	 end
	 
	 always @(posedge sample_clk) begin 
	 
		if (reset) begin
			start_parallel <= 0; 
			crc_start <= 0;
			countHeader <= 0;
			parallel_header <= 0;
			deserial_on <= 0;
			process_counter <= 0;
			bit_counter <= 0;
			nth_sample <= 0;
			state <= S_IDLE;
		end
		
		else case (state) 
			S_IDLE: begin 
				if (~signal) begin ///////////////////
					start_parallel <= 0; 
					crc_start <= 0;
					countHeader <= 0;
					parallel_header <=0;
					process_counter <= 0;
					bit_counter <= 0;
					nth_sample <= 0;
				end
				else if (signal) begin
					state <= S_CHECK_HEADER;
					bit_counter <= bit_counter + 1;
					nth_sample <= nth_sample + 1;
				end
			end
			
			S_CHECK_HEADER: begin
					
					nth_sample <= nth_sample + 1; 
					if (countHeader == 3 && nth_sample == 3) begin
						if (bit_counter >= 3 && parallel_header == 3'b100) begin // got final 1, header correctly processed
							state <= S_PROCESSING_DATA;
							crc_start <= 1;
							deserial_on <= 1;
							countHeader <= 0;
							nth_sample <= 0;
							bit_counter <= 0;
							parallel_header <=0;
							start_parallel <= 1; ////////////////////////
						end
						else begin
							state <= S_IDLE; // incorrect header, reset to idle state
							start_parallel <= 0; 
							crc_start <= 0;
							countHeader <= 0;
							parallel_header <=0;
							process_counter <= 0;
							bit_counter <= 0;
							nth_sample <= 0;
						end
						
					end
					
					else if (nth_sample == 3) begin // got to 4th sample of bit
						countHeader <= countHeader + 1; // next full bit
						nth_sample <= 0; // reset sample back to zero
						
						if (bit_counter >= 3) begin
							parallel_header[2:0] <= {parallel_header[1:0],1'b1}; // received 1
							bit_counter <= 0;
						end
						
						else if (bit_counter <= 1) begin
							parallel_header[2:0] <= {parallel_header[1:0],1'b0}; // received 0
							bit_counter <= 0;
						end
					end	
						
					else bit_counter <= bit_counter + signal;
						

			end
					
			S_PROCESSING_DATA: begin
				process_counter <= process_counter + 1;
				if (process_counter == 3) begin
					crc_start <= 0;
					deserial_on <= 0;
					start_parallel <= 0;
				end
				if (process_counter == 289) state <= S_IDLE;
			end //state 
			
			default: state <= S_IDLE;
		
		endcase 
		

		// start new case for sub fsm
		case (clk_state)
		 
		S_WAIT: begin
			if (countHeader == 3 && nth_sample == 1 && signal == 1) begin
				transmission_clk <= 1;
				clock_divide_count <= 0;
				clk_state <= S_CLK;
			end
			else begin
				transmission_clk <= 0;
				clock_divide_count <= 0;
			end
		end
		
		S_CLK: begin
			clock_divide_count <= clock_divide_count + 1;
			if (clock_divide_count == 1) transmission_clk <= ~transmission_clk;
			if (process_counter == 289) clk_state <= S_WAIT;
			
		end
		
		default: clk_state <= S_WAIT;
		
		endcase
	
	 end //always
	 
endmodule
