`timescale 1ns / 1ps

module deserial (input clk, input reset, input deserial_on, input serial_data_in, output reg [70:0] parallel_data_out);
	reg [7:0] count;
	reg [1:0] deserial_state;
	reg wait_count;
	
	parameter S_IDLE = 0;
	parameter S_WORKING = 1;
	parameter S_HOLDING = 2;
	
	initial begin
		parallel_data_out <= 0;
		count <= 0;
		deserial_state <= S_IDLE;
		wait_count <= 0;
	end
	
	always @ (posedge clk) begin
		
		if (reset) begin 
			parallel_data_out <= 0;
			count <= 0;
			deserial_state <= S_IDLE;
			wait_count <= 0;
			
		end
		
		case (deserial_state)
		
			S_IDLE: begin
				if (~deserial_on) begin ////////////////////////////
					parallel_data_out <= 0;
					count <= 0;
					wait_count <= 0;
				end
				else if (deserial_on) begin  
					parallel_data_out[70:0] <= {70'b0,serial_data_in};
					count <= count + 1; 
					deserial_state <= S_WORKING;
				end
			end
			
			S_WORKING: begin
				parallel_data_out[70:0] <= {parallel_data_out[69:0],serial_data_in};
				count <= count + 1; 
				if (count == 70) begin //Shifted in all 71 bits. Stop. //BASICALLY THIS SHOULD BE HANDLED IN TRANSMITTER MODULE
					count <= 0;
					deserial_state <= S_HOLDING;
				end 
			end
			
			S_HOLDING: begin
				wait_count <= wait_count + 1;
				if (wait_count == 1) deserial_state <= S_IDLE;
			end
			
			default: deserial_state <= S_IDLE;
		
		endcase
		
	end

endmodule
