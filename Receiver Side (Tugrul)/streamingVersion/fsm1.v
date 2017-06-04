`timescale 1ns / 1ps

module transmitting_fsm(input clk, input reset, input [35:0] data_from_zbt, input [35:0] data_from_recorder, 
				input [18:0] ac97_address, input ac97_en, input memory_full,
				output reg [18:0] zbt_address, output reg [35:0] data_to_zbt, output reg [35:0] data_to_recorder,
				output write_enable, output reg [35:0] to_transmitter_data, output reg [18:0] transmit_address,
				output reg transmit_en
				);
	
	// when ready is 1, we let the recorder module take control of address and data
	
	reg [5:0] counter;
	//assign write_enable = ready;
	
	always @(posedge clk) begin
		//zbt_address <= ready ? ac97_address : transmit_address; // ac97 writes to zbt when ready
		//data_to_zbt <= data_from_recorder;
		//to_transmitter_data <= data_from_zbt;

		
		if (ac97_en) begin
			counter <= counter + 1;
			transmit_en <= 1;
			to_transmitter_data <= data_from_recorder;
			transmit_address <= ac97_address;
		end
		else if (counter == 56) begin 
			transmit_en <= 0;
			counter<=0;
		end
		else counter <= counter + 1;
		
		//if (~ready && transmit_address != ac97_address) transmit_address <= transmit_address + 1;
	end
	

endmodule
