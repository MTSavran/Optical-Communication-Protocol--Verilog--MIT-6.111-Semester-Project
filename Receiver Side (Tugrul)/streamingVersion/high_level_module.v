`timescale 1ns / 1ps
`default_nettype none

///////////////////////////////////////////////////////////////////////////////
//
// Switch Debounce Module
//
///////////////////////////////////////////////////////////////////////////////

module debounce (
  input wire reset, clock, noisy,
  output reg clean
);
  reg [18:0] count;
  reg new;

  always @(posedge clock)
    if (reset) begin
      count <= 0;
      new <= noisy;
      clean <= noisy;
    end
    else if (noisy != new) begin
      // noisy input changed, restart the .01 sec clock
      new <= noisy;
      count <= 0;
    end
    else if (count == 270000)
      // noisy input stable for .01 secs, pass it along!
      clean <= new;
    else
      // waiting for .01 sec to pass
      count <= count+1;

endmodule

///////////////////////////////////////////////////////////////////////////////
//
// bi-directional monaural interface to AC97
//
///////////////////////////////////////////////////////////////////////////////

module labkit_audio (
  input wire clock_27mhz_in,
  input wire reset,
  input wire [4:0] volume,
  output wire [11:0] audio_in_data, //
  input wire [11:0] audio_out_data, //
  output wire ready,
  output reg audio_reset_b,   // ac97 interface signals
  output wire ac97_sdata_out,
  input wire ac97_sdata_in,
  output wire ac97_synch,
  input wire ac97_bit_clock
);

  wire [7:0] command_address;
  wire [15:0] command_data;
  wire command_valid;
  wire [19:0] left_in_data, right_in_data;
  wire [19:0] left_out_data, right_out_data;

  // wait a little before enabling the AC97 codec
  reg [9:0] reset_count;
  always @(posedge clock_27mhz_in) begin
    if (reset) begin
      audio_reset_b = 1'b0;
      reset_count = 0;
    end else if (reset_count == 1023)
      audio_reset_b = 1'b1;
    else
      reset_count = reset_count+1;
  end

  wire ac97_ready;
  ac97 ac97(.ready(ac97_ready),
            .command_address(command_address),
            .command_data(command_data),
            .command_valid(command_valid),
            .left_data(left_out_data), .left_valid(1'b1),
            .right_data(right_out_data), .right_valid(1'b1),
            .left_in_data(left_in_data), .right_in_data(right_in_data),
            .ac97_sdata_out(ac97_sdata_out),
            .ac97_sdata_in(ac97_sdata_in),
            .ac97_synch(ac97_synch),
            .ac97_bit_clock(ac97_bit_clock));

  // ready: one cycle pulse synchronous with clock_27mhz_in
  reg [2:0] ready_sync;
  always @ (posedge clock_27mhz_in) ready_sync <= {ready_sync[1:0], ac97_ready};
  assign ready = ready_sync[1] & ~ready_sync[2];

  reg [11:0] out_data; //
  always @ (posedge clock_27mhz_in)
    if (ready) out_data <= audio_out_data;
  assign audio_in_data = left_in_data[19:8]; //
  assign left_out_data = {out_data, 8'b00000000}; //
  assign right_out_data = left_out_data;

  // generate repeating sequence of read/writes to AC97 registers
  ac97commands cmds(.clock(clock_27mhz_in), .ready(ready),
                    .command_address(command_address),
                    .command_data(command_data),
                    .command_valid(command_valid),
                    .volume(volume),
                    .source(3'b000));     // mic
endmodule


/////////////////////////////////////////////////////////////////////////////////
////
//// 6.111 FPGA Labkit -- Template Toplevel Module
////
//// For Labkit Revision 004
//// Created: October 31, 2004, from revision 003 file
//// Author: Nathan Ickes, 6.111 staff
////
/////////////////////////////////////////////////////////////////////////////////

module labkit   (beep, audio_reset_b, ac97_sdata_out, ac97_sdata_in, ac97_synch,
	       ac97_bit_clock,
	       
	       vga_out_red, vga_out_green, vga_out_blue, vga_out_sync_b,
	       vga_out_blank_b, vga_out_pixel_clock, vga_out_hsync,
	       vga_out_vsync,

	       tv_out_ycrcb, tv_out_reset_b, tv_out_clock, tv_out_i2c_clock,
	       tv_out_i2c_data, tv_out_pal_ntsc, tv_out_hsync_b,
	       tv_out_vsync_b, tv_out_blank_b, tv_out_subcar_reset,

	       tv_in_ycrcb, tv_in_data_valid, tv_in_line_clock1,
	       tv_in_line_clock2, tv_in_aef, tv_in_hff, tv_in_aff,
	       tv_in_i2c_clock, tv_in_i2c_data, tv_in_fifo_read,
	       tv_in_fifo_clock, tv_in_iso, tv_in_reset_b, tv_in_clock,

	       ram0_data, ram0_address, ram0_adv_ld, ram0_clk, ram0_cen_b,
	       ram0_ce_b, ram0_oe_b, ram0_we_b, ram0_bwe_b, 

	       ram1_data, ram1_address, ram1_adv_ld, ram1_clk, ram1_cen_b,
	       ram1_ce_b, ram1_oe_b, ram1_we_b, ram1_bwe_b,

	       clock_feedback_out, clock_feedback_in,

	       flash_data, flash_address, flash_ce_b, flash_oe_b, flash_we_b,
	       flash_reset_b, flash_sts, flash_byte_b,

	       rs232_txd, rs232_rxd, rs232_rts, rs232_cts,

	       mouse_clock, mouse_data, keyboard_clock, keyboard_data,

	       clock_27mhz, clock1, clock2,

	       disp_blank, disp_data_out, disp_clock, disp_rs, disp_ce_b,
	       disp_reset_b, disp_data_in,

	       button0, button1, button2, button3, button_enter, button_right,
	       button_left, button_down, button_up,

	       switch,

	       led,
	       
	       user1, user2, user3, user4,
	       
	       daughtercard,

	       systemace_data, systemace_address, systemace_ce_b,
	       systemace_we_b, systemace_oe_b, systemace_irq, systemace_mpbrdy,
	       
	       analyzer1_data, analyzer1_clock,
 	       analyzer2_data, analyzer2_clock,
 	       analyzer3_data, analyzer3_clock,
 	       analyzer4_data, analyzer4_clock);

   output beep, audio_reset_b, ac97_synch, ac97_sdata_out;
   input  ac97_bit_clock, ac97_sdata_in;
   
   output [7:0] vga_out_red, vga_out_green, vga_out_blue;
   output vga_out_sync_b, vga_out_blank_b, vga_out_pixel_clock,
	  vga_out_hsync, vga_out_vsync;

   output [9:0] tv_out_ycrcb;
   output tv_out_reset_b, tv_out_clock, tv_out_i2c_clock, tv_out_i2c_data,
	  tv_out_pal_ntsc, tv_out_hsync_b, tv_out_vsync_b, tv_out_blank_b,
	  tv_out_subcar_reset;
   
   input  [19:0] tv_in_ycrcb;
   input  tv_in_data_valid, tv_in_line_clock1, tv_in_line_clock2, tv_in_aef,
	  tv_in_hff, tv_in_aff;
   output tv_in_i2c_clock, tv_in_fifo_read, tv_in_fifo_clock, tv_in_iso,
	  tv_in_reset_b, tv_in_clock;
   inout  tv_in_i2c_data;
        
   inout  [35:0] ram0_data;
   output [18:0] ram0_address;
   output ram0_adv_ld, ram0_clk, ram0_cen_b, ram0_ce_b, ram0_oe_b, ram0_we_b;
   output [3:0] ram0_bwe_b;
   
   inout  [35:0] ram1_data;
   output [18:0] ram1_address;
   output ram1_adv_ld, ram1_clk, ram1_cen_b, ram1_ce_b, ram1_oe_b, ram1_we_b;
   output [3:0] ram1_bwe_b;

   input  clock_feedback_in;
   output clock_feedback_out;
   
   inout  [15:0] flash_data;
   output [23:0] flash_address;
   output flash_ce_b, flash_oe_b, flash_we_b, flash_reset_b, flash_byte_b;
   input  flash_sts;
   
   output rs232_txd, rs232_rts;
   input  rs232_rxd, rs232_cts;

   input  mouse_clock, mouse_data, keyboard_clock, keyboard_data;

   input  clock_27mhz, clock1, clock2;

   output disp_blank, disp_clock, disp_rs, disp_ce_b, disp_reset_b;  
   input  disp_data_in;
   output  disp_data_out;
   
   input  button0, button1, button2, button3, button_enter, button_right,
	  button_left, button_down, button_up;
   input  [7:0] switch;
   output [7:0] led;

   inout [31:0] user1, user2, user3, user4;
   
   inout [43:0] daughtercard;

   inout  [15:0] systemace_data;
   output [6:0]  systemace_address;
   output systemace_ce_b, systemace_we_b, systemace_oe_b;
   input  systemace_irq, systemace_mpbrdy;

   output [15:0] analyzer1_data, analyzer2_data, analyzer3_data, 
		 analyzer4_data;
   output analyzer1_clock, analyzer2_clock, analyzer3_clock, analyzer4_clock;

   ////////////////////////////////////////////////////////////////////////////
   //
   // I/O Assignments
   //
   ////////////////////////////////////////////////////////////////////////////
   

   // Audio Input and Output
   assign beep = 1'b0;
   //labkit assign audio_reset_b = 1'b0;
   //labkit assign ac97_synch = 1'b0;
   //labkit assign ac97_sdata_out = 1'b0;
   // ac97_sdata_in is an input

   // VGA Output
   assign vga_out_red = 10'h0;
   assign vga_out_green = 10'h0;
   assign vga_out_blue = 10'h0;
   assign vga_out_sync_b = 1'b1;
   assign vga_out_blank_b = 1'b1;
   assign vga_out_pixel_clock = 1'b0;
   assign vga_out_hsync = 1'b0;
   assign vga_out_vsync = 1'b0;

   // Video Output
   assign tv_out_ycrcb = 10'h0;
   assign tv_out_reset_b = 1'b0;
   assign tv_out_clock = 1'b0;
   assign tv_out_i2c_clock = 1'b0;
   assign tv_out_i2c_data = 1'b0;
   assign tv_out_pal_ntsc = 1'b0;
   assign tv_out_hsync_b = 1'b1;
   assign tv_out_vsync_b = 1'b1;
   assign tv_out_blank_b = 1'b1;
   assign tv_out_subcar_reset = 1'b0;
   
   // Video Input
   assign tv_in_i2c_clock = 1'b0;
   assign tv_in_fifo_read = 1'b0;
   assign tv_in_fifo_clock = 1'b0;
   assign tv_in_iso = 1'b0;
   assign tv_in_reset_b = 1'b0;
   assign tv_in_clock = 1'b0;
   assign tv_in_i2c_data = 1'bZ;
   // tv_in_ycrcb, tv_in_data_valid, tv_in_line_clock1, tv_in_line_clock2, 
   // tv_in_aef, tv_in_hff, and tv_in_aff are inputs
   
   // SRAMs
	
	//Enable RAM0
	assign ram0_ce_b = 1'b0;
	assign ram0_oe_b = 1'b0;
	assign ram0_bwe_b = 4'h0;
	assign ram0_adv_ld = 1'b0;
//	Stop assigning values to important RAM I/O
// assign ram0_data = 36'hZ;        // audio data
// assign ram0_address = 19'h0;     // ram address
//	assign ram0_clk = clock_27mhz_in;
//	assign ram0_cen_b = 1'b0;
// assign ram0_we_b = 1'b1          // ram write control active low

	
	//Disable RAM1
   assign ram1_data = 36'hZ; 
   assign ram1_address = 19'h0;
   assign ram1_adv_ld = 1'b0;
   assign ram1_clk = 1'b0;
   assign ram1_cen_b = 1'b1;
   assign ram1_ce_b = 1'b1;
   assign ram1_oe_b = 1'b1;
   assign ram1_we_b = 1'b1;
   assign ram1_bwe_b = 4'hF;
	//assign clock_feedback_out = 1'b0;
   //clock_feedback_in is an input
	
   // Flash ROM
   assign flash_data = 16'hZ;
   assign flash_address = 24'h0;
   assign flash_ce_b = 1'b1;
   assign flash_oe_b = 1'b1;
   assign flash_we_b = 1'b1;
   assign flash_reset_b = 1'b0;
   assign flash_byte_b = 1'b1;
   // flash_sts is an input

   // RS-232 Interface
   assign rs232_txd = 1'b1;
   assign rs232_rts = 1'b1;
   // rs232_rxd and rs232_cts are inputs

   // PS/2 Ports
   // mouse_clock, mouse_data, keyboard_clock, and keyboard_data are inputs

   // LED Displays
	/*
   assign disp_blank = 1'b1;
   assign disp_clock = 1'b0;
   assign disp_rs = 1'b0;
   assign disp_ce_b = 1'b1;
   assign disp_reset_b = 1'b0;
   assign disp_data_out = 1'b0;
   // disp_data_in is an input
	*/

   // Buttons, Switches, and Individual LEDs
   //labkit assign led = 8'hFF;
   // button0, button1, button2, button3, button_enter, button_right,
   // button_left, button_down, button_up, and switches are inputs

   // User I/Os
   //assign user1 = 32'hZ;
   assign user2 = 32'hZ;
   assign user3 = 32'hZ;
   assign user4 = 32'hZ;

   // Daughtercard Connectors
   assign daughtercard = 44'hZ;

   // SystemACE Microprocessor Port
   assign systemace_data = 16'hZ;
   assign systemace_address = 7'h0;
   assign systemace_ce_b = 1'b1;
   assign systemace_we_b = 1'b1;
   assign systemace_oe_b = 1'b1;
   // systemace_irq and systemace_mpbrdy are inputs

   // Logic Analyzer
   //labkit assign analyzer1_data = 16'h0;
   //labkit assign analyzer1_clock = 1'b1;
   assign analyzer2_data = 16'h0;
   assign analyzer2_clock = 1'b1;
   //labkit assign analyzer3_data = 16'h0;
   //labkit assign analyzer3_clock = 1'b1;
   assign analyzer4_data = 16'h0;
   assign analyzer4_clock = 1'b1;

   ////////////////////////////////////////////////////////////////////////////
   //
   // Reset Generation
   //
   // A shift register primitive is used to generate an active-high reset
   // signal that remains high for 16 clock cycles after configuration finishes
   // and the FPGA's internal clocks begin toggling.
   //
   ////////////////////////////////////////////////////////////////////////////
   wire reset;
	wire reset_in;
	wire clock_27mhz_in;
   SRL16 #(.INIT(16'hFFFF)) reset_sr(.D(reset_in), .CLK(clock_27mhz_in), .Q(reset),
                                     .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1));
												 
   wire signed [11:0] from_ac97_data, to_ac97_data;
   wire ready;

   // allow user to adjust volume
   wire vup,vdown;
   reg old_vup,old_vdown;
   debounce bup(.reset(reset),.clock(clock_27mhz_in),.noisy(~button_up),.clean(vup));
   debounce bdown(.reset(reset),.clock(clock_27mhz_in),.noisy(~button_down),.clean(vdown));

	reg [4:0] volume;
   always @ (posedge clock_27mhz_in) begin
     if (reset) volume <= 5'd8;
     else begin
	if (vup & ~old_vup & volume != 5'd31) volume <= volume+1;       
	if (vdown & ~old_vdown & volume != 5'd0) volume <= volume-1;       
     end
     old_vup <= vup;
     old_vdown <= vdown;
   end

   // AC97 driver
   labkit_audio a(clock_27mhz_in, reset, volume, from_ac97_data, to_ac97_data, ready,
	       audio_reset_b, ac97_sdata_out, ac97_sdata_in,
	       ac97_synch, ac97_bit_clock);

	// reset
	debounce reset1(.clock(clock_27mhz_in),.noisy(~button0),.clean(reset_in));

	// SW 0 up plays tone (for debug)
   wire play_tone;
   debounce sw0(.reset(reset),.clock(clock_27mhz_in),.noisy(switch[0]),.clean(play_tone));

   // push ENTER button to record, release to playback
   wire playback;
   debounce benter(.reset(reset),.clock(clock_27mhz_in),.noisy(button_enter),.clean(playback));
	
	// push button3 to send data
	wire signal;
	wire send;
	wire receive_signal;
	debounce b1(.reset(reset),.clock(clock_27mhz_in),.noisy(~button3),.clean(send));
	assign user3[0] = signal; /////////////  ~signal 
	assign receive_signal = user3[5];
	
   // light up LEDs when recording, show volume during playback.
   // led is active low
   assign led = playback ? ~{play_tone, send, 1'b0, volume} : ~{play_tone, 7'hFF};


	// display module for debugging
   reg [63:0] dispdata;
   display_16hex hexdisp1(reset, clock_27mhz_in, dispdata,
			  disp_blank, disp_clock, disp_rs, disp_ce_b,
			  disp_reset_b, disp_data_out);
	
	
   // record module
	wire [35:0] recorder_to_fsm_data, fsm_to_recorder_data;
	wire [35:0] zbt_to_fsm, fsm_to_zbt;
	wire [35:0] transmitter_data;
	wire cen, we, mem_full;
	wire [18:0] ac97_address, zbt_address, transmit_address;
	wire audio_state;
	wire ac97_en;
	wire transmit_en;
   recorder r(.clock(clock_27mhz_in), .reset(reset), .ready(ready),
              .playback(1'b1), .play_tone(play_tone),
				  .dout(fsm_to_recorder_data),
				  .audio_state(audio_state),
				  .from_ac97_data(from_ac97_data),
				  .to_ac97_data(to_ac97_data),
				  .mem_address(ac97_address),
				  .ac97_en(ac97_en),
				  //.we(we)	
				  .din(recorder_to_fsm_data));			

	// zbt ram bank 0			 
	wire ram0_clk_not_used;
	zbt_6111 zbt1(.clk(clock_27mhz_in), .cen(1'b1), .we(we), .addr(zbt_address), .write_data(fsm_to_zbt), .read_data(zbt_to_fsm),
		  .ram_clk(ram0_clk_not_used), .ram_we_b(ram0_we_b), .ram_address(ram0_address), .ram_data(ram0_data), .ram_cen_b(ram0_cen_b));
	
	// XKCD Transmission Clock
	wire clock_125khz;
	wire clock_500khz;
	wire clock_5hz;
	wire clock_20hz;
	
	//transmitclk 	clk125kHz(.clock(clock_27mhz_in), .new_clock(clock_125khz), .reset(reset));
	sampleclk 		clk500kHz(.clock(clock_27mhz_in), .new_clock(clock_500khz), .reset(reset));
	//slowclk 			clk5hz(.clock(clock_27mhz_in), .new_clock(clock_5hz), .reset(reset));
	//slowsampleclk 	clk20hz(.clock(clock_27mhz_in), .new_clock(clock_20hz), .reset(reset));

	
	
	////////////////////// 5hz land
	
	//reg clock;
	//reg transmitter_clock;
	wire [35:0] tr_data;
	wire [18:0] tr_address; 
	//wire transmit_en; 
	
	//assign transmit_en = send;
	assign tr_data = 36'hDEADBEEF1;
	assign tr_address = 19'b1011010010110100101;

	// Outputs
	wire done;
	wire [15:0] trans_r;
	//wire [5:0] count;
	//wire serial_data;
	wire start;
	wire [58:0] transmit_packet;
	//wire [1:0] state;
	//wire [74:0] seventy_five_bit_packet;
	//reg [5:0] counter;
	//reg [5:0] crc_counter;
	
	
	//wire turn_on_parallelizer;
	
	

   transmitter_crc trans_crc1 (  // crc outgoing data
      .clock(clock_5hz), 
		.reset(reset),
      .start(start), 
      .tr_data(transmitter_data),
		.tr_address(transmit_address),
      .done(done), 
		//.count(count),
      .r(trans_r)
   );
	
		
	transmitterNew transNew1 (  // take data, address, crc, create packet and transmit
		.clk(clock_5hz), 
		.reset(reset), 
		.transmit_en(transmit_en), 
		.to_transmitter_data(transmitter_data), 
		.transmit_address(transmit_address), 
		.crc_tail(trans_r),
		.crc_done(done), 
		.crc_start(start), 
		.serial_data(signal),
		.packet(transmit_packet)
		//.state(state),
		//.deserial_on(turn_on_parallelizer),
		//.counter(counter),
		//.crc_counter(crc_counter)
		); 
 	
////////////////////////////////////////////
	//reg sample_clk;
	//assign signal = serial_data;

	// Outputs
	wire rec_crc_start;
	wire deserial_on;
	wire [1:0] countHeader;
	//wire [1:0] state;
	wire [2:0] parallel_header;
	wire [1:0] bit_counter;
	wire [1:0] nth_sample;
	wire transmission_clk;
	wire clk_state;
	wire [70:0] parallel_data_out;
	wire crc_good;
	wire crc_done;
	wire [15:0] rec_r;
	wire [1:0] chk_state;
	
	wire [35:0] data_to_recorder;
	wire [35:0] data_to_zbt;
	wire [18:0] receiver_zbt_address;
	wire [18:0] resend_address;
	wire write_enable;
	wire receiver_flag;

	// Instantiate the Unit Under Test (UUT)
	checkpoint chkpt (
		.sample_clk(clock_500khz), 
		.reset(reset), 
		.signal(receive_signal), 
		.crc_done(crc_done), 
		.crc_start(rec_crc_start), 
		.countHeader(countHeader),
		.deserial_on(deserial_on),
		.state(chk_state),
		.parallel_header(parallel_header),
		.bit_counter(bit_counter),
		.nth_sample(nth_sample),
		.transmission_clk(transmission_clk),
		.clk_state(clk_state)
	);
	
	deserial ds1(
		.clk(transmission_clk), 
		.reset(reset), 
		.deserial_on(deserial_on), 
		.serial_data_in(receive_signal), 
		.parallel_data_out(parallel_data_out) 
		);
	
	receiver_crc rc1(
		.clock(transmission_clk), 
		.start(rec_crc_start),
		.serial_data_in(receive_signal), 
		.done(crc_done), 
		.r(rec_r), 
		.receiver_flag(receiver_flag),
		//.count(), 
		.crc_good(crc_good) 
		);
		
	receiverNew rcN1(
		.clk(clock_27mhz_in), // TEMP
		.reset(reset),
		.seventy_one_bit_packet(parallel_data_out),
		.crc_done(crc_done),
		.crc_good(crc_good),
		.memory_full(),
		.receive_en(receiver_flag), //for now, not permanent since receiver is technically FSM2
		//.state(), [1:0]
		.data_to_recorder(data_to_recorder),
		.data_to_zbt(data_to_zbt),
		.zbt_address(receiver_zbt_address),
		.resend_address(resend_address),
		.write_enable(write_enable)
		);
	
	
	
	mybram #(.LOGSIZE(16),.WIDTH(36)) bram11(.addr(receiver_zbt_address),
															 .clk(clock_27mhz_in),
															 .din(data_to_zbt),
															 .dout(fsm_to_recorder_data),
															 .we(write_enable));
	
	
	////////////////////// 5hz land
	
	
	
	always @(posedge clock_27mhz_in) begin
		//dispdata[18:0] = ac97_address;
		//dispdata[63:32] = clock_125mhz_counter;
		
		//dispdata[19:16] = done;
		//dispdata[15:0] = trans_r;
		
		//dispdata[58:0] = transmit_packet;
		//dispdata[63:60] = crc_done;
		//dispdata[59:56] = crc_good;
		dispdata[54:0] = {receiver_zbt_address,data_to_zbt};
		//dispdata[54:0] = {transmit_address,transmitter_data};
		
		//dispdata[54:0] = parallel_data_out[54:0];
		//dispdata[51:0] = datareg[51:0];
		//dispdata[63:60] = send;
		//dispdata[63:60] = receive_signal;
		//dispdata[59:56] = chk_state;
		//dispdata[59:56] = serial;
		//dispdata[63:60] = audio_state;
	end
	
	
	
	
	
	
//   // use FPGA's digital clock manager to produce a
//   // 65MHz clock (actually 64.8MHz)
//   wire clock_65mhz_unbuf,clock_65mhz;
//   DCM vclk1(.CLKIN(clock_27mhz_in),.CLKFX(clock_65mhz_unbuf));
//   // synthesis attribute CLKFX_DIVIDE of vclk1 is 10
//   // synthesis attribute CLKFX_MULTIPLY of vclk1 is 24
//   // synthesis attribute CLK_FEEDBACK of vclk1 is NONE
//   // synthesis attribute CLKIN_PERIOD of vclk1 is 37
//   BUFG vclk2(.O(clock_65mhz),.I(clock_65mhz_unbuf));


	// ramclock module
	wire locked;
   ramclock rc(.ref_clock(clock_27mhz), .fpga_clock(clock_27mhz_in),
					.ram0_clock(ram0_clk), 
					//.ram1_clock(ram1_clk),   //uncomment if ram1 is used
					.clock_feedback_in(clock_feedback_in),
					.clock_feedback_out(clock_feedback_out), .locked(locked));

   // output useful things to the logic analyzer connectors
   assign analyzer1_clock = ac97_bit_clock;
   assign analyzer1_data[0] = audio_reset_b;
   assign analyzer1_data[1] = ac97_sdata_out;
   assign analyzer1_data[2] = ac97_sdata_in;
   assign analyzer1_data[3] = ac97_synch;
   assign analyzer1_data[15:4] = 0;

   assign analyzer3_clock = ready;
   assign analyzer3_data = {from_ac97_data, to_ac97_data};
endmodule





