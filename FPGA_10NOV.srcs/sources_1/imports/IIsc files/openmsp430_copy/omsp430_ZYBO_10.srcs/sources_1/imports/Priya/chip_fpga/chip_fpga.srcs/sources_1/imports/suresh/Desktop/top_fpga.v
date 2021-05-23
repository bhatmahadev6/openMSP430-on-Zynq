`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Devanjan Maiti 
// 
// Create Date: 18.04.2017 14:35:36
// Design Name: 
// Module Name: top_fpga
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This is the top level FPGA wrapper for C2S56001 ASIC. This wrapper
//              includes:
//              1. top_digital FPGA version (digital top level wrapper for C2S56001)
//              2. XADC instance (ADC IP Core from Xilinx)
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module top_fpga(
input  wire clk_in,
input  wire master_rst_n,


input  wire dbg_en_ext_i,
input  wire dbg_uart_rxd_in,
output wire dbg_uart_txd,
output wire pwm_out,

//output wire sclk,
//output wire Sin,
//input  wire So,
//output wire Cen, 

input  wire flash_program_mode,
input  wire normal_mode,
output wire start_load,
input  wire program_loaded,

//TODO: GPIO Testing
input [1:0] p1_din,
output [1:0] p1_dout,

//output ta_out0,
//output ta_out0_en,
//input  ta_cci0a,ta_cci0b,
output wire memory_check_done,
output wire memory_error,
 
// UART 
output wire Dtx,
input  wire Drx,

//input wire [3:0] temp_count_i,
//output wire [3:0]switch_o,
//output wire counter_rst_o,


//// XADC (Analog pins)
input  wire vp_in,
input  wire vn_in
//output wire ot_out
//output reg led

    );
    
   
//wire [1:0] led_per_wen;
//wire led_per_cen;
//wire [13:0] led_per_addr;
//wire [15:0] led_per_din;
//wire [15:0] led_per_dout;

//------------------------
// Reg/Wire declarations
//------------------------
wire clk_100M;   // XADC clock
wire puc_rst;    // Main system reset

//// Bus interface for XADC 
wire [13:0] xadc_per_addr;  // controller address
wire [15:0] xadc_per_din;   // controller data input; from processor
wire        xadc_per_en;    // controller enable (high active)
wire [ 1:0] xadc_per_we;    // Peripheral write enable (high active)
wire [15:0] xadc_per_dout;  // controller data output; to processor

//// XADC Read Address
wire [ 6:0] xadc_rd_addr;

//------------
//Glue Logic
//------------
// Mapping CNTRL1 address 0170h to internal address of XADC Status register
// Address 03h for Vp/Vn data (ref: Pg. 37, UG 480 (ver. 1.9) XADC User Guide 
assign xadc_rd_addr = ((xadc_per_addr == 15'h00b8) ? 7'h03 : 7'hX); //TODO: Address should be halved 



//assign led_per_dout = 0;
//	 always @(posedge clk_in or posedge master_rst_n) begin
//		if(master_rst_n) led <= 0;
//		else if(led_per_wen[0] & led_per_cen)
//				if(led_per_addr == 0)
//					led <= led_per_din[0];
//	 end
	 


 clk_wiz_0 clk_fpga_generator
   (
   // Clock in ports
    .clk_in1(clk_in),      // input clk_in1
    // Clock out ports
    .PLLOUT(PLLOUT));    // output PLLOUT
    
   
    
    
//-------------------------------------
// C2S56001 top level digital instance
//-------------------------------------
top_verilog top_digital_inst (
  .memory_check_done(memory_check_done),
  .dbg_uart_rxd_in(dbg_uart_rxd_in),
  .dbg_uart_txd(dbg_uart_txd), // chip pins
  .pwm_out(pwm_out), // chip pins  
//  .sclk_flash(sclk), //Serial Flash Chip Pins
//  .din_flash(Sin),
//  .dout_flash(So),
//  .cs_flash(Cen),  
  .dbg_en_p(dbg_en_ext_i),
  .flash_program_mode(flash_program_mode),
  .normal_mode(normal_mode),
  .start_load(start_load),
  .program_loaded(program_loaded),
  .clk_100M(clk_100M),
  .poweron_reset(master_rst_n),
  .PLLOUT(PLLOUT),
  .p1_dout(p1_dout), 
  .p1_din(p1_din),
//  .p1_sel(p1_sel),
  .p1_dout_en(p1_dout_en),
  .memory_error(memory_error),
//  // UART 
  .Dtx(Dtx),
  .Drx(Drx),

//  //temperature sensor
//  .temp_count_i(temp_count_i),
//  .switch_o(switch_o),
//  .counter_rst_o(counter_rst_o),
        
//  // Bus interface for XADC 
  .xadc_per_addr(xadc_per_addr), // controller address
  .xadc_per_din(xadc_per_din),   // controller data input, from processor
  .xadc_per_en(xadc_per_en),     // controller enable (high active)
  .xadc_per_we(xadc_per_we),     // Peripheral write enable (high active)
  .xadc_per_dout(xadc_per_dout), // controller data output, to processor
  .puc_rst_o(puc_rst)           // reset for XADC
  
//  .led_per_cen(led_per_cen),
//  .led_per_wen(led_per_wen),
//  .led_per_addr(led_per_addr),
//  .led_per_din(led_per_din),
//  .led_per_dout(led_per_dout)
  
      );

//-------------------------------
// Single channel ADC instance   
//-------------------------------   
xadc_wiz_0 xadc_inst(
  .di_in(xadc_per_din),    // input wire [15 : 0] di_in
  .daddr_in(xadc_rd_addr), // input wire [6 : 0] daddr_in
  .den_in(xadc_per_en),    // input wire den_in
  .dwe_in(xadc_per_we),    // input wire dwe_in
  .drdy_out(),             // output wire drdy_out TODO
  .do_out(xadc_per_dout),  // output wire [15 : 0] do_out
  .dclk_in(clk_100M),      // input wire dclk_in
  .reset_in(puc_rst),      // input wire reset_in
  .vp_in(1'b0),            // input wire vp_in
  .vn_in(1'b0),            // input wire vn_in
  .vauxp6(vp_in),          // input wire vauxp6
  .vauxn6(vn_in),          // input wire vauxn6
  .ot_out(ot_out),         // output wire ot_out
  .channel_out(),          // output wire [4 : 0] channel_out
  .eoc_out(),              // output wire eoc_out TODO
  .alarm_out(),            // output wire alarm_out
  .eos_out(),              // output wire eos_out
  .busy_out()              // output wire busy_out TODO
);      


//wire sclk,Sin,So,Cen;
/*
mem_flash_serial serial_flash(

.Cen(Cen),
.Sclk(sclk),
.Sin(Sin),
.Sout(Sout)

);*/

endmodule
