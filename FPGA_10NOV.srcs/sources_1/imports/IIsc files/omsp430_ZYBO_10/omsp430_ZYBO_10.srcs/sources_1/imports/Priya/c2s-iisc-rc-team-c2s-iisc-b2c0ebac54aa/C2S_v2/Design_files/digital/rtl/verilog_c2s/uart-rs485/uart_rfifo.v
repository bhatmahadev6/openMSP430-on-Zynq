//////////////////////////////////////////////////////////////////////
////                                                              ////
////  uart_rfifo.v (Modified from uart_fifo.v)                    ////
////                                                              ////
////                                                              ////
////  This file is part of the "UART 16550 compatible" project    ////
////  http://www.opencores.org/cores/uart16550/                   ////
////                                                              ////
////  Documentation related to this project:                      ////
////  - http://www.opencores.org/cores/uart16550/                 ////
////                                                              ////
////  Projects compatibility:                                     ////
////  - WISHBONE                                                  ////
////  RS232 Protocol                                              ////
////  16550D uart (mostly supported)                              ////
////                                                              ////
////  Overview (main Features):                                   ////
////  UART core receiver FIFO                                     ////
////                                                              ////
////  To Do:                                                      ////
////  Nothing.                                                    ////
////                                                              ////
////  Author(s):                                                  ////
////      - gorban@opencores.org                                  ////
////      - Jacob Gorban                                          ////
////      - Igor Mohor (igorm@opencores.org)                      ////
////                                                              ////
////  Created:        2001/05/12                                  ////
////  Last Updated:   2002/07/22                                  ////
////                  (See log for the revision history)          ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000, 2001 Authors                             ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: not supported by cvs2svn $
// Revision 1.3  2003/06/11 16:37:47  gorban
// This fixes errors in some cases when data is being read and put to the FIFO at the same time. Patch is submitted by Scott Furman. Update is very recommended.
//
// Revision 1.2  2002/07/29 21:16:18  gorban
// The uart_defines.v file is included again in sources.
//
// Revision 1.1  2002/07/22 23:02:23  gorban
// Bug Fixes:
//  * Possible loss of sync and bad reception of stop bit on slow baud rates fixed.
//   Problem reported by Kenny.Tung.
//  * Bad (or lack of ) loopback handling fixed. Reported by Cherry Withers.
//
// Improvements:
//  * Made FIFO's as general inferrable memory where possible.
//  So on FPGA they should be inferred as RAM (Distributed RAM on Xilinx).
//  This saves about 1/3 of the Slice count and reduces P&R and synthesis times.
//
//  * Added optional baudrate output (baud_o).
//  This is identical to BAUDOUT* signal on 16550 chip.
//  It outputs 16xbit_clock_rate - the divided clock.
//  It's disabled by default. Define UART_HAS_BAUDRATE_OUTPUT to use.
//
// Revision 1.16  2001/12/20 13:25:46  mohor
// rx push changed to be only one cycle wide.
//
// Revision 1.15  2001/12/18 09:01:07  mohor
// Bug that was entered in the last update fixed (rx state machine).
//
// Revision 1.14  2001/12/17 14:46:48  mohor
// overrun signal was moved to separate block because many sequential lsr
// reads were preventing data from being written to rx fifo.
// underrun signal was not used and was removed from the project.
//
// Revision 1.13  2001/11/26 21:38:54  gorban
// Lots of fixes:
// Break condition wasn't handled correctly at all.
// LSR bits could lose their values.
// LSR value after reset was wrong.
// Timing of THRE interrupt signal corrected.
// LSR bit 0 timing corrected.
//
// Revision 1.12  2001/11/08 14:54:23  mohor
// Comments in Slovene language deleted, few small fixes for better work of
// old tools. IRQs need to be fix.
//
// Revision 1.11  2001/11/07 17:51:52  gorban
// Heavily rewritten interrupt and LSR subsystems.
// Many bugs hopefully squashed.
//
// Revision 1.10  2001/10/20 09:58:40  gorban
// Small synopsis fixes
//
// Revision 1.9  2001/08/24 21:01:12  mohor
// Things connected to parity changed.
// Clock devider changed.
//
// Revision 1.8  2001/08/24 08:48:10  mohor
// FIFO was not cleared after the data was read bug fixed.
//
// Revision 1.7  2001/08/23 16:05:05  mohor
// Stop bit bug fixed.
// Parity bug fixed.
// WISHBONE read cycle bug fixed,
// OE indicator (Overrun Error) bug fixed.
// PE indicator (Parity Error) bug fixed.
// Register read bug fixed.
//
// Revision 1.3  2001/05/31 20:08:01  gorban
// FIFO changes and other corrections.
//
// Revision 1.3  2001/05/27 17:37:48  gorban
// Fixed many bugs. Updated spec. Changed FIFO files structure. See CHANGES.txt file.
//
// Revision 1.2  2001/05/17 18:34:18  gorban
// First 'stable' release. Should be sythesizable now. Also added new header.
//
// Revision 1.0  2001-05-17 21:27:12+02  jacob
// Initial revision
//
//

// synopsys translate_off
//`timescale 1ns/10ps
// synopsys translate_on

//`include "/home/smart/Desktop/C2S_projects/UART_Viv/RTL/uart_defines.v"
//`include "uart_defines.v"
//`include "/home/smart/revised_rtl_jul18/rtl/verilog_c2s/uart-rs485/uart_defines.v"

module uart_rfifo (clk, 
	wb_rst_i, data_in, data_out,
// Control signals
	push, // push strobe, active high
	pop,   // pop strobe, active high
// status signals
	overrun,
	count,
	error_bit,
	fifo_reset,
	reset_status
	);


// FIFO parameters
parameter fifo_width = `UART_FIFO_WIDTH;
parameter fifo_depth = `UART_FIFO_DEPTH;
parameter fifo_pointer_w = `UART_FIFO_POINTER_W;
parameter fifo_counter_w = `UART_FIFO_COUNTER_W;

input				clk;
input				wb_rst_i;
input				push;
input				pop;
input	[`UART_FIFO_REC_WIDTH-1:0]	data_in;//input	[fifo_width-1:0]	data_in;
input				fifo_reset;
input       reset_status;

output	[`UART_FIFO_REC_WIDTH-1:0]	data_out;//output	[fifo_width-1:0]	data_out;
output				overrun;
output	[fifo_counter_w-1:0]	count;
output				error_bit;

wire	[`UART_FIFO_REC_WIDTH-1:0]	data_out;//wire	[fifo_width-1:0]	data_out;
wire [7:0] data8_out;
// flags FIFO
reg	[2:0]	fifo[fifo_depth-1:0];

// FIFO pointers
reg	[fifo_pointer_w-1:0]	top;
reg	[fifo_pointer_w-1:0]	bottom;

reg	[fifo_counter_w-1:0]	count;
reg				overrun;

wire [fifo_pointer_w-1:0] top_plus_1 = top + 8'h01;

raminfrRx #(fifo_pointer_w,8,fifo_depth) rfifo  
        (		.rst(wb_rst_i),
			.clk(clk), 
			.we(push), 
			.a(top), 
			.dpra(bottom), 
			.di(data_in[fifo_width-1:fifo_width-8]), 
			.dpo(data8_out)
		); 

always @(posedge clk or posedge wb_rst_i) // synchronous FIFO
begin
	if (wb_rst_i)
	begin
		top		<= 8'h00;
		bottom		<=8'h00;
		count		<=9'h000;
		fifo[0] <= 3'b000;
		fifo[1] <= 3'b000;
		fifo[2] <= 3'b000;
		fifo[3] <= 3'b000;
		fifo[4] <= 3'b000;
		fifo[5] <= 3'b000;
		fifo[6] <= 3'b000;
		fifo[7] <= 3'b000;
		fifo[8] <= 3'b000;
		fifo[9] <= 3'b000;
		fifo[10] <= 3'b000;
		fifo[11] <= 3'b000;
		fifo[12] <= 3'b000;
		fifo[13] <= 3'b000;
		fifo[14] <= 3'b000;
		fifo[15] <= 3'b000;
		fifo[16] <= 3'b000;
		fifo[17] <= 3'b000;
		fifo[18] <= 3'b000;
		fifo[19] <= 3'b000;
		fifo[20] <= 3'b000;
		fifo[21] <= 3'b000;
		fifo[22] <= 3'b000;
		fifo[23] <= 3'b000;
		fifo[24] <= 3'b000;
		fifo[25] <= 3'b000;
		fifo[26] <= 3'b000;
		fifo[27] <= 3'b000;
		fifo[28] <= 3'b000;
		fifo[29] <= 3'b000;
		fifo[30] <= 3'b000;
		fifo[31] <= 3'b000;
		fifo[32] <= 3'b000;
		fifo[33] <= 3'b000;
		fifo[34] <= 3'b000;
		fifo[35] <= 3'b000;
		fifo[36] <= 3'b000;
		fifo[37] <= 3'b000;
		fifo[38] <= 3'b000;
		fifo[39] <= 3'b000;
		fifo[40] <= 3'b000;
		fifo[41] <= 3'b000;
		fifo[42] <= 3'b000;
		fifo[43] <= 3'b000;
		fifo[44] <= 3'b000;
		fifo[45] <= 3'b000;
		fifo[46] <= 3'b000;
		fifo[47] <= 3'b000;
		fifo[48] <= 3'b000;
		fifo[49] <= 3'b000;
		fifo[50] <= 3'b000;
		fifo[51] <= 3'b000;
		fifo[52] <= 3'b000;
		fifo[53] <= 3'b000;
		fifo[54] <= 3'b000;
		fifo[55] <= 3'b000;
		fifo[56] <= 3'b000;
		fifo[57] <= 3'b000;
		fifo[58] <= 3'b000;
		fifo[59] <= 3'b000;
		fifo[60] <= 3'b000;
		fifo[61] <= 3'b000;
		fifo[62] <= 3'b000;
		fifo[63] <= 3'b000;
		fifo[64] <= 3'b000;
		fifo[65] <= 3'b000;
		fifo[66] <= 3'b000;
		fifo[67] <= 3'b000;
		fifo[68] <= 3'b000;
		fifo[69] <= 3'b000;
		fifo[70] <= 3'b000;
		fifo[71] <= 3'b000;
		fifo[72] <= 3'b000;
		fifo[73] <= 3'b000;
		fifo[74] <= 3'b000;
		fifo[75] <= 3'b000;
		fifo[76] <= 3'b000;
		fifo[77] <= 3'b000;
		fifo[78] <= 3'b000;
		fifo[79] <= 3'b000;
		fifo[80] <= 3'b000;
		fifo[81] <= 3'b000;
		fifo[82] <= 3'b000;
		fifo[83] <= 3'b000;
		fifo[84] <= 3'b000;
		fifo[85] <= 3'b000;
		fifo[86] <= 3'b000;
		fifo[87] <= 3'b000;
		fifo[88] <= 3'b000;
		fifo[89] <= 3'b000;
		fifo[90] <= 3'b000;
		fifo[91] <= 3'b000;
		fifo[92] <= 3'b000;
		fifo[93] <= 3'b000;
		fifo[94] <= 3'b000;
		fifo[95] <= 3'b000;
		fifo[96] <= 3'b000;
		fifo[97] <= 3'b000;
		fifo[98] <= 3'b000;
		fifo[99] <= 3'b000;
		fifo[100] <= 3'b000;
		fifo[101] <= 3'b000;
		fifo[102] <= 3'b000;
		fifo[103] <= 3'b000;
		fifo[104] <= 3'b000;
		fifo[105] <= 3'b000;
		fifo[106] <= 3'b000;
		fifo[107] <= 3'b000;
		fifo[108] <= 3'b000;
		fifo[109] <= 3'b000;
		fifo[110] <= 3'b000;
		fifo[111] <= 3'b000;
		fifo[112] <= 3'b000;
		fifo[113] <= 3'b000;
		fifo[114] <= 3'b000;
		fifo[115] <= 3'b000;
		fifo[116] <= 3'b000;
		fifo[117] <= 3'b000;
		fifo[118] <= 3'b000;
		fifo[119] <= 3'b000;
		fifo[120] <= 3'b000;
		fifo[121] <= 3'b000;
		fifo[122] <= 3'b000;
		fifo[123] <= 3'b000;
		fifo[124] <= 3'b000;
		fifo[125] <= 3'b000;
		fifo[126] <= 3'b000;
		fifo[127] <= 3'b000;
		fifo[128] <= 3'b000;
		fifo[129] <= 3'b000;
		fifo[130] <= 3'b000;
		fifo[131] <= 3'b000;
		fifo[132] <= 3'b000;
		fifo[133] <= 3'b000;
		fifo[134] <= 3'b000;
		fifo[135] <= 3'b000;
		fifo[136] <= 3'b000;
		fifo[137] <= 3'b000;
		fifo[138] <= 3'b000;
		fifo[139] <= 3'b000;
		fifo[140] <= 3'b000;
		fifo[141] <= 3'b000;
		fifo[142] <= 3'b000;
		fifo[143] <= 3'b000;
		fifo[144] <= 3'b000;
		fifo[145] <= 3'b000;
		fifo[146] <= 3'b000;
		fifo[147] <= 3'b000;
		fifo[148] <= 3'b000;
		fifo[149] <= 3'b000;
		fifo[150] <= 3'b000;
		fifo[151] <= 3'b000;
		fifo[152] <= 3'b000;
		fifo[153] <= 3'b000;
		fifo[154] <= 3'b000;
		fifo[155] <= 3'b000;
		fifo[156] <= 3'b000;
		fifo[157] <= 3'b000;
		fifo[158] <= 3'b000;
		fifo[159] <= 3'b000;
		fifo[160] <= 3'b000;
		fifo[161] <= 3'b000;
		fifo[162] <= 3'b000;
		fifo[163] <= 3'b000;
		fifo[164] <= 3'b000;
		fifo[165] <= 3'b000;
		fifo[166] <= 3'b000;
		fifo[167] <= 3'b000;
		fifo[168] <= 3'b000;
		fifo[169] <= 3'b000;
		fifo[170] <= 3'b000;
		fifo[171] <= 3'b000;
		fifo[172] <= 3'b000;
		fifo[173] <= 3'b000;
		fifo[174] <= 3'b000;
		fifo[175] <= 3'b000;
		fifo[176] <= 3'b000;
		fifo[177] <= 3'b000;
		fifo[178] <= 3'b000;
		fifo[179] <= 3'b000;
		fifo[180] <= 3'b000;
		fifo[181] <= 3'b000;
		fifo[182] <= 3'b000;
		fifo[183] <= 3'b000;
		fifo[184] <= 3'b000;
		fifo[185] <= 3'b000;
		fifo[186] <= 3'b000;
		fifo[187] <= 3'b000;
		fifo[188] <= 3'b000;
		fifo[189] <= 3'b000;
		fifo[190] <= 3'b000;
		fifo[191] <= 3'b000;
		fifo[192] <= 3'b000;
		fifo[193] <= 3'b000;
		fifo[194] <= 3'b000;
		fifo[195] <= 3'b000;
		fifo[196] <= 3'b000;
		fifo[197] <= 3'b000;
		fifo[198] <= 3'b000;
		fifo[199] <= 3'b000;
		fifo[200] <= 3'b000;
		fifo[201] <= 3'b000;
		fifo[202] <= 3'b000;
		fifo[203] <= 3'b000;
		fifo[204] <= 3'b000;
		fifo[205] <= 3'b000;
		fifo[206] <= 3'b000;
		fifo[207] <= 3'b000;
		fifo[208] <= 3'b000;
		fifo[209] <= 3'b000;
		fifo[210] <= 3'b000;
		fifo[211] <= 3'b000;
		fifo[212] <= 3'b000;
		fifo[213] <= 3'b000;
		fifo[214] <= 3'b000;
		fifo[215] <= 3'b000;
		fifo[216] <= 3'b000;
		fifo[217] <= 3'b000;
		fifo[218] <= 3'b000;
		fifo[219] <= 3'b000;
		fifo[220] <= 3'b000;
		fifo[221] <= 3'b000;
		fifo[222] <= 3'b000;
		fifo[223] <= 3'b000;
		fifo[224] <= 3'b000;
		fifo[225] <= 3'b000;
		fifo[226] <= 3'b000;
		fifo[227] <= 3'b000;
		fifo[228] <= 3'b000;
		fifo[229] <= 3'b000;
		fifo[230] <= 3'b000;
		fifo[231] <= 3'b000;
		fifo[232] <= 3'b000;
		fifo[233] <= 3'b000;
		fifo[234] <= 3'b000;
		fifo[235] <= 3'b000;
		fifo[236] <= 3'b000;
		fifo[237] <= 3'b000;
		fifo[238] <= 3'b000;
		fifo[239] <= 3'b000;
		fifo[240] <= 3'b000;
		fifo[241] <= 3'b000;
		fifo[242] <= 3'b000;
		fifo[243] <= 3'b000;
		fifo[244] <= 3'b000;
		fifo[245] <= 3'b000;
		fifo[246] <= 3'b000;
		fifo[247] <= 3'b000;
		fifo[248] <= 3'b000;
		fifo[249] <= 3'b000;
		fifo[250] <= 3'b000;
		fifo[251] <= 3'b000;
		fifo[252] <= 3'b000;
		fifo[253] <= 3'b000;
		fifo[254] <= 3'b000;
		fifo[255] <= 3'b000;
	end
	else
	if (fifo_reset) begin
		top		<= 8'h00;
		bottom		<=8'h00;
		count		<=9'h000;
		fifo[0] <= 3'b000;
		fifo[1] <= 3'b000;
		fifo[2] <= 3'b000;
		fifo[3] <= 3'b000;
		fifo[4] <= 3'b000;
		fifo[5] <= 3'b000;
		fifo[6] <= 3'b000;
		fifo[7] <= 3'b000;
		fifo[8] <= 3'b000;
		fifo[9] <= 3'b000;
		fifo[10] <= 3'b000;
		fifo[11] <= 3'b000;
		fifo[12] <= 3'b000;
		fifo[13] <= 3'b000;
		fifo[14] <= 3'b000;
		fifo[15] <= 3'b000;
		fifo[17] <= 3'b000;
		fifo[18] <= 3'b000;
		fifo[19] <= 3'b000;
		fifo[20] <= 3'b000;
		fifo[21] <= 3'b000;
		fifo[22] <= 3'b000;
		fifo[23] <= 3'b000;
		fifo[24] <= 3'b000;
		fifo[25] <= 3'b000;
		fifo[26] <= 3'b000;
		fifo[27] <= 3'b000;
		fifo[28] <= 3'b000;
		fifo[29] <= 3'b000;
		fifo[30] <= 3'b000;
		fifo[31] <= 3'b000;
		fifo[32] <= 3'b000;
		fifo[33] <= 3'b000;
		fifo[34] <= 3'b000;
		fifo[35] <= 3'b000;
		fifo[36] <= 3'b000;
		fifo[37] <= 3'b000;
		fifo[38] <= 3'b000;
		fifo[39] <= 3'b000;
		fifo[40] <= 3'b000;
		fifo[41] <= 3'b000;
		fifo[42] <= 3'b000;
		fifo[43] <= 3'b000;
		fifo[44] <= 3'b000;
		fifo[45] <= 3'b000;
		fifo[46] <= 3'b000;
		fifo[47] <= 3'b000;
		fifo[48] <= 3'b000;
		fifo[49] <= 3'b000;
		fifo[50] <= 3'b000;
		fifo[51] <= 3'b000;
		fifo[52] <= 3'b000;
		fifo[53] <= 3'b000;
		fifo[54] <= 3'b000;
		fifo[55] <= 3'b000;
		fifo[56] <= 3'b000;
		fifo[57] <= 3'b000;
		fifo[58] <= 3'b000;
		fifo[59] <= 3'b000;
		fifo[60] <= 3'b000;
		fifo[61] <= 3'b000;
		fifo[62] <= 3'b000;
		fifo[63] <= 3'b000;
		fifo[64] <= 3'b000;
		fifo[65] <= 3'b000;
		fifo[66] <= 3'b000;
		fifo[67] <= 3'b000;
		fifo[68] <= 3'b000;
		fifo[69] <= 3'b000;
		fifo[70] <= 3'b000;
		fifo[71] <= 3'b000;
		fifo[72] <= 3'b000;
		fifo[73] <= 3'b000;
		fifo[74] <= 3'b000;
		fifo[75] <= 3'b000;
		fifo[76] <= 3'b000;
		fifo[77] <= 3'b000;
		fifo[78] <= 3'b000;
		fifo[79] <= 3'b000;
		fifo[80] <= 3'b000;
		fifo[81] <= 3'b000;
		fifo[82] <= 3'b000;
		fifo[83] <= 3'b000;
		fifo[84] <= 3'b000;
		fifo[85] <= 3'b000;
		fifo[86] <= 3'b000;
		fifo[87] <= 3'b000;
		fifo[88] <= 3'b000;
		fifo[89] <= 3'b000;
		fifo[90] <= 3'b000;
		fifo[91] <= 3'b000;
		fifo[92] <= 3'b000;
		fifo[93] <= 3'b000;
		fifo[94] <= 3'b000;
		fifo[95] <= 3'b000;
		fifo[96] <= 3'b000;
		fifo[97] <= 3'b000;
		fifo[98] <= 3'b000;
		fifo[99] <= 3'b000;
		fifo[100] <= 3'b000;
		fifo[101] <= 3'b000;
		fifo[102] <= 3'b000;
		fifo[103] <= 3'b000;
		fifo[104] <= 3'b000;
		fifo[105] <= 3'b000;
		fifo[106] <= 3'b000;
		fifo[107] <= 3'b000;
		fifo[108] <= 3'b000;
		fifo[109] <= 3'b000;
		fifo[110] <= 3'b000;
		fifo[111] <= 3'b000;
		fifo[112] <= 3'b000;
		fifo[113] <= 3'b000;
		fifo[114] <= 3'b000;
		fifo[115] <= 3'b000;
		fifo[116] <= 3'b000;
		fifo[117] <= 3'b000;
		fifo[118] <= 3'b000;
		fifo[119] <= 3'b000;
		fifo[120] <= 3'b000;
		fifo[121] <= 3'b000;
		fifo[122] <= 3'b000;
		fifo[123] <= 3'b000;
		fifo[124] <= 3'b000;
		fifo[125] <= 3'b000;
		fifo[126] <= 3'b000;
		fifo[127] <= 3'b000;
		fifo[128] <= 3'b000;
		fifo[129] <= 3'b000;
		fifo[130] <= 3'b000;
		fifo[131] <= 3'b000;
		fifo[132] <= 3'b000;
		fifo[133] <= 3'b000;
		fifo[134] <= 3'b000;
		fifo[135] <= 3'b000;
		fifo[136] <= 3'b000;
		fifo[137] <= 3'b000;
		fifo[138] <= 3'b000;
		fifo[139] <= 3'b000;
		fifo[140] <= 3'b000;
		fifo[141] <= 3'b000;
		fifo[142] <= 3'b000;
		fifo[143] <= 3'b000;
		fifo[144] <= 3'b000;
		fifo[145] <= 3'b000;
		fifo[146] <= 3'b000;
		fifo[147] <= 3'b000;
		fifo[148] <= 3'b000;
		fifo[149] <= 3'b000;
		fifo[150] <= 3'b000;
		fifo[151] <= 3'b000;
		fifo[152] <= 3'b000;
		fifo[153] <= 3'b000;
		fifo[154] <= 3'b000;
		fifo[155] <= 3'b000;
		fifo[156] <= 3'b000;
		fifo[157] <= 3'b000;
		fifo[158] <= 3'b000;
		fifo[159] <= 3'b000;
		fifo[160] <= 3'b000;
		fifo[161] <= 3'b000;
		fifo[162] <= 3'b000;
		fifo[163] <= 3'b000;
		fifo[164] <= 3'b000;
		fifo[165] <= 3'b000;
		fifo[166] <= 3'b000;
		fifo[167] <= 3'b000;
		fifo[168] <= 3'b000;
		fifo[169] <= 3'b000;
		fifo[170] <= 3'b000;
		fifo[171] <= 3'b000;
		fifo[172] <= 3'b000;
		fifo[173] <= 3'b000;
		fifo[174] <= 3'b000;
		fifo[175] <= 3'b000;
		fifo[176] <= 3'b000;
		fifo[177] <= 3'b000;
		fifo[178] <= 3'b000;
		fifo[179] <= 3'b000;
		fifo[180] <= 3'b000;
		fifo[181] <= 3'b000;
		fifo[182] <= 3'b000;
		fifo[183] <= 3'b000;
		fifo[184] <= 3'b000;
		fifo[185] <= 3'b000;
		fifo[186] <= 3'b000;
		fifo[187] <= 3'b000;
		fifo[188] <= 3'b000;
		fifo[189] <= 3'b000;
		fifo[190] <= 3'b000;
		fifo[191] <= 3'b000;
		fifo[192] <= 3'b000;
		fifo[193] <= 3'b000;
		fifo[194] <= 3'b000;
		fifo[195] <= 3'b000;
		fifo[196] <= 3'b000;
		fifo[197] <= 3'b000;
		fifo[198] <= 3'b000;
		fifo[199] <= 3'b000;
		fifo[200] <= 3'b000;
		fifo[201] <= 3'b000;
		fifo[202] <= 3'b000;
		fifo[203] <= 3'b000;
		fifo[204] <= 3'b000;
		fifo[205] <= 3'b000;
		fifo[206] <= 3'b000;
		fifo[207] <= 3'b000;
		fifo[208] <= 3'b000;
		fifo[209] <= 3'b000;
		fifo[210] <= 3'b000;
		fifo[211] <= 3'b000;
		fifo[212] <= 3'b000;
		fifo[213] <= 3'b000;
		fifo[214] <= 3'b000;
		fifo[215] <= 3'b000;
		fifo[216] <= 3'b000;
		fifo[217] <= 3'b000;
		fifo[218] <= 3'b000;
		fifo[219] <= 3'b000;
		fifo[220] <= 3'b000;
		fifo[221] <= 3'b000;
		fifo[222] <= 3'b000;
		fifo[223] <= 3'b000;
		fifo[224] <= 3'b000;
		fifo[225] <= 3'b000;
		fifo[226] <= 3'b000;
		fifo[227] <= 3'b000;
		fifo[228] <= 3'b000;
		fifo[229] <= 3'b000;
		fifo[230] <= 3'b000;
		fifo[231] <= 3'b000;
		fifo[232] <= 3'b000;
		fifo[233] <= 3'b000;
		fifo[234] <= 3'b000;
		fifo[235] <= 3'b000;
		fifo[236] <= 3'b000;
		fifo[237] <= 3'b000;
		fifo[238] <= 3'b000;
		fifo[239] <= 3'b000;
		fifo[240] <= 3'b000;
		fifo[241] <= 3'b000;
		fifo[242] <= 3'b000;
		fifo[243] <= 3'b000;
		fifo[244] <= 3'b000;
		fifo[245] <= 3'b000;
		fifo[246] <= 3'b000;
		fifo[247] <= 3'b000;
		fifo[248] <= 3'b000;
		fifo[249] <= 3'b000;
		fifo[250] <= 3'b000;
		fifo[251] <= 3'b000;
		fifo[252] <= 3'b000;
		fifo[253] <= 3'b000;
		fifo[254] <= 3'b000;
		fifo[255] <= 3'b000;
	end
  else
	begin
		case ({push, pop})
		2'b10 : if (count<fifo_depth)  // overrun condition
			begin
				top       <= top_plus_1;
				fifo[top] <= data_in[2:0];
				count     <= count + 9'h001;
			end
			else 
			begin
				top<=top;
				count <= count;
				fifo[top]<=fifo[top];
			end
		2'b01 : if(count>0)
			begin
        //fifo[bottom] <= #1 0;
                		fifo[bottom] <= 3'b000;
				bottom   <=  bottom + 8'h01;
				count	 <=  count - 9'h001;
			end
			else 
			begin
				top<=top;
				count <= count;
				fifo[top]<=fifo[top];
			end
		2'b11 : begin
				bottom   <=  bottom + 8'h01;
				top       <=  top_plus_1;
				fifo[top] <=  data_in[2:0];
		        end
    		default: begin
			top<=top;
			count <= count;
			fifo[top]<=fifo[top];
			end
		endcase
	end
end   // always

always @(posedge clk or posedge wb_rst_i) // synchronous FIFO
begin
  if (wb_rst_i)
    overrun   <= 1'b0;
  else
  if(fifo_reset | reset_status) 
    overrun   <= 1'b0;
  else
  if(push & ~pop & (count==fifo_depth))
    overrun   <= 1'b1;
  else
    overrun <= overrun;
end   // always


// please note though that data_out is only valid one clock after pop signal
assign data_out = {data8_out,fifo[bottom]};

// Additional logic for detection of error conditions (parity and framing) inside the FIFO
// for the Line Status Register bit 7

wire	[2:0]	word0 = fifo[0];
wire	[2:0]	word1 = fifo[1];
wire	[2:0]	word2 = fifo[2];
wire	[2:0]	word3 = fifo[3];
wire	[2:0]	word4 = fifo[4];
wire	[2:0]	word5 = fifo[5];
wire	[2:0]	word6 = fifo[6];
wire	[2:0]	word7 = fifo[7];

wire	[2:0]	word8 = fifo[8];
wire	[2:0]	word9 = fifo[9];
wire	[2:0]	word10 = fifo[10];
wire	[2:0]	word11 = fifo[11];
wire	[2:0]	word12 = fifo[12];
wire	[2:0]	word13 = fifo[13];
wire	[2:0]	word14 = fifo[14];
wire	[2:0]	word15 = fifo[15];

// a 1 is returned if any of the error bits in the fifo is 1
assign	error_bit = |(word0[2:0]  | word1[2:0]  | word2[2:0]  | word3[2:0]  |
            		      word4[2:0]  | word5[2:0]  | word6[2:0]  | word7[2:0]  |
            		      word8[2:0]  | word9[2:0]  | word10[2:0] | word11[2:0] |
            		      word12[2:0] | word13[2:0] | word14[2:0] | word15[2:0] );

endmodule
