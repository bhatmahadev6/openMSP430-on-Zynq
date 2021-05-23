//`timescale 1ns/10ps
//////////////////////////////////////////////////////////////////////
////                                                              ////
////  raminfr.v                                                   ////
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
////  Inferrable Distributed RAM for FIFOs                        ////
////                                                              ////
////  Known problems (limits):                                    ////
////  None                .                                       ////
////                                                              ////
////  To Do:                                                      ////
////  Nothing so far.                                             ////
////                                                              ////
////  Author(s):                                                  ////
////      - gorban@opencores.org                                  ////
////      - Jacob Gorban                                          ////
////                                                              ////
////  Created:        2002/07/22                                  ////
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

//Following is the Verilog code for a dual-port RAM with asynchronous read. 
module raminfrRx   
        (rst,clk, we, a, dpra, di, dpo); 

parameter addr_width = 8;
parameter data_width = 8;
parameter depth = 256;

input clk,rst;   
input we;   
input  [addr_width-1:0] a;   
input  [addr_width-1:0] dpra;   
input  [data_width-1:0] di;   
//output [data_width-1:0] spo;   
output [data_width-1:0] dpo;   
reg    [data_width-1:0] ram [depth-1:0]; 

wire [data_width-1:0] dpo;
wire  [data_width-1:0] di;   
wire  [addr_width-1:0] a;   
wire  [addr_width-1:0] dpra;   
 
  //always @(posedge clk) begin   
   // if (we) 
	always @(posedge clk or posedge rst) 
	begin
	if (rst)
		begin
		//dpo = 8'h00;
		ram[a]<=8'h00;
		ram[dpra]<=8'h00;
		//for(i=0;i<depth;i=i+1)
		//	ram[i]<=8'h00;
		    ram[255] <= 8'h00;
		    ram[254] <= 8'h00;
		    ram[253] <= 8'h00;
		    ram[252] <= 8'h00;
		    ram[251] <= 8'h00;
		    ram[250] <= 8'h00;
		    ram[249] <= 8'h00;
		    ram[248] <= 8'h00;
		    ram[247] <= 8'h00;
		    ram[246] <= 8'h00;
		    ram[245] <= 8'h00;
		    ram[244] <= 8'h00;
		    ram[243] <= 8'h00;
		    ram[242] <= 8'h00;
		    ram[241] <= 8'h00;
		    ram[240] <= 8'h00;
		    ram[239] <= 8'h00;
		    ram[238] <= 8'h00;
		    ram[237] <= 8'h00;
		    ram[236] <= 8'h00;
		    ram[235] <= 8'h00;
		    ram[234] <= 8'h00;
		    ram[233] <= 8'h00;
		    ram[232] <= 8'h00;
		    ram[231] <= 8'h00;
		    ram[230] <= 8'h00;
		    ram[229] <= 8'h00;
		    ram[228] <= 8'h00;
		    ram[227] <= 8'h00;
		    ram[226] <= 8'h00;
		    ram[225] <= 8'h00;
		    ram[224] <= 8'h00;
		    ram[223] <= 8'h00;
		    ram[222] <= 8'h00;
		    ram[221] <= 8'h00;
		    ram[220] <= 8'h00;
		    ram[219] <= 8'h00;
		    ram[218] <= 8'h00;
		    ram[217] <= 8'h00;
		    ram[216] <= 8'h00;
		    ram[215] <= 8'h00;
		    ram[214] <= 8'h00;
		    ram[213] <= 8'h00;
		    ram[212] <= 8'h00;
		    ram[211] <= 8'h00;
		    ram[210] <= 8'h00;
		    ram[209] <= 8'h00;
		    ram[208] <= 8'h00;
		    ram[207] <= 8'h00;
		    ram[206] <= 8'h00;
		    ram[205] <= 8'h00;
		    ram[204] <= 8'h00;
		    ram[203] <= 8'h00;
		    ram[202] <= 8'h00;
		    ram[201] <= 8'h00;
		    ram[200] <= 8'h00;
		    ram[199] <= 8'h00;
		    ram[198] <= 8'h00;
		    ram[197] <= 8'h00;
		    ram[196] <= 8'h00;
		    ram[195] <= 8'h00;
		    ram[194] <= 8'h00;
		    ram[193] <= 8'h00;
		    ram[192] <= 8'h00;
		    ram[191] <= 8'h00;
		    ram[190] <= 8'h00;
		    ram[189] <= 8'h00;
		    ram[188] <= 8'h00;
		    ram[187] <= 8'h00;
		    ram[186] <= 8'h00;
		    ram[185] <= 8'h00;
		    ram[184] <= 8'h00;
		    ram[183] <= 8'h00;
		    ram[182] <= 8'h00;
		    ram[181] <= 8'h00;
		    ram[180] <= 8'h00;
		    ram[179] <= 8'h00;
		    ram[178] <= 8'h00;
		    ram[177] <= 8'h00;
		    ram[176] <= 8'h00;
		    ram[175] <= 8'h00;
		    ram[174] <= 8'h00;
		    ram[173] <= 8'h00;
		    ram[172] <= 8'h00;
		    ram[171] <= 8'h00;
		    ram[170] <= 8'h00;
		    ram[169] <= 8'h00;
		    ram[168] <= 8'h00;
		    ram[167] <= 8'h00;
		    ram[166] <= 8'h00;
		    ram[165] <= 8'h00;
		    ram[164] <= 8'h00;
		    ram[163] <= 8'h00;
		    ram[162] <= 8'h00;
		    ram[161] <= 8'h00;
		    ram[160] <= 8'h00;
		    ram[159] <= 8'h00;
		    ram[158] <= 8'h00;
		    ram[157] <= 8'h00;
		    ram[156] <= 8'h00;
		    ram[155] <= 8'h00;
		    ram[154] <= 8'h00;
		    ram[153] <= 8'h00;
		    ram[152] <= 8'h00;
		    ram[151] <= 8'h00;
		    ram[150] <= 8'h00;
		    ram[149] <= 8'h00;
		    ram[148] <= 8'h00;
		    ram[147] <= 8'h00;
		    ram[146] <= 8'h00;
		    ram[145] <= 8'h00;
		    ram[144] <= 8'h00;
		    ram[143] <= 8'h00;
		    ram[142] <= 8'h00;
		    ram[141] <= 8'h00;
		    ram[140] <= 8'h00;
		    ram[139] <= 8'h00;
		    ram[138] <= 8'h00;
		    ram[137] <= 8'h00;
		    ram[136] <= 8'h00;
		    ram[135] <= 8'h00;
		    ram[134] <= 8'h00;
		    ram[133] <= 8'h00;
		    ram[132] <= 8'h00;
		    ram[131] <= 8'h00;
		    ram[130] <= 8'h00;
		    ram[129] <= 8'h00;
		    ram[128] <= 8'h00;
		    ram[127] <= 8'h00;
		    ram[126] <= 8'h00;
		    ram[125] <= 8'h00;
		    ram[124] <= 8'h00;
		    ram[123] <= 8'h00;
		    ram[122] <= 8'h00;
		    ram[121] <= 8'h00;
		    ram[120] <= 8'h00;
		    ram[119] <= 8'h00;
		    ram[118] <= 8'h00;
		    ram[117] <= 8'h00;
		    ram[116] <= 8'h00;
		    ram[115] <= 8'h00;
		    ram[114] <= 8'h00;
		    ram[113] <= 8'h00;
		    ram[112] <= 8'h00;
		    ram[111] <= 8'h00;
		    ram[110] <= 8'h00;
		    ram[109] <= 8'h00;
		    ram[108] <= 8'h00;
		    ram[107] <= 8'h00;
		    ram[106] <= 8'h00;
		    ram[105] <= 8'h00;
		    ram[104] <= 8'h00;
		    ram[103] <= 8'h00;
		    ram[102] <= 8'h00;
		    ram[101] <= 8'h00;
		    ram[100] <= 8'h00;
		    ram[99] <= 8'h00;
		    ram[98] <= 8'h00;
		    ram[97] <= 8'h00;
		    ram[96] <= 8'h00;
		    ram[95] <= 8'h00;
		    ram[94] <= 8'h00;
		    ram[93] <= 8'h00;
		    ram[92] <= 8'h00;
		    ram[91] <= 8'h00;
		    ram[90] <= 8'h00;
		    ram[89] <= 8'h00;
		    ram[88] <= 8'h00;
		    ram[87] <= 8'h00;
		    ram[86] <= 8'h00;
		    ram[85] <= 8'h00;
		    ram[84] <= 8'h00;
		    ram[83] <= 8'h00;
		    ram[82] <= 8'h00;
		    ram[81] <= 8'h00;
		    ram[80] <= 8'h00;
		    ram[79] <= 8'h00;
		    ram[78] <= 8'h00;
		    ram[77] <= 8'h00;
		    ram[76] <= 8'h00;
		    ram[75] <= 8'h00;
		    ram[74] <= 8'h00;
		    ram[73] <= 8'h00;
		    ram[72] <= 8'h00;
		    ram[71] <= 8'h00;
		    ram[70] <= 8'h00;
		    ram[69] <= 8'h00;
		    ram[68] <= 8'h00;
		    ram[67] <= 8'h00;
		    ram[66] <= 8'h00;
		    ram[65] <= 8'h00;
		    ram[64] <= 8'h00;
		    ram[63] <= 8'h00;
		    ram[62] <= 8'h00;
		    ram[61] <= 8'h00;
		    ram[60] <= 8'h00;
		    ram[59] <= 8'h00;
		    ram[58] <= 8'h00;
		    ram[57] <= 8'h00;
		    ram[56] <= 8'h00;
		    ram[55] <= 8'h00;
		    ram[54] <= 8'h00;
		    ram[53] <= 8'h00;
		    ram[52] <= 8'h00;
		    ram[51] <= 8'h00;
		    ram[50] <= 8'h00;
		    ram[49] <= 8'h00;
		    ram[48] <= 8'h00;
		    ram[47] <= 8'h00;
		    ram[46] <= 8'h00;
		    ram[45] <= 8'h00;
		    ram[44] <= 8'h00;
		    ram[43] <= 8'h00;
		    ram[42] <= 8'h00;
		    ram[41] <= 8'h00;
		    ram[40] <= 8'h00;
		    ram[39] <= 8'h00;
		    ram[38] <= 8'h00;
		    ram[37] <= 8'h00;
		    ram[36] <= 8'h00;
		    ram[35] <= 8'h00;
		    ram[34] <= 8'h00;
		    ram[33] <= 8'h00;
		    ram[32] <= 8'h00;
		    ram[31] <= 8'h00;
		    ram[30] <= 8'h00;
		    ram[29] <= 8'h00;
		    ram[28] <= 8'h00;
		    ram[27] <= 8'h00;
		    ram[26] <= 8'h00;
		    ram[25] <= 8'h00;
		    ram[24] <= 8'h00;
		    ram[23] <= 8'h00;
		    ram[22] <= 8'h00;
		    ram[21] <= 8'h00;
		    ram[20] <= 8'h00;
		    ram[19] <= 8'h00;
		    ram[18] <= 8'h00;
		    ram[17] <= 8'h00;
		    ram[16] <= 8'h00;
		    ram[15] <= 8'h00;
		    ram[14] <= 8'h00;
		    ram[13] <= 8'h00;
		    ram[12] <= 8'h00;
		    ram[11] <= 8'h00;
		    ram[10] <= 8'h00;
		    ram[9] <= 8'h00;
		    ram[8] <= 8'h00;
		    ram[7] <= 8'h00;
		    ram[6] <= 8'h00;
		    ram[5] <= 8'h00;
		    ram[4] <= 8'h00;
		    ram[3] <= 8'h00;
		    ram[2] <= 8'h00;
		    ram[1] <= 8'h00;
		    ram[0] <= 8'h00;
		end
	else if (we)     
		begin
      		ram[a] <= di; 
		//ram[dpra]<=ram[dpra];  
		end
	else
		begin
		ram[a]<=ram[a];
		ram[dpra]<=ram[dpra];  
		end
  end   
//  assign spo = ram[a];   
  assign dpo = ram[dpra];   
endmodule 

