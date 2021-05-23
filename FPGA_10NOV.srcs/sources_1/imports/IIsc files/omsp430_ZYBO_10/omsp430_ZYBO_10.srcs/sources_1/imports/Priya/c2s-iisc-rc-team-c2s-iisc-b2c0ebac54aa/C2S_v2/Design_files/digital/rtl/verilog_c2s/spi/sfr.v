`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/22/2016 10:20:49 AM
// Design Name: 
// Module Name: sfr
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define addr_FMA_high 16'h0190
`define addr_FMA_low 16'h0192
`define addr_FMC 16'h0194



///////////////////////////////////////////////////////////////////////////////


module sfr(clk_sfr50,clk_sfr25,rst,per_en,we_en,we_tx_fifo,we_rx_fifo,rd_tx_fifo,rd_rx_fifo,addr,fifo_addr,
            per_data,data,FMA_low,FMA_high,FMC,tx_fifo_cur,rx_fifo_cur,per_dout
    );
    
    input clk_sfr50,clk_sfr25;
    input rst;
    input per_en;
    input we_en;
    input we_tx_fifo;
    input we_rx_fifo;
    input rd_tx_fifo;
    input rd_rx_fifo;
    
    input [15:0]  addr;
    input [15:0]  per_data;
    input [15:0]  data;
    input [7:0]   fifo_addr;
    //reg [7:0]   fifo_addr1;
    
    output    reg[15:0] FMA_low;
    output    reg[15:0] FMA_high;
    output    reg[15:0] FMC;
    output  reg [15:0] per_dout;
    
    output reg[15:0] tx_fifo_cur;
    output reg[15:0] rx_fifo_cur;
    
    reg[15:0] tx_fifo[127:0];
    reg[15:0] rx_fifo[127:0];
	wire[15:0] per_addr;
  //  reg clk_sfr;
	
	assign per_addr = addr;
    

  /*  always @(posedge clk_sfr50)
      if (~rst) 
		clk_sfr <= 1'b1;
      else         
		clk_sfr <= ~(clk_sfr);*/

 
    always @(posedge clk_sfr25) begin
        if(~rst) begin
            tx_fifo_cur <= 16'b0;
            rx_fifo_cur <= 16'b0;
        end    
        else if( rd_tx_fifo ) begin
            tx_fifo_cur <= tx_fifo[fifo_addr[7:0]];
        end
        else if( rd_rx_fifo ) begin
            rx_fifo_cur <= rx_fifo[fifo_addr[7:0]];
        end
	else begin
		rx_fifo_cur<=rx_fifo_cur;
		tx_fifo_cur<=tx_fifo_cur;
	end
    end

    always @(posedge clk_sfr50) begin
        if(~rst) begin
            per_dout <= 16'b0;
        end    
        else if(per_en && (~we_en) && (per_addr == 16'h0192)) begin
            per_dout <= FMA_low;
        end
        else if( per_en && (~we_en) && (per_addr == 16'h0190)) begin
            per_dout <= FMA_high;
        end
        else if( per_en && (~we_en) && (per_addr == 16'h0194)) begin
            per_dout <= FMC;
        end
    end

    always @(posedge clk_sfr50) begin
        if(~rst) begin
            FMA_low <= 16'b0;
            FMA_high <= 16'b0;
            FMC <= 16'b0;
        end    
        else if( we_en && per_en &&(per_addr == 16'h0192)) begin
            FMA_low <= per_data;
        end
        else if( we_en && per_en && (per_addr == 16'h0190)) begin
            FMA_high <= per_data;
        end
        else if( we_en && per_en && (per_addr == 16'h0194)) begin
            FMC <= per_data;
        end
	else begin
		FMA_low <= FMA_low;
            	FMA_high <= FMA_high;
            	FMC <= FMC;
		
	     end
    end

   
    always @(posedge clk_sfr25) begin
        if(~rst) begin    
            tx_fifo[127] <= 16'b0;
            tx_fifo[126] <= 16'b0;
            tx_fifo[125] <= 16'b0;
            tx_fifo[124] <= 16'b0;
            tx_fifo[123] <= 16'b0;
            tx_fifo[122] <= 16'b0;
            tx_fifo[121] <= 16'b0;
            tx_fifo[120] <= 16'b0;
            tx_fifo[119] <= 16'b0;
            tx_fifo[118] <= 16'b0;
            tx_fifo[117] <= 16'b0;
            tx_fifo[116] <= 16'b0;
            tx_fifo[115] <= 16'b0;
            tx_fifo[114] <= 16'b0;
            tx_fifo[113] <= 16'b0;
            tx_fifo[112] <= 16'b0;
            tx_fifo[111] <= 16'b0;
            tx_fifo[110] <= 16'b0;
            tx_fifo[109] <= 16'b0;
            tx_fifo[108] <= 16'b0;
            tx_fifo[107] <= 16'b0;
            tx_fifo[106] <= 16'b0;
            tx_fifo[105] <= 16'b0;
            tx_fifo[104] <= 16'b0;
            tx_fifo[103] <= 16'b0;
            tx_fifo[102] <= 16'b0;
            tx_fifo[101] <= 16'b0;
            tx_fifo[100] <= 16'b0;
            tx_fifo[99] <= 16'b0;
            tx_fifo[98] <= 16'b0;
            tx_fifo[97] <= 16'b0;
            tx_fifo[96] <= 16'b0;
            tx_fifo[95] <= 16'b0;
            tx_fifo[94] <= 16'b0;
            tx_fifo[93] <= 16'b0;
            tx_fifo[92] <= 16'b0;
            tx_fifo[91] <= 16'b0;
            tx_fifo[90] <= 16'b0;
            tx_fifo[89] <= 16'b0;
            tx_fifo[88] <= 16'b0;
            tx_fifo[87] <= 16'b0;
            tx_fifo[86] <= 16'b0;
            tx_fifo[85] <= 16'b0;
            tx_fifo[84] <= 16'b0;
            tx_fifo[83] <= 16'b0;
            tx_fifo[82] <= 16'b0;
            tx_fifo[81] <= 16'b0;
            tx_fifo[80] <= 16'b0;
            tx_fifo[79] <= 16'b0;
            tx_fifo[78] <= 16'b0;
            tx_fifo[77] <= 16'b0;
            tx_fifo[76] <= 16'b0;
            tx_fifo[75] <= 16'b0;
            tx_fifo[74] <= 16'b0;
            tx_fifo[73] <= 16'b0;
            tx_fifo[72] <= 16'b0;
            tx_fifo[71] <= 16'b0;
            tx_fifo[70] <= 16'b0;
            tx_fifo[69] <= 16'b0;
            tx_fifo[68] <= 16'b0;
            tx_fifo[67] <= 16'b0;
            tx_fifo[66] <= 16'b0;
            tx_fifo[65] <= 16'b0;
            tx_fifo[64] <= 16'b0;
            tx_fifo[63] <= 16'b0;
            tx_fifo[62] <= 16'b0;
            tx_fifo[61] <= 16'b0;
            tx_fifo[60] <= 16'b0;
            tx_fifo[59] <= 16'b0;
            tx_fifo[58] <= 16'b0;
            tx_fifo[57] <= 16'b0;
            tx_fifo[56] <= 16'b0;
            tx_fifo[55] <= 16'b0;
            tx_fifo[54] <= 16'b0;
            tx_fifo[53] <= 16'b0;
            tx_fifo[52] <= 16'b0;
            tx_fifo[51] <= 16'b0;
            tx_fifo[50] <= 16'b0;
            tx_fifo[49] <= 16'b0;
            tx_fifo[48] <= 16'b0;
            tx_fifo[47] <= 16'b0;
            tx_fifo[46] <= 16'b0;
            tx_fifo[45] <= 16'b0;
            tx_fifo[44] <= 16'b0;
            tx_fifo[43] <= 16'b0;
            tx_fifo[42] <= 16'b0;
            tx_fifo[41] <= 16'b0;
            tx_fifo[40] <= 16'b0;
            tx_fifo[39] <= 16'b0;
            tx_fifo[38] <= 16'b0;
            tx_fifo[37] <= 16'b0;
            tx_fifo[36] <= 16'b0;
            tx_fifo[35] <= 16'b0;
            tx_fifo[34] <= 16'b0;
            tx_fifo[33] <= 16'b0;
            tx_fifo[32] <= 16'b0;
            tx_fifo[31] <= 16'b0;
            tx_fifo[30] <= 16'b0;
            tx_fifo[29] <= 16'b0;
            tx_fifo[28] <= 16'b0;
            tx_fifo[27] <= 16'b0;
            tx_fifo[26] <= 16'b0;
            tx_fifo[25] <= 16'b0;
            tx_fifo[24] <= 16'b0;
            tx_fifo[23] <= 16'b0;
            tx_fifo[22] <= 16'b0;
            tx_fifo[21] <= 16'b0;
            tx_fifo[20] <= 16'b0;
            tx_fifo[19] <= 16'b0;
            tx_fifo[18] <= 16'b0;
            tx_fifo[17] <= 16'b0;
            tx_fifo[16] <= 16'b0;
            tx_fifo[15] <= 16'b0;
            tx_fifo[14] <= 16'b0;
            tx_fifo[13] <= 16'b0;
            tx_fifo[12] <= 16'b0;
            tx_fifo[11] <= 16'b0;
            tx_fifo[10] <= 16'b0;
            tx_fifo[9] <= 16'b0;
            tx_fifo[8] <= 16'b0;
            tx_fifo[7] <= 16'b0;
            tx_fifo[6] <= 16'b0;
            tx_fifo[5] <= 16'b0;
            tx_fifo[4] <= 16'b0;
            tx_fifo[3] <= 16'b0;
            tx_fifo[2] <= 16'b0;
            tx_fifo[1] <= 16'b0;
            tx_fifo[0] <= 16'b0; 
           
            rx_fifo[127] <= 16'b0;
            rx_fifo[126] <= 16'b0;
            rx_fifo[125] <= 16'b0;
            rx_fifo[124] <= 16'b0;
            rx_fifo[123] <= 16'b0;
            rx_fifo[122] <= 16'b0;
            rx_fifo[121] <= 16'b0;
            rx_fifo[120] <= 16'b0;
            rx_fifo[119] <= 16'b0;
            rx_fifo[118] <= 16'b0;
            rx_fifo[117] <= 16'b0;
            rx_fifo[116] <= 16'b0;
            rx_fifo[115] <= 16'b0;
            rx_fifo[114] <= 16'b0;
            rx_fifo[113] <= 16'b0;
            rx_fifo[112] <= 16'b0;
            rx_fifo[111] <= 16'b0;
            rx_fifo[110] <= 16'b0;
            rx_fifo[109] <= 16'b0;
            rx_fifo[108] <= 16'b0;
            rx_fifo[107] <= 16'b0;
            rx_fifo[106] <= 16'b0;
            rx_fifo[105] <= 16'b0;
            rx_fifo[104] <= 16'b0;
            rx_fifo[103] <= 16'b0;
            rx_fifo[102] <= 16'b0;
            rx_fifo[101] <= 16'b0;
            rx_fifo[100] <= 16'b0;
            rx_fifo[99] <= 16'b0;
            rx_fifo[98] <= 16'b0;
            rx_fifo[97] <= 16'b0;
            rx_fifo[96] <= 16'b0;
            rx_fifo[95] <= 16'b0;
            rx_fifo[94] <= 16'b0;
            rx_fifo[93] <= 16'b0;
            rx_fifo[92] <= 16'b0;
            rx_fifo[91] <= 16'b0;
            rx_fifo[90] <= 16'b0;
            rx_fifo[89] <= 16'b0;
            rx_fifo[88] <= 16'b0;
            rx_fifo[87] <= 16'b0;
            rx_fifo[86] <= 16'b0;
            rx_fifo[85] <= 16'b0;
            rx_fifo[84] <= 16'b0;
            rx_fifo[83] <= 16'b0;
            rx_fifo[82] <= 16'b0;
            rx_fifo[81] <= 16'b0;
            rx_fifo[80] <= 16'b0;
            rx_fifo[79] <= 16'b0;
            rx_fifo[78] <= 16'b0;
            rx_fifo[77] <= 16'b0;
            rx_fifo[76] <= 16'b0;
            rx_fifo[75] <= 16'b0;
            rx_fifo[74] <= 16'b0;
            rx_fifo[73] <= 16'b0;
            rx_fifo[72] <= 16'b0;
            rx_fifo[71] <= 16'b0;
            rx_fifo[70] <= 16'b0;
            rx_fifo[69] <= 16'b0;
            rx_fifo[68] <= 16'b0;
            rx_fifo[67] <= 16'b0;
            rx_fifo[66] <= 16'b0;
            rx_fifo[65] <= 16'b0;
            rx_fifo[64] <= 16'b0;
            rx_fifo[63] <= 16'b0;
            rx_fifo[62] <= 16'b0;
            rx_fifo[61] <= 16'b0;
            rx_fifo[60] <= 16'b0;
            rx_fifo[59] <= 16'b0;
            rx_fifo[58] <= 16'b0;
            rx_fifo[57] <= 16'b0;
            rx_fifo[56] <= 16'b0;
            rx_fifo[55] <= 16'b0;
            rx_fifo[54] <= 16'b0;
            rx_fifo[53] <= 16'b0;
            rx_fifo[52] <= 16'b0;
            rx_fifo[51] <= 16'b0;
            rx_fifo[50] <= 16'b0;
            rx_fifo[49] <= 16'b0;
            rx_fifo[48] <= 16'b0;
            rx_fifo[47] <= 16'b0;
            rx_fifo[46] <= 16'b0;
            rx_fifo[45] <= 16'b0;
            rx_fifo[44] <= 16'b0;
            rx_fifo[43] <= 16'b0;
            rx_fifo[42] <= 16'b0;
            rx_fifo[41] <= 16'b0;
            rx_fifo[40] <= 16'b0;
            rx_fifo[39] <= 16'b0;
            rx_fifo[38] <= 16'b0;
            rx_fifo[37] <= 16'b0;
            rx_fifo[36] <= 16'b0;
            rx_fifo[35] <= 16'b0;
            rx_fifo[34] <= 16'b0;
            rx_fifo[33] <= 16'b0;
            rx_fifo[32] <= 16'b0;
            rx_fifo[31] <= 16'b0;
            rx_fifo[30] <= 16'b0;
            rx_fifo[29] <= 16'b0;
            rx_fifo[28] <= 16'b0;
            rx_fifo[27] <= 16'b0;
            rx_fifo[26] <= 16'b0;
            rx_fifo[25] <= 16'b0;
            rx_fifo[24] <= 16'b0;
            rx_fifo[23] <= 16'b0;
            rx_fifo[22] <= 16'b0;
            rx_fifo[21] <= 16'b0;
            rx_fifo[20] <= 16'b0;
            rx_fifo[19] <= 16'b0;
            rx_fifo[18] <= 16'b0;
            rx_fifo[17] <= 16'b0;
            rx_fifo[16] <= 16'b0;
            rx_fifo[15] <= 16'b0;
            rx_fifo[14] <= 16'b0;
            rx_fifo[13] <= 16'b0;
            rx_fifo[12] <= 16'b0;
            rx_fifo[11] <= 16'b0;
            rx_fifo[10] <= 16'b0;
            rx_fifo[9] <= 16'b0;
            rx_fifo[8] <= 16'b0;
            rx_fifo[7] <= 16'b0;
            rx_fifo[6] <= 16'b0;
            rx_fifo[5] <= 16'b0;
            rx_fifo[4] <= 16'b0;
            rx_fifo[3] <= 16'b0;
            rx_fifo[2] <= 16'b0;
            rx_fifo[1] <= 16'b0;
            rx_fifo[0] <= 16'b0;

                      
       end
        else if ( we_tx_fifo ) begin
            
            tx_fifo[fifo_addr[7:0]] <= data;
        end
        else if ( we_rx_fifo ) begin
            rx_fifo[fifo_addr[7:0]] <= data;
        end

	else begin
	    tx_fifo[127] <= tx_fifo[127];
            tx_fifo[126] <= tx_fifo[126];
            tx_fifo[125] <= tx_fifo[125];
            tx_fifo[124] <= tx_fifo[124];
            tx_fifo[123] <= tx_fifo[123];
            tx_fifo[122] <= tx_fifo[122];
            tx_fifo[121] <= tx_fifo[121];
            tx_fifo[120] <= tx_fifo[120];
            tx_fifo[119] <= tx_fifo[119];
            tx_fifo[118] <=  tx_fifo[118];
            tx_fifo[117] <= tx_fifo[117];
            tx_fifo[116] <= tx_fifo[116];
            tx_fifo[115] <=  tx_fifo[115];
            tx_fifo[114] <= tx_fifo[114];
            tx_fifo[113] <= tx_fifo[113];
            tx_fifo[112] <= tx_fifo[112];
            tx_fifo[111] <= tx_fifo[111];
            tx_fifo[110] <= tx_fifo[110];
            tx_fifo[109] <= tx_fifo[109];
            tx_fifo[108] <= tx_fifo[108];
            tx_fifo[107] <= tx_fifo[107];
            tx_fifo[106] <= tx_fifo[106];
            tx_fifo[105] <= tx_fifo[105];
            tx_fifo[104] <= tx_fifo[104];
            tx_fifo[103] <= tx_fifo[103];
            tx_fifo[102] <= tx_fifo[102];
            tx_fifo[101] <= tx_fifo[101];
            tx_fifo[100] <= tx_fifo[100];
            tx_fifo[99] <= tx_fifo[99];
            tx_fifo[98] <= tx_fifo[98];
            tx_fifo[97] <= tx_fifo[97];
            tx_fifo[96] <= tx_fifo[96];
            tx_fifo[95] <= tx_fifo[95];
            tx_fifo[94] <= tx_fifo[94];
            tx_fifo[93] <= tx_fifo[93];
            tx_fifo[92] <= tx_fifo[92];
            tx_fifo[91] <= tx_fifo[91];
            tx_fifo[90] <= tx_fifo[90];
            tx_fifo[89] <= tx_fifo[89];
            tx_fifo[88] <= tx_fifo[88];
            tx_fifo[87] <= tx_fifo[87];
            tx_fifo[86] <= tx_fifo[86];
            tx_fifo[85] <= tx_fifo[85];
            tx_fifo[84] <= tx_fifo[84];
            tx_fifo[83] <= tx_fifo[83];
            tx_fifo[82] <= tx_fifo[82];
            tx_fifo[81] <= tx_fifo[81];
            tx_fifo[80] <= tx_fifo[80];
            tx_fifo[79] <= tx_fifo[79];
            tx_fifo[78] <= tx_fifo[78];
            tx_fifo[77] <= tx_fifo[77];
            tx_fifo[76] <= tx_fifo[76];
            tx_fifo[75] <= tx_fifo[75];
            tx_fifo[74] <= tx_fifo[74];
            tx_fifo[73] <= tx_fifo[73];
            tx_fifo[72] <= tx_fifo[72];
            tx_fifo[71] <= tx_fifo[71];
            tx_fifo[70] <= tx_fifo[70];
            tx_fifo[69] <= tx_fifo[69];
            tx_fifo[68] <= tx_fifo[68];
            tx_fifo[67] <= tx_fifo[67];
            tx_fifo[66] <= tx_fifo[66];
            tx_fifo[65] <= tx_fifo[65];
            tx_fifo[64] <= tx_fifo[64];
            tx_fifo[63] <= tx_fifo[63];
            tx_fifo[62] <= tx_fifo[62];
            tx_fifo[61] <= tx_fifo[61];
            tx_fifo[60] <= tx_fifo[60];
            tx_fifo[59] <= tx_fifo[59];
            tx_fifo[58] <= tx_fifo[58];
            tx_fifo[57] <= tx_fifo[57];
            tx_fifo[56] <= tx_fifo[56];
            tx_fifo[55] <= tx_fifo[55];
            tx_fifo[54] <= tx_fifo[54];
            tx_fifo[53] <= tx_fifo[53];
            tx_fifo[52] <= tx_fifo[52];
            tx_fifo[51] <= tx_fifo[51];
            tx_fifo[50] <= tx_fifo[50];
            tx_fifo[49] <= tx_fifo[49];
            tx_fifo[48] <= tx_fifo[48];
            tx_fifo[47] <= tx_fifo[47];
            tx_fifo[46] <= tx_fifo[46];
            tx_fifo[45] <= tx_fifo[45];
            tx_fifo[44] <= tx_fifo[44];
            tx_fifo[43] <= tx_fifo[43];
            tx_fifo[42] <=  tx_fifo[42];
            tx_fifo[41] <=  tx_fifo[41];
            tx_fifo[40] <= tx_fifo[40];
            tx_fifo[39] <= tx_fifo[39];
            tx_fifo[38] <= tx_fifo[38];
            tx_fifo[37] <= tx_fifo[37];
            tx_fifo[36] <= tx_fifo[36];
            tx_fifo[35] <= tx_fifo[35];
            tx_fifo[34] <= tx_fifo[34];
            tx_fifo[33] <= tx_fifo[33];
            tx_fifo[32] <= tx_fifo[32];
            tx_fifo[31] <= tx_fifo[31];
            tx_fifo[30] <= tx_fifo[30];
            tx_fifo[29] <= tx_fifo[29];
            tx_fifo[28] <= tx_fifo[28];
            tx_fifo[27] <= tx_fifo[27];
            tx_fifo[26] <= tx_fifo[26];
            tx_fifo[25] <= tx_fifo[25];
            tx_fifo[24] <= tx_fifo[24];
            tx_fifo[23] <= tx_fifo[23];
            tx_fifo[22] <= tx_fifo[22];
            tx_fifo[21] <= tx_fifo[21];
            tx_fifo[20] <= tx_fifo[20];
            tx_fifo[19] <= tx_fifo[19];
            tx_fifo[18] <= tx_fifo[18];
            tx_fifo[17] <= tx_fifo[17];
            tx_fifo[16] <= tx_fifo[16];
            tx_fifo[15] <= tx_fifo[15];
            tx_fifo[14] <= tx_fifo[14];
            tx_fifo[13] <= tx_fifo[13];
            tx_fifo[12] <= tx_fifo[12];
            tx_fifo[11] <= tx_fifo[11];
            tx_fifo[10] <= tx_fifo[10];
            tx_fifo[9] <= tx_fifo[9];
            tx_fifo[8] <= tx_fifo[8];
            tx_fifo[7] <= tx_fifo[7];
            tx_fifo[6] <= tx_fifo[6];
            tx_fifo[5] <= tx_fifo[5];
            tx_fifo[4] <= tx_fifo[4];
            tx_fifo[3] <= tx_fifo[3];
            tx_fifo[2] <= tx_fifo[2];
            tx_fifo[1] <= tx_fifo[1];
            tx_fifo[0] <= tx_fifo[0]; 


 	    rx_fifo[127] <= rx_fifo[127];
            rx_fifo[126] <= rx_fifo[126];
            rx_fifo[125] <= rx_fifo[125];
            rx_fifo[124] <= rx_fifo[124];
            rx_fifo[123] <= rx_fifo[123];
            rx_fifo[122] <= rx_fifo[122];
            rx_fifo[121] <= rx_fifo[121];
            rx_fifo[120] <= rx_fifo[120];
            rx_fifo[119] <= rx_fifo[119];
            rx_fifo[118] <=  rx_fifo[118];
            rx_fifo[117] <= rx_fifo[117];
            rx_fifo[116] <= rx_fifo[116];
            rx_fifo[115] <=  rx_fifo[115];
            rx_fifo[114] <= rx_fifo[114];
            rx_fifo[113] <= rx_fifo[113];
            rx_fifo[112] <= rx_fifo[112];
            rx_fifo[111] <= rx_fifo[111];
            rx_fifo[110] <= rx_fifo[110];
            rx_fifo[109] <= rx_fifo[109];
            rx_fifo[108] <= rx_fifo[108];
            rx_fifo[107] <= rx_fifo[107];
            rx_fifo[106] <= rx_fifo[106];
            rx_fifo[105] <= rx_fifo[105];
            rx_fifo[104] <= rx_fifo[104];
            rx_fifo[103] <= rx_fifo[103];
            rx_fifo[102] <= rx_fifo[102];
            rx_fifo[101] <= rx_fifo[101];
            rx_fifo[100] <= rx_fifo[100];
            rx_fifo[99] <= rx_fifo[99];
            rx_fifo[98] <= rx_fifo[98];
            rx_fifo[97] <= rx_fifo[97];
            rx_fifo[96] <= rx_fifo[96];
            rx_fifo[95] <= rx_fifo[95];
            rx_fifo[94] <= rx_fifo[94];
            rx_fifo[93] <= rx_fifo[93];
            rx_fifo[92] <= rx_fifo[92];
            rx_fifo[91] <= rx_fifo[91];
            rx_fifo[90] <= rx_fifo[90];
            rx_fifo[89] <= rx_fifo[89];
            rx_fifo[88] <= rx_fifo[88];
            rx_fifo[87] <= rx_fifo[87];
            rx_fifo[86] <= rx_fifo[86];
            rx_fifo[85] <= rx_fifo[85];
            rx_fifo[84] <= rx_fifo[84];
            rx_fifo[83] <= rx_fifo[83];
            rx_fifo[82] <= rx_fifo[82];
            rx_fifo[81] <= rx_fifo[81];
            rx_fifo[80] <= rx_fifo[80];
            rx_fifo[79] <= rx_fifo[79];
            rx_fifo[78] <= rx_fifo[78];
            rx_fifo[77] <= rx_fifo[77];
            rx_fifo[76] <= rx_fifo[76];
            rx_fifo[75] <= rx_fifo[75];
            rx_fifo[74] <= rx_fifo[74];
            rx_fifo[73] <= rx_fifo[73];
            rx_fifo[72] <= rx_fifo[72];
            rx_fifo[71] <= rx_fifo[71];
            rx_fifo[70] <= rx_fifo[70];
            rx_fifo[69] <= rx_fifo[69];
            rx_fifo[68] <= rx_fifo[68];
            rx_fifo[67] <= rx_fifo[67];
            rx_fifo[66] <= rx_fifo[66];
            rx_fifo[65] <= rx_fifo[65];
            rx_fifo[64] <= rx_fifo[64];
            rx_fifo[63] <= rx_fifo[63];
            rx_fifo[62] <= rx_fifo[62];
            rx_fifo[61] <= rx_fifo[61];
            rx_fifo[60] <= rx_fifo[60];
            rx_fifo[59] <= rx_fifo[59];
            rx_fifo[58] <= rx_fifo[58];
            rx_fifo[57] <= rx_fifo[57];
            rx_fifo[56] <= rx_fifo[56];
            rx_fifo[55] <= rx_fifo[55];
            rx_fifo[54] <= rx_fifo[54];
            rx_fifo[53] <= rx_fifo[53];
            rx_fifo[52] <= rx_fifo[52];
            rx_fifo[51] <= rx_fifo[51];
            rx_fifo[50] <= rx_fifo[50];
            rx_fifo[49] <= rx_fifo[49];
            rx_fifo[48] <= rx_fifo[48];
            rx_fifo[47] <= rx_fifo[47];
            rx_fifo[46] <= rx_fifo[46];
            rx_fifo[45] <= rx_fifo[45];
            rx_fifo[44] <= rx_fifo[44];
            rx_fifo[43] <= rx_fifo[43];
            rx_fifo[42] <=  rx_fifo[42];
            rx_fifo[41] <=  rx_fifo[41];
            rx_fifo[40] <= rx_fifo[40];
            rx_fifo[39] <= rx_fifo[39];
            rx_fifo[38] <= rx_fifo[38];
            rx_fifo[37] <= rx_fifo[37];
            rx_fifo[36] <= rx_fifo[36];
            rx_fifo[35] <= rx_fifo[35];
            rx_fifo[34] <= rx_fifo[34];
            rx_fifo[33] <= rx_fifo[33];
            rx_fifo[32] <= rx_fifo[32];
            rx_fifo[31] <= rx_fifo[31];
            rx_fifo[30] <= rx_fifo[30];
            rx_fifo[29] <= rx_fifo[29];
            rx_fifo[28] <= rx_fifo[28];
            rx_fifo[27] <= rx_fifo[27];
            rx_fifo[26] <= rx_fifo[26];
            rx_fifo[25] <= rx_fifo[25];
            rx_fifo[24] <= rx_fifo[24];
            rx_fifo[23] <= rx_fifo[23];
            rx_fifo[22] <= rx_fifo[22];
            rx_fifo[21] <= rx_fifo[21];
            rx_fifo[20] <= rx_fifo[20];
            rx_fifo[19] <= rx_fifo[19];
            rx_fifo[18] <= rx_fifo[18];
            rx_fifo[17] <= rx_fifo[17];
            rx_fifo[16] <= rx_fifo[16];
            rx_fifo[15] <= rx_fifo[15];
            rx_fifo[14] <= rx_fifo[14];
            rx_fifo[13] <= rx_fifo[13];
            rx_fifo[12] <= rx_fifo[12];
            rx_fifo[11] <= rx_fifo[11];
            rx_fifo[10] <= rx_fifo[10];
            rx_fifo[9] <= rx_fifo[9];
            rx_fifo[8] <= rx_fifo[8];
            rx_fifo[7] <= rx_fifo[7];
            rx_fifo[6] <= rx_fifo[6];
            rx_fifo[5] <= rx_fifo[5];
            rx_fifo[4] <= rx_fifo[4];
            rx_fifo[3] <= rx_fifo[3];
            rx_fifo[2] <= rx_fifo[2];
            rx_fifo[1] <= rx_fifo[1];
            rx_fifo[0] <= rx_fifo[0]; 
	    end
            

   end

    
            
    
      
endmodule
