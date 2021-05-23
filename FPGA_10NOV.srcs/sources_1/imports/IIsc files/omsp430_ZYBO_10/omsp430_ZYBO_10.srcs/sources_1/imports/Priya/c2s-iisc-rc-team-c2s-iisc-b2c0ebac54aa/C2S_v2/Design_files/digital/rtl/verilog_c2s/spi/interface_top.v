`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/18/2016 05:30:24 PM
// Design Name: 
// Module Name: interface_top
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


module interface_top(
    // OUTPUTs to core
        flash_dout,                      // flash data output
        flash_busy,                      // status of flash
        per_dout,
        
    // INPUTs from core
        flash_addr,                      // flash address
        flash_cen,                       // flash chip enable (low active)
        flash_clk_50,                       // flash clock - 50MHz
	flash_clk_25,                       // flash clock - 25MHz
        flash_rst,                       // flash_interface reset
        flash_din,                       // flash data input
        flash_wen,                       // flash write enable (low active)
	flash_oen, 
        done,                            // tells reading is finished
        per_addr,       // Peripheral address
        per_din,        // Peripheral data input
        per_en,         // Peripheral enable (high active)
        per_we,         // Peripheral write enable (high active)

    // OUTPUTs to flash
        sclk,                            // flash data output
        cs_n,
        din,
    
    // INPUTs from flash
        dout,  
        en ,
        data_width , busy_normal  
        
    );
    
    
    // OUTPUTs to core
    //============
    output   [15:0] flash_dout;          // flash data output
    output   flash_busy; 
     output busy_normal;                // Valid data sending
    output   [15:0] per_dout;        // Peripheral data output
    
    // INPUTs from core
    //============
    input [15:0] flash_addr;       // flash address
    input              flash_cen;        // flash chip enable (low active)
    input              flash_clk_50;        // flash clock -50 MHz
    input              flash_clk_25;        // flash clock -25 MHz
    input              flash_rst;        // flash_interface reset
    input       [15:0] flash_din;        // flash data input
    input              flash_wen;        // flash write enable (low active)
    input 		flash_oen;
    input              done; 
    input        [13:0] per_addr;       // Peripheral address       
    input        [15:0] per_din;        // Peripheral data input
    input               per_en;         // Peripheral enable (high active)    // OUTPUTs to flash
    input        [1:0]  per_we;         // Peripheral write enable (high active)output             sclk;                            // flash data output
    output             cs_n,sclk;
    output           din;//output           reg  din;
     input en;     
     // INPUTs from flash
     input             dout;
     
     wire [15:0]      FMA_low;
     wire [15:0]      FMA_high;
     wire [15:0]      FMC;  
     
     
     wire [15:0]      tx_fifo_cur;  
     wire [15:0]      rx_fifo_cur;
     
     
//     wire              we_sfr;
     wire              we_tx_fifo;
     wire              we_rx_fifo;
     wire              rd_tx_fifo;
     wire              rd_rx_fifo;
     wire [15:0]       data;
     wire [15:0]       addr;
     wire [7:0]        fifo_addr;
     //wire [7:0] count1; //wire [3:0]        count1;
     wire [7:0] count1;
     wire [3:0]        state;
     wire [15:0]      rddata;
     wire  din_1;
     input [6:0] data_width;
     wire cs; //reg cs;
     reg clk1;
     assign cs_n = ~cs;
    
  /*  always@(posedge flash_clk)
     begin
        if(~flash_rst)
          clk1<= 1'b1;
        else clk1 <= ~(clk1);
     end*/
  
  /* always@(posedge flash_clk_25) //always@(negedge flash_clk_25)//always@(posedge clk1)
     begin
	din<= din_1;//din<= #2 din_1;
     end
   */ 
    
    
       
       
spi_intermediate spi_intermediate(.clk_spi(flash_clk_25),.rst(flash_rst), .flash_cen(flash_cen), .flash_wen(flash_wen), .FMC(FMC),
                  .FMA_high(FMA_high),  .FMA_low(FMA_low), .data_in(tx_fifo_cur),
                  .flash_data_out(dout), .done(done), .state(state), .count1(count1), 
                  .busy(flash_busy), .wr_tx_fifo(we_tx_fifo), .rd_tx_fifo(rd_tx_fifo), .wr_rx_fifo(we_rx_fifo), .rd_rx_fifo(rd_rx_fifo), .flash_en(cs),
                  .flash_clk(sclk), .flash_data_in(din), .fifo_addr(fifo_addr) ,.enable(en),.rddata(rddata),.data_width(data_width),.busy_normal(busy_normal),
                   .flash_addr(flash_addr)

                );



       
       
sfr flash_sfr(
	.clk_sfr50(flash_clk_50),
	.clk_sfr25(flash_clk_25),
	.rst(flash_rst),
	.per_en(per_en),
	.we_en(|per_we),
	.we_tx_fifo(we_tx_fifo),
	.we_rx_fifo(we_rx_fifo),
	.rd_tx_fifo(rd_tx_fifo),
	.rd_rx_fifo(rd_rx_fifo),
	.addr({1'b0,per_addr,1'b0}),
	.data(we_rx_fifo ? rddata : flash_din),
//	.data(flash_din),
	.per_data(per_din),
	.fifo_addr(fifo_addr),
	.FMA_low(FMA_low),
	.FMA_high(FMA_high),
	.FMC(FMC),
	.tx_fifo_cur(tx_fifo_cur),
	.rx_fifo_cur(flash_dout),
	.per_dout(per_dout)
                 );
      

    
       
    
    
    
    
endmodule

