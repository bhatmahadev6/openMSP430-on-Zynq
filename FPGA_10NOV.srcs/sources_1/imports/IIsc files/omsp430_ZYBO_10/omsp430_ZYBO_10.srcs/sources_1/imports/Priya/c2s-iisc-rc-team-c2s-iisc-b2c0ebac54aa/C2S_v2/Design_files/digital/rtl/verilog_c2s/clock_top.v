`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01.02.2017 12:34:03
// Design Name: 
// Module Name: clock_divider
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


module clock_top(
	clk_in, rst, clk_100M, clk_50M,clk_25M, clk_48K, clk_32K, rst_o
	);


input clk_in; //input clock of 100Mhz frequency
input rst; //Active high reset input
output clk_100M; //output clock of 100MHz frequecy
output reg clk_50M; //output clock of 50MHz frequecy
output clk_25M;//output clock of 25MHz frequecy
output clk_48K; //output clock of 48KHz frequecy
output clk_32K; //output clock of 32KHz frequecy
output rst_o; //reset for all modules

reg [10:0] count;
reg [2:0]  edge_count;
parameter noclk = 40;  //better give even number
reg rst_d0;            //Delayed reset
reg rst_int;
reg [noclk-1:0] ff;

//--------------50MHz clock--------------
always @(posedge clk_in or posedge rst)
begin
  if (rst)
  begin
	count <= 1'b0;
	clk_50M <= 1'b0;
  end
  else
  begin
    count <= count+1;
    clk_50M <= count[0];
  end
end

assign clk_100M = clk_in;       //100MHz clock

//--------------25MHz clock------------
assign clk_25M = count[1];  

//-------------48.8KHz clock------------
assign clk_48K = count[10];  //48.8KHz

//-------------32khz clock--------------
always @(posedge count[8] or posedge rst)
begin
    if(rst)
        begin
            edge_count <= 3'd1;
        end
    else
        begin
            if(edge_count == 3'd6)
                edge_count <= 3'd1;
            else
                edge_count <= edge_count + 1;
        end
end

assign clk_32K = edge_count[2];


always @(posedge clk_in)
begin
    rst_d0 <= rst;              //Synchronized reset input
    rst_int <= (rst_d0 || ((|ff) && (~rst_d0)));

end
//-------------Reset Generation---------
always @(posedge clk_in or posedge rst)
begin
    if(rst)
        begin
        ff <= 0;
        end
    else
        begin
        ff <= {ff[noclk-2:0],rst_d0};
        end
end

assign rst_o = rst_int;

endmodule
