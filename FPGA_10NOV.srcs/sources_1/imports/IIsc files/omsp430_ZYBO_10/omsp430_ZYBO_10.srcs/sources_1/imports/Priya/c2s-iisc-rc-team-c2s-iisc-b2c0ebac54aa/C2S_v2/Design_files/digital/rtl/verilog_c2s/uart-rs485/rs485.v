//`timescale 1ns/10ps



module rs485_interface_module	(
	Rclk_i, clk_i, rst_i, imp_ctrl_o
	);

//parameter 							 uart_data_width = `UART_DATA_WIDTH;
//parameter 							 uart_addr_width = `UART_ADDR_WIDTH;

input 								 Rclk_i;
input 								 clk_i;
input 								 rst_i;
output reg [2:0] imp_ctrl_o;
//output en_proc_ring_o;

reg dsync_o;
reg dsync;

wire en;
reg [5:0] count_value;
reg [1:0] count_clk;  
reg [5:0] count;


//-------Double Stage Synchronizer----------
always @(posedge Rclk_i or posedge rst_i)
begin
  if (rst_i)
  begin
	dsync <= 1'b0;
  end
  else
    dsync <= en;
end

always @(posedge Rclk_i or posedge rst_i)
begin
  if (rst_i)
  begin
	dsync_o <= 1'b0;
  end
  else
    dsync_o <= dsync;
end
//-------------End of DSS------------------

//----------12.5MHz clock generation---------
always @(posedge clk_i or posedge rst_i)
begin
    if (rst_i)
      count_clk <= 0;
    else
      count_clk <= count_clk + 1;
end

assign en = count_clk[1];

//---------------Counter-------------------
always @(posedge Rclk_i or posedge rst_i)
begin
    if (rst_i)
      count <= 0;
    else if(dsync_o)
      count <= count + 1; 
    //else if((~dsync_o)&(~count_clk[0]))
     else if((~dsync_o)&(count_clk[0]))
      count <= 0;
     else
      count<=count;
end

//--------------Impedence Control----------
always @(negedge dsync_o or posedge rst_i)
begin
    if (rst_i)
      count_value <= 0;
    else
      count_value <= count; 
end

////--------------Impedence Control----------
//always @(negedge count_clk[0] or posedge rst_i)
//begin
//    if (rst_i)
//      count_value <= 0;
//    else if(~count_clk[1])
//      count_value <= count; 
//end

//---------------imp_ctrl-----------------
always @(negedge count_clk[0] or posedge rst_i)
begin
    if (rst_i)
      imp_ctrl_o <= 0;
    else
      if(count_value>=30 && count_value<=42)
        imp_ctrl_o <= 3'b011;
      else if(count_value>=15 && count_value<=29)
        imp_ctrl_o <= 3'b111;
      else if(count_value>=43 /* && count_value<=63*/)
        imp_ctrl_o <= 3'b001;
      else
	imp_ctrl_o <= imp_ctrl_o;
end

endmodule
