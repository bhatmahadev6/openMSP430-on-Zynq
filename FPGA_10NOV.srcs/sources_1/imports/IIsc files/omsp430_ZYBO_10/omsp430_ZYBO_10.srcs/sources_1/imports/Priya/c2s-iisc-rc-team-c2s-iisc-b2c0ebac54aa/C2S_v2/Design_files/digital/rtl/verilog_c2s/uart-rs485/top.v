//`timescale 1ns/10ps
//`include "/home/smart/revised_rtl_jul18/rtl/verilog_c2s/uart-rs485/uart_defines.v"
`include "uart_defines.v"


module uart_top	(
	clk_i, proc_ring_i,

	rst, adr_i, dat_i, dat_o, per_en, we_i,

	tx_o, rx_i, en_proc_ring_o, imp_ctrl_2, imp_ctrl_1, imp_ctrl_0, transmitted, received
`ifdef UART_HAS_BAUDRATE_OUTPUT
	, baud_o
`endif
	);

parameter 							 uart_data_width = `UART_DATA_WIDTH;
parameter 							 uart_addr_width = `UART_ADDR_WIDTH;

input 								 clk_i;
input 								 proc_ring_i;

input 								 rst;
input [uart_addr_width-1:0] 	 adr_i;
input [uart_data_width-1:0] 	 dat_i;
output [15:0] 	 dat_o;
input 						     per_en;
input 	[1:0]					 we_i;

input 								 rx_i;
output 								 tx_o;
output                              en_proc_ring_o;
output                         imp_ctrl_0;
output                         imp_ctrl_1;
output                         imp_ctrl_2;
output                         transmitted;
output                         received;

wire [7:0] data_o;
wire transmitted;
wire received;
wire we_int;
wire re_int;

assign we_int = (|we_i ) && per_en;
assign re_int = (~(|we_i)) && per_en;//assign re_int = (~|we_i) && per_en;


assign dat_o={8'h00, data_o};

// optional baudrate output
`ifdef UART_HAS_BAUDRATE_OUTPUT
output	baud_o;
`endif

uart_regs	regs(
	.clk(clk_i),
	.Rclk_i(proc_ring_i),
	.wb_rst_i(	rst	),
	.wb_dat_o(	data_o	),
    .wb_we_i (	we_int	),
    .wb_re_i (   re_int),
    .wb_addr_i(	adr_i	),
    .wb_dat_i (    dat_i[7:0]    ),    
	.stx_pad_o(		tx_o		),
	.srx_pad_i(		rx_i		),
	.imp_o(	{imp_ctrl_2,imp_ctrl_1,imp_ctrl_0}),
	.int_o(		int_o		),
	.transmitted(transmitted),
	.received(received)
`ifdef UART_HAS_BAUDRATE_OUTPUT
	, .baud_o(baud_o)
`endif
);

assign en_proc_ring_o = ~rst;


endmodule

