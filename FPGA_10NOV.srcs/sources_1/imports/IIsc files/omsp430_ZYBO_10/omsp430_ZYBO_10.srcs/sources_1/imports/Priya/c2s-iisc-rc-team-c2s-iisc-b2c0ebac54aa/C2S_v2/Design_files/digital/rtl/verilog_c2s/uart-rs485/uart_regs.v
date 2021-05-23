//`timescale 1ns/10ps
//`include "/home/smart/Desktop/C2S_projects/UART_Viv/RTL/uart_defines.v"
//`include "uart_defines.v"
//`include "/home/smart/revised_rtl_jul18/rtl/verilog_c2s/uart-rs485/uart_defines.v"

`define UART_DL1 7:0
`define UART_DL2 15:8

module uart_regs (clk,Rclk_i,
	wb_rst_i, wb_addr_i, wb_dat_i, wb_dat_o, wb_we_i, wb_re_i, 

// additional signals
//	modem_inputs,
	stx_pad_o, srx_pad_i,imp_o,				
//	rts_pad_o, dtr_pad_o,
	 int_o, transmitted, received
`ifdef UART_HAS_BAUDRATE_OUTPUT
	, baud_o
`endif
);


input 									clk;
input                                   Rclk_i; //Ring oscillator clock
input 									wb_rst_i;
input [`UART_ADDR_WIDTH-1:0]     		wb_addr_i;
input [7:0] 							wb_dat_i;
output [7:0]    					wb_dat_o;
input 									wb_we_i;
input 									wb_re_i;

output 									stx_pad_o;
input 									srx_pad_i;

output [2:0]                            imp_o;
//input [3:0] 							modem_inputs;
//output 									rts_pad_o;
//output 									dtr_pad_o;
output 									int_o;
output 									transmitted;
output 									received;
//output [15:0]  dl;

`ifdef UART_HAS_BAUDRATE_OUTPUT
output	baud_o;
`endif


wire [3:0] 								modem_inputs;
reg 										enable;
`ifdef UART_HAS_BAUDRATE_OUTPUT
assign baud_o = enable; // baud_o is actually the enable signal
`endif


wire 										stx_pad_o;		// received from transmitter module
wire 										srx_pad_i;
wire 										srx_pad;

reg [7:0] 								wb_dat_o;

wire [`UART_ADDR_WIDTH-1:0] 		wb_addr_i;
wire [7:0] 								wb_dat_i;
reg [7:0] 								din;
reg [3:0] 								ier;
reg [2:0]                               icr;
reg [3:0] 								iir;
reg [1:0] 								fcr;  /// bits 7 and 6 of fcr. Other bits are ignored
//reg [4:0] 								mcr;
reg [7:0] 								lcr;
//reg [7:0] 								msr;
reg [15:0] 								dl;  // 32-bit divisor latch
//reg [7:0] 								scratch; // UART scratch register
reg 										start_dlc; // activate dlc on writing to UART_DL1
reg 										lsr_mask_d; // delay for lsr_mask condition
//reg 										msi_reset; // reset MSR 4 lower bits indicator
//reg 										threi_clear; // THRE interrupt clear flag
reg [15:0] 								dlc;  // 32-bit divisor latch counter
reg 										int_o;

reg [7:0] 								trigger_level; // trigger level of the receiver FIFO
reg 										rx_reset;
reg 										tx_reset;

wire 										dlab;			   // divisor latch access bit
//wire 										cts_pad_i, dsr_pad_i, ri_pad_i, dcd_pad_i; // modem status bits
//wire 										loopback;		   // loopback bit (MCR bit 4)
//wire 										cts, dsr, ri, dcd;	   // effective signals
//wire                    cts_c, dsr_c, ri_c, dcd_c; // Complement effective signals (considering loopback)
//wire 										rts_pad_o, dtr_pad_o;		   // modem control outputs

// LSR bits wires and regs
wire [7:0] 								lsr;
wire 										lsr0, lsr1, lsr2, lsr3, lsr4, lsr5, lsr6, lsr7;
reg										lsr0r, lsr1r, lsr2r, lsr3r, lsr4r, lsr5r, lsr6r, lsr7r;
wire 										lsr_mask; // lsr_mask

//
// ASSINGS
//

assign 									lsr[7:0] = { lsr7r, lsr6r, lsr5r, lsr4r, lsr3r, lsr2r, lsr1r, lsr0r };

//assign 									{cts_pad_i, dsr_pad_i, ri_pad_i, dcd_pad_i} = modem_inputs;
//assign 									{cts, dsr, ri, dcd} = ~{cts_pad_i,dsr_pad_i,ri_pad_i,dcd_pad_i};

//assign                  {cts_c, dsr_c, ri_c, dcd_c} = loopback ? {mcr[`UART_MC_RTS],mcr[`UART_MC_DTR],mcr[`UART_MC_OUT1],mcr[`UART_MC_OUT2]}
//                                                               : {cts_pad_i,dsr_pad_i,ri_pad_i,dcd_pad_i};

assign 									dlab = lcr[`UART_LC_DL];
assign                                  imp_o = icr[2:0];
//assign 									loopback = mcr[4];

// assign modem outputs
//assign 									rts_pad_o = mcr[`UART_MC_RTS];
//assign 									dtr_pad_o = mcr[`UART_MC_DTR];

// Interrupt signals
wire 										rls_int;  // receiver line status interrupt
wire 										rda_int;  // receiver data available interrupt
wire 										ti_int;   // timeout indicator interrupt
wire										thre_int; // transmitter holding register empty interrupt
wire 										ms_int;   // modem status interrupt

// FIFO signals
reg 										tf_push;
reg 										rf_pop;
wire [`UART_FIFO_REC_WIDTH-1:0] 	rf_data_out;
wire 										rf_error_bit; // an error (parity or framing) is inside the fifo
wire [`UART_FIFO_COUNTER_W-1:0] 	rf_count;
wire [`UART_FIFO_COUNTER_W-1:0] 	tf_count;
wire [2:0] 								tstate;
wire [3:0] 								rstate;
wire [9:0] 								counter_t;

wire                      thre_set_en; // THRE status is delayed one character time when a character is written to fifo.
reg  [7:0]                block_cnt;   // While counter counts, THRE status is blocked (delayed one character cycle)
reg  [7:0]                block_value; // One character length minus stop bit

// Transmitter Instance
wire serial_out;
wire [7:0] rf_data_o;
wire [2:0] imp_ctrl_o;


uart_transmitter transmitter(clk, wb_rst_i, lcr, /*rf_push_pulse,rf_data_o*/tf_push,  din, enable, serial_out, tstate, tf_count, tx_reset, lsr_mask);
//uart_transmitter transmitter(clk, wb_rst_i, lcr, /*rf_push_pulse,rf_data_o*/tf_push, wb_dat_i , enable, serial_out, tstate, tf_count, tx_reset, lsr_mask);

rs485_interface_module rs485(Rclk_i,clk,wb_rst_i,imp_ctrl_o);

  // Synchronizing and sampling serial RX input
  uart_sync_flops    i_uart_sync_flops
  (
    .rst_i           (wb_rst_i),
    .clk_i           (clk),
    .stage1_rst_i    (1'b0),
    .stage1_clk_en_i (1'b1),
    .async_dat_i     (srx_pad_i),
    .sync_dat_o      (srx_pad)
  );
  defparam i_uart_sync_flops.width      = 1;
  defparam i_uart_sync_flops.init_value = 1'b1;

// handle loopback
wire serial_in = /*loopback ? serial_out :*/ srx_pad;
assign stx_pad_o = /*loopback ? 1'b1 :*/ serial_out;

// Receiver Instance
uart_receiver receiver(clk, wb_rst_i, lcr, rf_pop, serial_in, enable, 
	counter_t, rf_count, rf_data_out, rf_error_bit, rf_overrun, rx_reset, lsr_mask, rstate, rf_push_pulse);


// Asynchronous reading here because the outputs are sampled in uart_wb.v file 
//always @(dl or dlab or ier or iir /*or scratch*/
//			or lcr or lsr or icr or rf_data_out or wb_addr_i or wb_re_i)   // asynchrounous reading
always @(posedge clk or posedge wb_rst_i)
begin
    if (wb_rst_i)
       wb_dat_o <= 8'h00;
    else if (wb_re_i)
    begin
	case (wb_addr_i)
		`UART_REG_RB    : wb_dat_o  <= dlab ? dl[`UART_DL1] : rf_data_out[10:3];
		`UART_REG_IE	: wb_dat_o <= dlab ? dl[`UART_DL2] : ier;
		`UART_REG_II	: wb_dat_o <= {4'b1100,iir};
		`UART_REG_LC	: wb_dat_o <= lcr;
		`UART_REG_LS	: wb_dat_o <= lsr;
		`UART_REG_IMP	: wb_dat_o <= {5'b00000,icr};
	//	`UART_REG_SR	: wb_dat_o <= scratch;
		default:  wb_dat_o <= 8'b00000000; // ??
	endcase // case(wb_addr_i)
	end
    else wb_dat_o <= wb_dat_o;
end // always @ (dl or dlab or ier or iir or scratch...

// rf_pop signal handling
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		rf_pop <=  1'b0; 
	else
	if (rf_pop)	// restore the signal to 0 after one clock cycle
		rf_pop <=  1'b0;
	else
	if (wb_re_i && wb_addr_i == `UART_REG_RB && !dlab)
		rf_pop <= 1'b1; // advance read pointer

	else rf_pop <= rf_pop;
end

wire 	lsr_mask_condition;
wire 	iir_read;
//wire  msr_read;
wire	fifo_read;
wire	fifo_write;

assign lsr_mask_condition = (wb_re_i && wb_addr_i == `UART_REG_LS && !dlab);
assign iir_read = (wb_re_i && wb_addr_i == `UART_REG_II && !dlab);
//assign msr_read = (wb_re_i && wb_addr_i == `UART_REG_MS && !dlab);
assign fifo_read = (wb_re_i && wb_addr_i == `UART_REG_RB && !dlab);
assign fifo_write = (wb_we_i && wb_addr_i == `UART_REG_TR && !dlab);

// lsr_mask_d delayed signal handling
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		lsr_mask_d <= 1'b0;
	else // reset bits in the Line Status Register
		lsr_mask_d <= lsr_mask_condition;
end

// lsr_mask is rise detected
assign lsr_mask = lsr_mask_condition && ~lsr_mask_d;

// msi_reset signal handling
//always @(posedge clk or posedge wb_rst_i)
//begin
//	if (wb_rst_i)
//		msi_reset <= #1 1;
//	else
//	if (msi_reset)
//		msi_reset <= #1 0;
//	else
//	if (msr_read)
//		msi_reset <= #1 1; // reset bits in Modem Status Register
//end


//
//   WRITES AND RESETS   //
//
// Line Control Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		lcr <= 8'b00000011; // 8n1 setting
	else
	if (wb_we_i && wb_addr_i==`UART_REG_LC)
		lcr <= wb_dat_i;
	else 
		lcr <= lcr;

//Impedance control Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
	   icr <= 3'b000; // 8n1 setting
	else if (wb_we_i && (~lcr[0]) && wb_addr_i==`UART_REG_IMP)
		icr <= wb_dat_i[2:0];
    else if(lcr[0])
        icr <= imp_ctrl_o;

	else icr <= icr;




// Interrupt Enable Register or UART_DL2
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
	begin
		ier <= 4'b0000; // no interrupts after reset
		dl[`UART_DL2] <= 8'b00000000 ;//8'h00;
	end
	else
	if (wb_we_i && wb_addr_i==`UART_REG_IE)
		if (dlab)
		begin
			dl[`UART_DL2] <= wb_dat_i;
			ier <= ier;
		end
		else
			begin
			ier <= wb_dat_i[3:0]; // ier uses only 4 lsb
			dl[`UART_DL2]<=dl[`UART_DL2];
			end

	else 
		begin
		ier <= ier;
		dl[`UART_DL2]<=dl[`UART_DL2];
		end

// FIFO Control Register and rx_reset, tx_reset signals
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) begin
		fcr <= 2'b11; 
		rx_reset <= 1'b0;
		tx_reset <= 1'b0;
	end else
	if (wb_we_i && wb_addr_i==`UART_REG_FC) begin
		fcr <= wb_dat_i[7:6];
		rx_reset <= wb_dat_i[1];
		tx_reset <= wb_dat_i[2];
	end else begin
		rx_reset <= 1'b0;
		tx_reset <= 1'b0;
		fcr <= fcr;
	end

// Modem Control Register
/*always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		mcr <= #1 5'b0; 
	else
	if (wb_we_i && wb_addr_i==`UART_REG_MC)
			mcr <= #1 wb_dat_i[4:0];
*/

// Scratch register
// Line Control Register
//always @(posedge clk or posedge wb_rst_i)
//	if (wb_rst_i)
//		scratch <= #1 0; // 8n1 setting
//	else
//	if (wb_we_i && wb_addr_i==`UART_REG_SR)
//		scratch <= #1 wb_dat_i;

// TX_FIFO or UART_DL1

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
	begin
		dl[`UART_DL1]  <= 8'h01;//8'd1; //8'd00
		tf_push   <= 1'b0;
		din	  <= 8'b00000000;
		start_dlc <= 1'b0;
	end
	else
	if (wb_we_i && wb_addr_i==`UART_REG_TR)
		if (dlab)
		begin
			dl[`UART_DL1] <= wb_dat_i;
			start_dlc <= 1'b1; // enable DL counter
			din	  <= wb_dat_i;
			tf_push <= 1'b0;
		end
		else
		begin
			tf_push   <= 1'b1;
			din	  <= wb_dat_i;
			start_dlc <= 1'b0;
			dl[`UART_DL1] <= dl[`UART_DL1];
		end // else: !if(dlab)
	else
	begin
		start_dlc <= 1'b0;
		tf_push   <= 1'b0; //1'b0;
		dl[`UART_DL1] <= dl[`UART_DL1];
		din	  <= wb_dat_i;
	end // else: !if(dlab)

// Receiver FIFO trigger level selection logic (asynchronous mux)
always @(fcr or wb_rst_i)
	if (wb_rst_i)
		trigger_level = 1;
	else
	case (fcr[`UART_FC_TL])
		2'b00 : trigger_level = 1;
		2'b01 : trigger_level = 4;
		2'b10 : trigger_level = 30;
		2'b11 : trigger_level = 250;
		default : trigger_level = trigger_level;
	endcase // case(fcr[`UART_FC_TL])
	
//
//  STATUS REGISTERS  //
//

// Modem Status Register
/*reg [3:0] delayed_modem_signals;
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
	  begin
  		msr <= #1 0;
	  	delayed_modem_signals[3:0] <= #1 0;
	  end
	else begin
		msr[`UART_MS_DDCD:`UART_MS_DCTS] <= #1 msi_reset ? 4'b0 :
			msr[`UART_MS_DDCD:`UART_MS_DCTS] | ({dcd, ri, dsr, cts} ^ delayed_modem_signals[3:0]);
		msr[`UART_MS_CDCD:`UART_MS_CCTS] <= #1 {dcd_c, ri_c, dsr_c, cts_c};
		delayed_modem_signals[3:0] <= #1 {dcd, ri, dsr, cts};
	end
end
*/

// Line Status Register

// activation conditions
assign lsr0 = (rf_count==9'h000 && rf_push_pulse);  // data in receiver fifo available set condition
assign lsr1 = rf_overrun;     // Receiver overrun error
assign lsr2 = rf_data_out[1]; // parity error bit
assign lsr3 = rf_data_out[0]; // framing error bit
assign lsr4 = rf_data_out[2]; // break error in the character
assign lsr5 = (tf_count==5'b00000 && thre_set_en);  // transmitter fifo is empty
assign lsr6 = (tf_count==5'b00000 && thre_set_en && (tstate == /*`S_IDLE */ 0)); // transmitter empty
assign lsr7 = rf_error_bit | rf_overrun;

// lsr bit0 (receiver data available)
reg 	 lsr0_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr0_d <= 1'b0;
	else lsr0_d <= lsr0;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr0r <= 1'b0;
	else lsr0r <= (rf_count==9'h001 && rf_pop && !rf_push_pulse || rx_reset) ? 1'b0 : // deassert condition
					  lsr0r || (lsr0 && ~lsr0_d); // set on rise of lsr0 and keep asserted until deasserted 

// lsr bit 1 (receiver overrun)
reg lsr1_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr1_d <= 1'b0;
	else lsr1_d <= lsr1;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr1r <= 1'b0;
	else	lsr1r <= lsr_mask ? 1'b0 : lsr1r || (lsr1 && ~lsr1_d); // set on rise

// lsr bit 2 (parity error)
reg lsr2_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr2_d <= 1'b0;
	else lsr2_d <= lsr2;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr2r <= 1'b0;
	else lsr2r <= lsr_mask ? 1'b0 : lsr2r || (lsr2 && ~lsr2_d); // set on rise

// lsr bit 3 (framing error)
reg lsr3_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr3_d <= 1'b0;
	else lsr3_d <= lsr3;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr3r <= 1'b0;
	else lsr3r <= lsr_mask ? 1'b0 : lsr3r || (lsr3 && ~lsr3_d); // set on rise

// lsr bit 4 (break indicator)
reg lsr4_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr4_d <= 1'b0;
	else lsr4_d <= lsr4;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr4r <= 1'b0;
	else lsr4r <= lsr_mask ? 1'b0 : lsr4r || (lsr4 && ~lsr4_d);

// lsr bit 5 (transmitter fifo is empty)
reg lsr5_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr5_d <= 1'b1;
	else lsr5_d <= lsr5;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr5r <= 1'b1;
	else lsr5r <= (fifo_write) ? 1'b0 :  lsr5r || (lsr5 && ~lsr5_d);

// lsr bit 6 (transmitter empty indicator)
reg lsr6_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr6_d <= 1'b1;
	else lsr6_d <= lsr6;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr6r <= 1'b1;
	else lsr6r <= (fifo_write) ? 1'b0 : lsr6r || (lsr6 && ~lsr6_d);

// lsr bit 7 (error in fifo)
reg lsr7_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr7_d <= 1'b0;
	else lsr7_d <= lsr7;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr7r <= 1'b0;
	else lsr7r <= lsr_mask ? 1'b0 : lsr7r || (lsr7 && ~lsr7_d);

// Frequency divider
always @(posedge clk or posedge wb_rst_i) 
begin
	if (wb_rst_i)
		dlc <= 16'h0000;
	else
		if (start_dlc | ~ (|dlc))
  			dlc <= dl - 16'h0001;               // preset counter
		else
			dlc <= dlc - 16'h0001;              // decrement counter
end

// Enable signal generation logic
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		enable <= 1'b0;
	else
		if (|dl & ~(|dlc))     // dl>0 & dlc==0
			enable <= 1'b1;
		else
			enable <= 1'b0;
end

// Delaying THRE status for one character cycle after a character is written to an empty fifo.
always @(lcr or wb_rst_i)
if (wb_rst_i)
	block_value = 127;
else
  case (lcr[3:0])
//    4'b0000                             : block_value =  95; // 6 bits
//    4'b0100                             : block_value = 103; // 6.5 bits
//    4'b0001, 4'b1000                    : block_value = 111; // 7 bits
//    4'b1100                             : block_value = 119; // 7.5 bits
    3'b000                              : block_value = 127; // 8 bits
    3'b001, 3'b010, 3'b100              : block_value = 143; // 9 bits
    3'b011, 3'b101, 3'b110              : block_value = 159; // 10 bits
    4'b111                              : block_value = 175; // 11 bits
    default                             : block_value = 127; // 11 bits
  endcase // case(lcr[3:0])

// Counting time of one character minus stop bit
always @(posedge clk or posedge wb_rst_i)
begin
  if (wb_rst_i)
    block_cnt <= 8'd0000;
  else
  if(lsr5r & fifo_write)  // THRE bit set & write to fifo occured
    block_cnt <= block_value;
  else
  if (enable & block_cnt != 8'b00000000)  // only work on enable times
    block_cnt <= block_cnt - 8'h01;  // decrement break counter
  else
    block_cnt <= block_cnt;
end // always of break condition detection

// Generating THRE status enable signal
assign thre_set_en = ~(|block_cnt);


//
//	INTERRUPT LOGIC
//

assign rls_int  = ier[`UART_IE_RLS] && (lsr[`UART_LS_OE] || lsr[`UART_LS_PE] || lsr[`UART_LS_FE] || lsr[`UART_LS_BI]);
assign rda_int  = ier[`UART_IE_RDA] && (rf_count >= {1'b0,trigger_level});
assign thre_int = ier[`UART_IE_THRE] && lsr[`UART_LS_TFE];
//assign ms_int   = ier[`UART_IE_MS] && (| msr[3:0]);
assign ti_int   = ier[`UART_IE_RDA] && (counter_t == 10'b0000000000) && (|rf_count);

reg 	 rls_int_d;
reg 	 thre_int_d;
//reg 	 ms_int_d;
reg 	 ti_int_d;
reg 	 rda_int_d;

// delay lines
always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rls_int_d <=  1'b0;
	else rls_int_d <= rls_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rda_int_d <=  1'b0;
	else rda_int_d <=  rda_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) thre_int_d <=  1'b0;
	else thre_int_d <=  thre_int;

//always  @(posedge clk or posedge wb_rst_i)
//	if (wb_rst_i) ms_int_d <= #1 0;
//	else ms_int_d <= #1 ms_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ti_int_d <= 1'b0;
	else ti_int_d <= ti_int;

// rise detection signals

wire 	 rls_int_rise;
wire 	 thre_int_rise;
wire 	 ms_int_rise;
wire 	 ti_int_rise;
wire 	 rda_int_rise;

assign rda_int_rise    = rda_int & ~rda_int_d;
assign rls_int_rise 	  = rls_int & ~rls_int_d;
assign thre_int_rise   = thre_int & ~thre_int_d;
//assign ms_int_rise 	  = ms_int & ~ms_int_d;
assign ti_int_rise 	  = ti_int & ~ti_int_d;

// interrupt pending flags
reg 	rls_int_pnd;
reg	rda_int_pnd;
reg 	thre_int_pnd;
//reg 	ms_int_pnd;
reg 	ti_int_pnd;

// interrupt pending flags assignments
always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rls_int_pnd <= 1'b0; 
	else 
		rls_int_pnd <= lsr_mask ? 1'b0 :  						// reset condition
							rls_int_rise ? 1'b1 :						// latch condition
							rls_int_pnd && ier[`UART_IE_RLS];	// default operation: remove if masked

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rda_int_pnd <= 1'b0; 
	else 
		rda_int_pnd <= ((rf_count == {1'b0,trigger_level}) && fifo_read) ? 1'b0 :  	// reset condition
							rda_int_rise ? 1'b1 :						// latch condition
							rda_int_pnd && ier[`UART_IE_RDA];	// default operation: remove if masked

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) thre_int_pnd <= 1'b0; 
	else 
		thre_int_pnd <= fifo_write || (iir_read & ~iir[`UART_II_IP] & iir[`UART_II_II] == `UART_II_THRE)? 1'b0 : 
							thre_int_rise ? 1'b1 :
							thre_int_pnd && ier[`UART_IE_THRE];

//always  @(posedge clk or posedge wb_rst_i)
//	if (wb_rst_i) ms_int_pnd <= #1 0; 
//	else 
//		ms_int_pnd <= #1 msr_read ? 0 : 
//							ms_int_rise ? 1 :
//							ms_int_pnd && ier[`UART_IE_MS];

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ti_int_pnd <= 1'b0; 
	else 
		ti_int_pnd <= fifo_read ? 1'b0 : 
							ti_int_rise ? 1'b1 :
							ti_int_pnd && ier[`UART_IE_RDA];
// end of pending flags

//transmitted and received fifo interrupts
assign transmitted = (~iir[3])&(~iir[2])&(iir[1]);
assign received = (~iir[3])&(iir[2])&(~iir[1]);


// INT_O logic
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)	
		int_o <= 1'b0;
	else
		int_o <= 
					rls_int_pnd		?	~lsr_mask					:
					rda_int_pnd		? 1'b1								:
					ti_int_pnd		? ~fifo_read					:
					thre_int_pnd	? !(fifo_write & iir_read) :
//					ms_int_pnd		? ~msr_read						:
					1'b0;	// if no interrupt are pending
end


// Interrupt Identification register
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		iir <= 4'h01;
	else
	if (rls_int_pnd)  // interrupt is pending
	begin
		iir[`UART_II_II] <= `UART_II_RLS;	// set identification register to correct value
		iir[`UART_II_IP] <= 1'b0;		// and clear the IIR bit 0 (interrupt pending)
	end else // the sequence of conditions determines priority of interrupt identification
	if (rda_int_pnd)
	begin
		iir[`UART_II_II] <=  `UART_II_RDA;
		iir[`UART_II_IP] <=  1'b0;
	end
	else if (ti_int_pnd)
	begin
		iir[`UART_II_II] <=  `UART_II_TI;
		iir[`UART_II_IP] <=  1'b0;
	end
	else if (thre_int_pnd)
	begin
		iir[`UART_II_II] <=  `UART_II_THRE;
		iir[`UART_II_IP] <=  1'b0;
	end
//	else if (ms_int_pnd)
//	begin
//		iir[`UART_II_II] <= `UART_II_MS;
//		iir[`UART_II_IP] <= 1'b0;
//	end 
	else	// no interrupt is pending
	begin
		iir[`UART_II_II] <= 1'b0;
		iir[`UART_II_IP] <= 1'b1;
	end
end

endmodule




