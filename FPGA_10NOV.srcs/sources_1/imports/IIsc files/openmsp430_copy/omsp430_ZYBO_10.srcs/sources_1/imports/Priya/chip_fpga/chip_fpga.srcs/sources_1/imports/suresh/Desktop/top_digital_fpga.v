// top module which contains openmsp and memory along with peripherals
// ADC, energy measurement units are outside. 
// openmsp,memory,pwm,adcinterface,secded,memchk,uart

module top_verilog(
        memory_check_done,
	    dbg_uart_rxd_in,
	    dbg_uart_txd,                  // chip pins
	    pwm_out,                       // chip pins        
        // SPI Flash pins
        // sclk_flash, 
        //din_flash,
        //dout_flash,
        //cs_flash,
        dbg_en_p,    
        flash_program_mode,
	    normal_mode,
	    start_load,
	    program_loaded,
       	clk_100M,
        poweron_reset,
        PLLOUT,
	  	p1_din,p1_dout,p1_dout_en, 
	    memory_error, 
//	    ta_out0,
	//    ta_out0_en,
	  //  ta_cci0a,ta_cci0b,
        //UART TX/RX
        Dtx,
	    Drx,
	    //temperature sensor related
        //temp_count_i[3:0],
        //switch_o[3:0],
        //counter_rst_o,
	    // Bus interface for XADC 
        xadc_per_addr,                 // controller address
        xadc_per_din,                  // controller data input, from processor
        xadc_per_en,                   // controller enable (high active)
        xadc_per_we,	               // Peripheral write enable (high active)
        xadc_per_dout,                 // controller data output, to processor
        puc_rst_o,                     // Main system reset        
        //led peripheral
        led_per_wen,                   //core enable
        led_per_addr,
        led_per_din,
        led_per_dout,
        led_per_cen
        
	);


//output sclk_flash,din_flash,cs_flash;
//output EnableTemp,Clktemp,Countcleartemp;
//input dout_flash;
//input  [11:0]Countouttemp;


//timer signals
wire ta_out1,ta_out2;
wire ta_out1_en,ta_out2_en;
reg ta_cci1a,ta_cci1b,ta_cci2a,ta_cci2b;
wire [13:0] irq_acc;         //Interrupt accepted


output wire clk_100M;
wire dco_clk,lfxt_clk;
output start_load;
input poweron_reset;
output wire puc_rst_o;
wire puc_rst;
assign puc_rst_o = puc_rst;
 
output Dtx;
input wire Drx;
input PLLOUT;
	
wire Rd_ready;

// added for monitoring
input program_loaded,normal_mode,flash_program_mode;

wire start_load,program_loaded;
output memory_check_done;
wire reset_n;
wire cpu_en,dbg_en,dbg_en_b;
wire [15:0] dma_din;
wire [15:0] dma_dout;
//wire [14:0] flash_addr;
//wire flash_cen,flash_oen,flash_wen;

input [1:0] p1_din;
output [1:0] p1_dout,p1_dout_en;
//input ta_cci0a,ta_cci0b;
//output ta_out0;
//output ta_out0_en;
input dbg_uart_rxd_in;
output dbg_uart_txd;
output pwm_out;
output	memory_error;
input dbg_en_p;

wire        [13:0] pmem_addr;
wire               pmem_cen;
wire        [15:0] pmem_din;
wire        [ 1:0] pmem_wen;
wire        [15:0] pmem_dout;

wire 	    [10:0] dmem_addr;
wire               dmem_cen;
wire        [15:0] dmem_din;
wire        [ 1:0] dmem_wen;
wire        [15:0] dmem_dout;

wire        [13:0] per_addr; //Nivedita changed wire to output
wire               per_en;
wire        [15:0] per_din;
wire         [1:0] per_we;
wire        [15:0] per_dout;


//Temperature sensor
//input [11:0] temp_count_i;
//assign temp_count_i[11:4]=8'b11000101;

//output [ 3:0] switch_o ;
//output wire counter_rst_o;

// peripheral bus I/O for XADC
output wire  [13:0] xadc_per_addr;
output wire         xadc_per_en;
output wire  [15:0] xadc_per_din;
output wire   [1:0] xadc_per_we;
input  wire  [15:0] xadc_per_dout;

assign xadc_per_addr = per_addr;
assign xadc_per_en   = per_en;
assign xadc_per_din  = per_din;
assign xadc_per_we   = per_we; 

//Peripheral for led
output wire [1:0] led_per_wen;
output wire led_per_cen;
output wire [13:0] led_per_addr;
output wire [15:0] led_per_din;
input wire [15:0] led_per_dout;

assign led_per_addr = per_addr;
assign led_per_wen = per_we;
assign led_per_din = per_din;
assign led_per_cen = per_en;
assign led_per_dout = 15'b0;


wire mem_err_dmem_double,mem_err_dmem_single;
wire mem_err_pmem_double,mem_err_pmem_single;

wire mclk;
wire smclk;
wire pwm_out;
wire [15:0] per_dout_adc,per_dout_secded_p,per_dout_secded_d,per_dout_uart,per_dout_pwm,per_dout_gpio,per_dout_spicntrl,per_dout_tempsense,per_dout_timer;
wire [7:0] dmem_addr_in;
wire [21:0] d_mem_din_in,p_mem_din_in;
wire [11:0] pmem_addr_in;
wire [1:0] pmem_wen_in,dmem_wen_in;
wire uart_received,uart_transmitted;
wire timer_interrupt_ta0,timer_interrupt_ta1;
wire irq_ta0_acc;


`define IRQ_NR 16    
wire [`IRQ_NR-3:0] irq_in;
wire adc_clock;
reg [2:0] count_adc;
reg adc_interrupt;

assign dbg_en = dbg_en_p | dbg_en_b;

//gpio interrupt
wire gpio_interrupt;
// generating the adc interrupt signal, as a pulse 7 clock cycle wide
always @(posedge clk_100M or posedge puc_rst)//always @(mclk or adc_clock or puc_rst)
	if (puc_rst)
		begin
			adc_interrupt = 1'b0;
			count_adc = 3'b111;
		end
	else
		if (adc_clock)
			begin
			if (|count_adc)
				begin
					adc_interrupt = 1'b1;
					count_adc = count_adc - 3'b001;
				end
			else
				adc_interrupt = 1'b0;
			end
		else
			count_adc = 3'b111;

assign irq_in  =        {mem_err_pmem_double,                 // Vector 13  (0xFFFA) mclk is to be stopped if we get this interrupt
                         mem_err_dmem_double,                 // Vector 12  (0xFFF8) mclk is to be stopped if we get this interrupt
                         mem_err_pmem_single,                 // Vector 11  (0xFFF6) memory correction to be carried out 
                         1'b0,                                // Vector 10  (0xFFF4) - Watchdog -
                         mem_err_dmem_single,                 // Vector  9  (0xFFF2) memory correction to be carried out
                         1'b0,                       // Vector  8  (0xFFEE)  can be used to read the data from receive buffer
                         adc_interrupt,                                // adc_interrupt,              // Vector  7  (0xFFF0) this ISR will carry out acquisition
                         uart_transmitted,                    // Vector  6  (0xFFEC) can be used to ensure data is transmitted
                         1'b0,                 // Vector  5  (0xFFEA)
                         1'b0,                 // Vector  4  (0xFFE8)
                         gpio_interrupt,                                // Vector  3  (0xFFE6)
                         1'b0,                                // Vector  2  (0xFFE4)
                         1'b0,                                // Vector  1  (0xFFE2)
                         1'b0};                               // Vector  0  (0xFFE0)

wire aclk;

wire         [15:1] dma_addr;                 // Direct Memory Access address
//wire         [15:0] dma_din;                // Direct Memory Access data wire
wire                dma_en;                   // Direct Memory Access enable (high active)
wire                dma_priority;             // Direct Memory Access priority (0:low / 1:high)
wire          [1:0] dma_we;                   // Direct Memory Access write byte enable (high active)
wire                dma_wkup;                 // ASIC ONLY: DMA Wake-up (asynchronous and non-glitchy)
//wire        [15:0] dma_dout;                // Direct Memory Access data output
wire               dma_ready;                 // Direct Memory Access is complete
wire               dma_resp;                  // Direct Memory Access response (0:Okay / 1:Error)
wire dbg_uart_txd;

openMSP430 CORE (

// OUTPUTs
    .aclk(aclk),                                    
    .aclk_en(aclk_en),                                 
    .dbg_freeze(dbg_freeze),                              
    .dbg_i2c_sda_out(),                         
    .dbg_uart_txd(dbg_uart_txd),                            
    .dco_enable(),                              
    .dco_wkup(),                                
    .dmem_addr(dmem_addr),                               
    .dmem_cen(dmem_cen),                                
    .dmem_din(dmem_din),                             
    .dmem_wen(dmem_wen),                              
    .irq_acc(irq_acc),                                
    .lfxt_enable(),                           
    .lfxt_wkup(),                            
    .mclk(mclk),                                
    .dma_dout(dma_dout),                             
    .dma_ready(dma_ready),                           
    .dma_resp(dma_resp),                             
    .per_addr(per_addr),                            
    .per_din(per_din),                             
    .per_en(per_en),                              
    .per_we(per_we),                               
    .pmem_addr(pmem_addr),                            
    .pmem_cen(pmem_cen),                              
    .pmem_din(pmem_din),                                
    .pmem_wen(pmem_wen),                                
    .puc_rst(puc_rst),                             
    .smclk(smclk),                                
    .smclk_en(), 

// INPUTs
    .cpu_en(cpu_en),                                  // cpu enable 1'b1 for cpu to work
    .dbg_en(dbg_en),                                  // Debug interface enable (asynchronous and non-glitchy)
    .dbg_i2c_addr(7'b0000000),                        // Debug interface: I2C Address
    .dbg_i2c_broadcast(7'b0000000),                   // Debug interface: I2C Broadcast Address (for multicore systems)
    .dbg_i2c_scl(1'b0),                               // Debug interface: I2C SCL
    .dbg_i2c_sda_in(1'b0),                            // Debug interface: I2C SDA IN
    .dbg_uart_rxd(dbg_uart_rxd),                      // Debug interface: UART RXD (asynchronous)
    .dco_clk(dco_clk),                                // Fast oscillator (fast clock)
    .dmem_dout(dmem_dout),                            // Data Memory data output
    .irq(irq_in),                                       // Maskable interrupts
    .lfxt_clk(lfxt_clk),                              // Low frequency oscillator (typ 32kHz)
    .dma_addr(dma_addr),                              // Direct Memory Access address
    .dma_din(dma_din),                                // Direct Memory Access data input
    .dma_en(dma_en),                                  // Direct Memory Access enable (high active)
    .dma_priority(dma_priority),                      // Direct Memory Access priority (0:low / 1:high)
    .dma_we(dma_we),                                  // Direct Memory Access write byte enable (high active)
    .dma_wkup(dma_wkup),                              // ASIC ONLY: DMA Sub-System Wake-up (asynchronous and non-glitchy)
    .nmi(1'b0),                                       // Non-maskable interrupt (asynchronous)
    .per_dout(per_dout),                              // Peripheral data output
    .pmem_dout(pmem_dout),                            // Program Memory data output
    .reset_n(reset_n),                                // Reset Pin (low active, asynchronous and non-glitchy)
    .scan_enable(1'b0),                               // ASIC ONLY: Scan enable (active during scan shifting)
    .scan_mode(1'b0),                                 // ASIC ONLY: Scan mode
    .wkup(1'b0)                                       // ASIC ONLY: System Wake-up (asynchronous and non-glitchy)
);

wire [21:0] d_mem_din,p_mem_din;

// to added for fpga
wire [35:0] dmem_mem_din = {14'h0000,d_mem_din};
wire [35:0] pmem_mem_din = {14'h0000,p_mem_din};

// to be added for asic
//wire [35:0] pmem_mem_din = {14'h000,p_mem_din_in};
//wire [35:0] dmem_mem_din = {14'h000,d_mem_din_in};

wire dmem_cen_in,pmem_cen_in;
wire [35:0] dmem_mem_dout,pmem_mem_dout;

// FPGA BRAM Instance for program memory
/*pmem PROGMEM (
  .clka(mclk),    // input wire clka
  .ena(~pmem_cen),      // input wire ena
  .wea(~|pmem_wen),      // input wire [0 : 0] wea
  .addra(pmem_addr),  // input wire [9 : 0] addra
  .dina(pmem_mem_din),    // input wire [35 : 0] dina
  .douta(pmem_mem_dout)  // output wire [35 : 0] douta
);
*/

// FPGA BRAM Instance for data memory
dmem DATAMEM(
  .dmem_clk(mclk),
  .dmem_wen(~|dmem_wen),
   //.ena(~dmem_cen),
  .dmem_addr(dmem_addr),
  .dmem_din(dmem_mem_din),
  .dmem_dout(dmem_mem_dout)
);


//Program Memory instantiation for synthesis
/*
SPRAM_8192x36 PROGMEM(
	.A(pmem_addr),
	.CE(mclk),
	.WEB(|pmem_wen), 
	.OEB(~(|pmem_wen)), 
	.CSB(pmem_cen),
	.I(pmem_mem_din),
	.O(pmem_mem_dout)
);
*/

//Data Memory Instatntiation for synthesis
/*
SPRAM_1024x36 DATAMEM(
	.A(dmem_addr),
	.CE(mclk),
	.WEB(|dmem_wen), // WEB is low, then write, high means read, dmem_wen becomes low during read
	.OEB(~(|dmem_wen)), 
	.CSB(dmem_cen),	// CSB shud be low for chip select, dmem_cen becomes low when chip select
	.I(dmem_mem_din),
	.O(dmem_mem_dout)
);
*/

//Program Memory Instatntiation for simulation
pmem PROGMEM(
  .pmem_clk(mclk),
  .pmem_wen(|pmem_wen),
  //.pmem_cen(pmem_cen),
  .pmem_addr(pmem_addr),
  .pmem_din(pmem_mem_din),
  .pmem_dout(pmem_mem_dout)
);
/*
//Data Memory Instatntiation for simulation
dmem DATAMEM(
  .dmem_clk(mclk),
  .dmem_wen(|dmem_wen),
  .dmem_cen(dmem_cen),
  .dmem_addr(dmem_addr),
  .dmem_din(dmem_mem_din),
  .dmem_dout(dmem_mem_dout)
);
*/

wire clk_48K;
wire clk_25M;




clock_top clk_divider(

	.clk_in(PLLOUT), 
	.rst(poweron_reset), 
	.clk_100M(clk_100M), 
	.clk_50M(dco_clk),
	.clk_25M(clk_25M), 
	.clk_48K(clk_48K), 
	.clk_32K(lfxt_clk), 
	.rst_o(clkdiv_reset)

);



wire [21:0] d_mem_dout = dmem_mem_dout[21:0];     // taking the 22 bits from 36 bits in memory. 
wire [21:0] p_mem_dout = pmem_mem_dout[21:0];

// connecting the peripheral output bus of all peripherals (including XADC)
assign per_dout = 
                    per_dout_uart |
//                  per_dout_adc   |
                    per_dout_secded_p |
		            per_dout_secded_d |
		            per_dout_gpio |
//		            per_dout_spicntrl|
//                  per_dout_tempsense|
		            per_dout_pwm  |
                    xadc_per_dout |
                    per_dout_timer ;
//                  | led_per_dout;  // XADC //changed

uart_top UART (

// OUTPUTs
    .dat_o(per_dout_uart),                       // Peripheral data output
	
// INPUTs
  //  .clk_i(dco_clk),                           // Main system clock
    .clk_i(mclk),                                // Main system clock
    .adr_i(per_addr),                            // Peripheral address
    .dat_i(per_din),                             // Peripheral data input
    .per_en(per_en),                             // Peripheral enable (high active)
    .we_i(per_we),                               // Peripheral write enable (high active)
    .en_proc_ring_o(EnRINGRS485),
    .proc_ring_i(RINGOUTRS485),
    .imp_ctrl_2(Resb2),
    .imp_ctrl_1(Resb1),
    .imp_ctrl_0(Resb0),
    .tx_o(Dtx),
    .rx_i(Drx),
    .rst(puc_rst),                               // Main system reset
    .transmitted(uart_transmitted),
    .received(uart_received)
);

// secded for program memory
secded_program SECDED_p (
	.per_dout(per_dout_secded_p),	             // peripheral output data
	.mclk(mclk),
	.puc_rst(puc_rst),
	
	.pmem_wen(pmem_wen),
	.pmem_cen(pmem_cen),
	.pmem_dout(pmem_dout),
	.pmem_din(pmem_din),
	.pmem_addr(pmem_addr),
	
	.mem_din(p_mem_din),
	
	.per_addr(per_addr),
	.per_din(per_din),
	.per_en(per_en),
	.per_we(per_we),
	
	.mem_dout(p_mem_dout),
	
	.mem_err_double(mem_err_pmem_double),
	.mem_err_single(mem_err_pmem_single)
);

// secded for data memory
secded_data SECDED_d (
	.per_dout(per_dout_secded_d),	            // peripheral output data
	.mclk(mclk),
	.puc_rst(puc_rst),
	
	.pmem_wen(dmem_wen),
	.pmem_cen(dmem_cen),
	.pmem_dout(dmem_dout),
	.pmem_din(dmem_din),
	.pmem_addr(dmem_addr),
	
	.mem_din(d_mem_din),
	
	.per_addr(per_addr),
	.per_din(per_din),
	.per_en(per_en),
	.per_we(per_we),
	
	.mem_dout(d_mem_dout),
	
	.mem_err_double(mem_err_dmem_double),
	.mem_err_single(mem_err_dmem_single)
);


// PWM module
pwm PWM (
    .per_dout(per_dout_pwm),                    // Peripheral data output
	.pwm_out(pwm_out), 							// PWM output signal
    .mclk(mclk),                                // Main system clock
	.smclk(aclk),
    .per_addr(per_addr),                        // Peripheral address
    .per_din(per_din),                          // Peripheral data input
    .per_en(per_en),                            // Peripheral enable (high active)
    .per_we(per_we),                            // Peripheral write enable (high active)
    .puc_rst(puc_rst)                           // Main system reset
);



//Temp Sensor Peripheral module



//wire en_temp_ring_o;//Doubt

//temp_sensor_top   temp_sensor_top(.mclk(mclk),.en_clk(clk_48K),.puc_rst(clkdiv_reset),.per_addr(per_addr),.per_din(per_din),.per_en(per_en),.per_we(per_we),.per_dout	    (per_dout_tempsense),.temp_count_i(temp_count_i),.counter_rst_o(counter_rst_o),.en_temp_ring_o(en_temp_ring_o),.switch_o(switch_o));



wire busy_normal;
wire fifo_en;
wire poweron_reset;                              // this is reset signal to the chip from outside
wire spi_busy,spi_done;
wire spi_wen,spi_oen,spi_cen;
wire [14:0] spi_addr;
wire dbg_rxd_memchk,dbg_rxd_boot;
wire [6:0] data_width;

mux dbg_rxd_memchk_boot(
	.dbg_rxd_memchk(dbg_rxd_memchk),
	.dbg_rxd_boot(dbg_rxd_boot),
	.memory_check_done(memory_check_done), 
	.dbg_uart_rxd(dbg_uart_rxd)
);


// power ON self test module
memorytest MEMTEST	(
	//.mclk(dco_clk),
	.cpu_en(cpu_en),
	.dbg_en(dbg_en),
	.dbg_uart_txd(dbg_uart_txd),
	.dbg_uart_rxd(dbg_rxd_memchk),
	 //.reset(clkdiv_reset),
	.memory_error(memory_error),
	.memory_check_done(memory_check_done),
	.test(test),
	.test_reset(clkdiv_reset),                  //.test_reset(test_reset),
	.mclk_int(clk_25M)
	);


bootchip BOOT(
	.dbg_en(dbg_en_b),
	.cpu_en(cpu_en),
	.mclk(dco_clk),
	.reset_n(reset_n),
	.poweron_reset(clkdiv_reset),			    //chip pin in
	.dma_addr(dma_addr),
	.dma_en(dma_en),
	.dma_priority(dma_priority),
	.dma_we(dma_we),
	.dma_wkup(dma_wkup),
	.dma_ready(dma_ready),
    .dma_resp(dma_resp),
	.memory_check_done(memory_check_done),
    //.flash_addr(spi_addr),
    //.flash_wen(spi_wen),
    //.flash_cen(spi_cen),
    //.flash_oen(spi_oen),
	.flash_program_mode(flash_program_mode),	//chip pin in
	.normal_mode(normal_mode),			        //chip pin in
	.program_loaded(program_loaded),		    //chip pin in
	.start_load(start_load),			        //chip pin out
	.dbg_uart_rxd_in(dbg_uart_rxd_in),
	.dbg_uart_rxd(dbg_rxd_boot),
    //.flash_busy(spi_busy),				    //status of flash
    //.done(spi_done),
    .sync(fifo_en),
    .data_width(data_width),
    .busy_normal(busy_normal),
	.test(test),
	//.test_reset(test_reset),
	.mclk_int(clk_25M)
	);


//SPI control as peripheral module


//interface_top spi_cntrl(

//	 .flash_dout(dma_din),                      // flash data output
//   .flash_busy(spi_busy),                     // status of flash
        
//    // INPUTs from core
//   .flash_addr({spi_addr,1'b0}),              // flash address
//   .flash_cen(spi_cen),                       // flash chip enable (low active)
//   .flash_clk_25(~clk_25M),                   // flash clock - 25M
//	 .flash_clk_50(mclk),				        // flash clock - 50M
//	 .flash_clk(dco_clk),				        // flash clock - 50M
//   .flash_rst(~clkdiv_reset),                 // flash_interface reset
//   .flash_din(dma_dout),                      // flash data input
//   .flash_wen(spi_wen),                       // flash write enable (low active)
//	 .flash_oen(spi_oen),
//   .done(spi_done),                           // tells reading is finished

//    // OUTPUTs to flash
//   .sclk(sclk_flash),                         //flash data output 
//   .cs_n(cs_flash),							//chip pin out
//   .din(din_flash),							//chip pin out
    
//    // INPUTs from flash
//    .dout(dout_flash),   						//chip pin in

//	//Peripheral connection related signals
//    .per_dout(per_dout_spicntrl),	            // peripheral output data
// 	  .per_addr(per_addr),                      // Peripheral address
//    .per_din(per_din),                        // Peripheral data input
//    .per_en(per_en),                          // Peripheral enable (high active)
//    .per_we(per_we),
//    .en(fifo_en) ,
//    .data_width(data_width) ,
//    .busy_normal(busy_normal)                 // Peripheral write enable (high active)
//                                              // Peripheral write enable (high active)
                             
//    );

wire[7:0] p11_dout,p11_dout_en,p11_sel;
wire [7:0] p11_din;
assign p1_dout=p11_dout[1:0];
assign p1_dout_en=p11_dout_en[1:0];
assign p1_sel=p11_sel[1:0];
assign p1_din=p11_din[1:0];


// GPIO module, taken from msp430 folder itself

omsp_gpio GPIO(

// OUTPUTs
    .irq_port1(gpio_interrupt),                      // Port 1 interrupt
    .irq_port2(),                      // Port 2 interrupt
    .p1_dout(p11_dout),                // Port 1 data output
    .p1_dout_en(p11_dout_en),          // Port 1 data output enable
    .p1_sel(p11_sel),                  // Port 1 function select
    .p2_dout(),                        // Port 2 data output
    .p2_dout_en(),                     // Port 2 data output enable
    .p2_sel(),                         // Port 2 function select
    .p3_dout(),                        // Port 3 data output
    .p3_dout_en(),                     // Port 3 data output enable
    .p3_sel(),                         // Port 3 function select
    .p4_dout(),                        // Port 4 data output
    .p4_dout_en(),                     // Port 4 data output enable
    .p4_sel(),                         // Port 4 function select
    .p5_dout(),                        // Port 5 data output
    .p5_dout_en(),                     // Port 5 data output enable
    .p5_sel(),                         // Port 5 function select
    .p6_dout(),                        // Port 6 data output
    .p6_dout_en(),                     // Port 6 data output enable
    .p6_sel(),                         // Port 6 function select
    .per_dout(per_dout_gpio),          // Peripheral data output

// INPUTs
    .mclk(mclk),                       // Main system clock
    .p1_din(p11_din),                  // Port 1 data input
    .p2_din(8'h00),                    // Port 2 data input
    .p3_din(8'h00),                    // Port 3 data input
    .p4_din(8'h00),                    // Port 4 data input
    .p5_din(8'h00),                    // Port 5 data input
    .p6_din(8'h00),                    // Port 6 data input
    .per_addr(per_addr),               // Peripheral address
    .per_din(per_din),                 // Peripheral data input
    .per_en(per_en),                   // Peripheral enable (high active)
    .per_we(per_we),                   // Peripheral write enable (high active)
    .puc_rst(puc_rst)                  // Main system reset
);
		


/*assign 
begin
p1_dout_top=p1_dout[1:0];
p1_sel_top=p1_sel[1:0];
p1_dout_en_top=p1_dout_en[1:0];
p1_din_top=p1_din[1:0];
end*/
//wire ta_out0,ta_out0_en,ta_out1,ta_out1_en,ta_out2,ta_out2_en;
//wire ta_cci0a,ta_cci0b,ta_cci1a,ta_cci1b,ta_cci2a,ta_cci2b;


//omsp_timerA TIMERA(

//// OUTPUTs
//    .irq_ta0(timer_interrupt_ta0),     // Timer A interrupt: TACCR0
//    .irq_ta1(timer_interrupt_ta1),     // Timer A interrupt: TAIV, TACCR1, TACCR2
//    .per_dout(per_dout_timer),         // Peripheral data output
//    .ta_out0(ta_out0),                 // Timer A output 0
//    .ta_out0_en(ta_out0_en),           // Timer A output 0 enable
//    .ta_out1(ta_out1),                 // Timer A output 1
//    .ta_out1_en(ta_out1_en),           // Timer A output 1 enable
//    .ta_out2(ta_out2),                 // Timer A output 2
//    .ta_out2_en(ta_out2_en),           // Timer A output 2 enable

//// INPUTs
//    .aclk_en(aclk_en),                        // ACLK enable (from CPU)
//    .dbg_freeze(dbg_freeze),                     // Freeze Timer A counter
//    .inclk(),                          // INCLK external timer clock (SLOW)
//    .irq_ta0_acc(irq_acc[5]),         // Interrupt request TACCR0 accepted
//    .mclk(mclk),                       // Main system clock
//    .per_addr(per_addr),               // Peripheral address
//    .per_din(per_din),                 // Peripheral data input
//    .per_en(per_en),                   // Peripheral enable (high active)
//    .per_we(per_we),                   // Peripheral write enable (high active)
//    .puc_rst(puc_rst),                 // Main system reset
//    .smclk_en(),                       // SMCLK enable (from CPU)
//    .ta_cci0a(ta_cci0a),               // Timer A capture 0 input A
//    .ta_cci0b(ta_cci0b),               // Timer A capture 0 input B
//    .ta_cci1a(ta_cci1a),               // Timer A capture 1 input A
//    .ta_cci1b(ta_cci1b),               // Timer A capture 1 input B
//    .ta_cci2a(ta_cci2a),               // Timer A capture 2 input A
//    .ta_cci2b(ta_cci2b),               // Timer A capture 2 input B
//    .taclk()                           // TACLK external timer clock (SLOW)
    
//);



endmodule
