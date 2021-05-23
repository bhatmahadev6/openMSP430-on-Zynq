`timescale 1ns / 1ps
// PWM module configured as a peripheral. 

module  pwm (

// OUTPUTs
    per_dout,                       // Peripheral data output
	pwm_out, 							// Serial transmit line

// INPUTs
    mclk,                           // Main system clock
	smclk,			    // this is used for generating the pwm output freq, this is divided by 100 to generate output, 
    per_addr,                       // Peripheral address
    per_din,                        // Peripheral data input
    per_en,                         // Peripheral enable (high active)
    per_we,                         // Peripheral write enable (high active)
    puc_rst ,                        // Main system reset
    
);

// OUTPUTs
//=========
output       [15:0] per_dout;       // Peripheral data output
output 				pwm_out;				// serial transmit line

// INPUTs
//=========
input               mclk;           // Main system clock
input 				smclk;
input        [13:0] per_addr;       // Peripheral address
input        [15:0] per_din;        // Peripheral data input
input               per_en;         // Peripheral enable (high active)
input        [1:0]  per_we;         // Peripheral write enable (high active)
input               puc_rst;        // Main system reset

reg pwm_out;
//=============================================================================
// 1)  PARAMETER DECLARATION
//=============================================================================

// Register base address (must be aligned to decoder bit width)
parameter       [14:0] BASE_ADDR   = 15'h0100; // to be addressed as 0x0080

// Decoder bit width (defines how many bits are considered for address decoding)
parameter              DEC_WD      =  2;

// Register addresses offset
parameter [DEC_WD-1:0] CNTRL1      = 'h0,
					   CNTRL2      = 'h2;

// Register one-hot decoder utilities
parameter              DEC_SZ      =  (1 << DEC_WD);
parameter [DEC_SZ-1:0] BASE_REG    =  {{DEC_SZ-1{1'b0}}, 1'b1};

// Register one-hot decoder
parameter [DEC_SZ-1:0] CNTRL1_D    = (BASE_REG << CNTRL1),
						CNTRL2_D    = (BASE_REG << CNTRL2);


//============================================================================
// 2)  REGISTER DECODER
//============================================================================

// Local register selection
wire              reg_sel   =  per_en & (per_addr[13:DEC_WD-1]==BASE_ADDR[14:DEC_WD]);

// Register local address
wire [DEC_WD-1:0] reg_addr  =  {per_addr[DEC_WD-2:0], 1'b0};

// Register address decode
wire [DEC_SZ-1:0] reg_dec   =  (CNTRL1_D  &  {DEC_SZ{(reg_addr == CNTRL1 )}}) | 
							   (CNTRL2_D  &  {DEC_SZ{(reg_addr == CNTRL2 )}});

// Read/Write probes
wire              reg_write =  |per_we & reg_sel;
wire              reg_read  = ~|per_we & reg_sel;

// Read/Write vectors
wire [DEC_SZ-1:0] reg_wr    = reg_dec & {DEC_SZ{reg_write}};
wire [DEC_SZ-1:0] reg_rd    = reg_dec & {DEC_SZ{reg_read}};

//============================================================================
// 3) REGISTERS
//============================================================================
// There are two registers for this module which can be written into by the software

// CNTRL1 Register
//-----------------   
reg  [15:0] cntrl1;

wire        cntrl1_wr = reg_wr[CNTRL1];

always @ (posedge mclk or posedge puc_rst)
  if (puc_rst)        cntrl1 <=  16'h0000;
  else if (cntrl1_wr) cntrl1 <=  per_din;

 // CNTRL2 Register
//-----------------   
reg  [15:0] cntrl2;

wire        cntrl2_wr = reg_wr[CNTRL2];

always @ (posedge mclk or posedge puc_rst)
  if (puc_rst)        cntrl2 <=  16'h0000;
  else if (cntrl2_wr) cntrl2 <=  per_din;

// Data output mux

wire [15:0] cntrl1_rd  = cntrl1  & {16{reg_rd[CNTRL1]}};
wire [15:0] cntrl2_rd  = cntrl2  & {16{reg_rd[CNTRL2]}};

wire [15:0] per_dout =  cntrl1_rd | cntrl2_rd;
  
// write a 16 bit data to the register to control the pulse width. the frequency of output can be taken from the smclk/aclk. 
// Depending on how you instantiate this module
// The input clock frequency will be divided by max_value and pwm_count of it will be high (duty cycle control)
// for eg: if max_value is 16'h0064 and pwm_count is 16'h0030, the input clock will be divided by 100(decimal equ of 16'h0064) 
// and 48 (decimal equ of 16'h0032) remains high. so 48% duty cycle.

wire [15:0] pwm_count = cntrl1;     //control value that defines pulse width
reg [15:0] counter; // = 16'h0000;
wire [15:0] max_value = cntrl2; // = 16'h0064;
 
always@ (posedge smclk or posedge puc_rst)
begin	
    if (puc_rst)
		begin
			counter = 16'h0000;
			pwm_out = 1'b0;
		end
	else 
		if (counter < pwm_count)
			begin
				counter = counter + 16'h0001;
				pwm_out = 1'b1;
			end
		else 
			if (counter < max_value)
				begin
					counter = counter + 16'h0001;
					pwm_out = 1'b0;
				end
				
			else 
				counter = 16'h0000;
end				

endmodule
