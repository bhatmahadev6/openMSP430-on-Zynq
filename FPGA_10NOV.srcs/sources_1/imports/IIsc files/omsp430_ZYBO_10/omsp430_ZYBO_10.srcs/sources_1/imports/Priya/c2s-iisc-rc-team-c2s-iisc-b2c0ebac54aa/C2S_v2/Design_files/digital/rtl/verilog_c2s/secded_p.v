`timescale 1ns / 1ps
// for DATA MEMORY

// this module takes output of pmem memory (pmem_dout) and then checks for error and gives the correct data bits to core
// it raises an error signal when parity doesnt match

///		 				---------------------------------
/// 					|								|
///		pmem_din -->	|								| --> mem_err_double
///		pmem_cen -->	|								| --> mem_err_single
///		mem_dout --> 	|								| --> mem_din
///		pmem_wen --> 	|			SECDED				| --> pmem_dout
///		mclk	 -->	|								|
///		puc_rst	 --> 	|								|
///						|								|
///						|								|
///		 				---------------------------------

module secded_program(
	per_dout,	// peripheral output data
	mclk,
	puc_rst,
	
	pmem_wen,
	pmem_cen,
	pmem_dout,
	pmem_din,
	pmem_addr,
	
	mem_din,
	
	per_addr,
	per_din,
	per_en,
	per_we,
	
	mem_dout,
	
	mem_err_double,
	mem_err_single
);

input       [15:0]  pmem_din; 
input               pmem_cen; // to be generated from the core as chip enable
input 		[1:0]   pmem_wen; // to be generated from the core as write enable
input				mclk,puc_rst;   
input 		[12:0] 	pmem_addr;	// the address for which the read command from memory was generated
output 		[15:0]  pmem_dout; 
output 		[21:0] mem_din; 
output 			   mem_err_double,mem_err_single; // memory error signal 
input 		[21:0] mem_dout; 
output       [15:0] per_dout;       // Peripheral data output
input        [13:0] per_addr;       // Peripheral address
input        [15:0] per_din;        // Peripheral data input
input               per_en;         // Peripheral enable (high active)
input         [1:0] per_we;         // Peripheral write enable (high active)

reg [15:0] pmem_dout;
reg [21:0] mem_din;
wire [21:0] gen_din;
wire [15:0] correctdataout;
	reg [12:0] 	pmem_addr1;


// generate parity from pmem_din and make mem_din	

// parity computed from data bits
wire DD0 = pmem_din[0];
wire DD1 = pmem_din[1];
wire DD2 = pmem_din[2];
wire DD3 = pmem_din[3];
wire DD4 = pmem_din[4];
wire DD5 = pmem_din[5];
wire DD6 = pmem_din[6];
wire DD7 = pmem_din[7];
wire DD8 = pmem_din[8];
wire DD9 = pmem_din[9];
wire DD10 = pmem_din[10];
wire DD11 = pmem_din[11];
wire DD12 = pmem_din[12];
wire DD13 = pmem_din[13];
wire DD14 = pmem_din[14];
wire DD15 = pmem_din[15];

wire PP0C = (DD15 ^ DD13) ^ (DD11 ^ DD10) ^ (DD8 ^ DD6) ^ (DD4 ^ DD3) ^ (DD1 ^ DD0);
wire PP1C = (DD13 ^ DD12) ^ (DD10 ^ DD9) ^ (DD6 ^ DD5) ^ (DD3 ^ DD2) ^ DD0;
wire PP2C = DD15 ^ (DD14 ^ DD10) ^ (DD9 ^ DD8) ^ (DD7 ^ DD3) ^ (DD2 ^ DD1);
wire PP3C = (DD10 ^ DD9) ^ (DD8 ^ DD7) ^ (DD6 ^ DD5) ^ DD4;
wire PP4C = (DD15 ^ DD14) ^ (DD13 ^ DD12) ^ DD11;
wire PP5C = (DD0 ^ DD1) ^ (DD2 ^ DD3) ^ (DD4 ^ DD5) ^ (DD6 ^ DD7) ^ (DD8 ^ DD9) ^ (DD10 ^ DD11) ^ (DD12 ^ DD13) ^ (DD14 ^ DD15) ^ (PP0C ^ PP1C) ^ (PP2C ^ PP3C) ^ PP4C;		
assign gen_din = {PP5C,DD15,DD14,DD13,DD12,DD11,PP4C,DD10,DD9,DD8,DD7,DD6,DD5,DD4,PP3C,DD3,DD2,DD1,PP2C,DD0,PP1C,PP0C};
	
// using mem_dout to generate parity and checking for single and double error
wire P0 = mem_dout[0];
wire P1 = mem_dout[1];
wire D0 = mem_dout[2];
wire P2 = mem_dout[3];
wire D1 = mem_dout[4];
wire D2 = mem_dout[5];
wire D3 = mem_dout[6];
wire P3 = mem_dout[7];
wire D4 = mem_dout[8];
wire D5 = mem_dout[9];
wire D6 = mem_dout[10];
wire D7 = mem_dout[11];
wire D8 = mem_dout[12];
wire D9 = mem_dout[13];
wire D10 = mem_dout[14];
wire P4 = mem_dout[15];
wire D11 = mem_dout[16];
wire D12 = mem_dout[17];
wire D13 = mem_dout[18];
wire D14 = mem_dout[19];
wire D15 = mem_dout[20];
wire P5 = mem_dout[21];

// parity computed from data bits
wire P0C = (D15 ^ D13) ^ (D11 ^ D10) ^ (D8 ^ D6) ^ (D4 ^ D3) ^ (D1 ^ D0);
wire P1C = (D13 ^ D12) ^ (D10 ^ D9) ^ (D6 ^ D5) ^ (D3 ^ D2) ^ D0;
wire P2C = D15 ^ (D14 ^ D10) ^ (D9 ^ D8) ^ (D7 ^ D3) ^ (D2 ^ D1);
wire P3C = (D10 ^ D9) ^ (D8 ^ D7) ^ (D6 ^ D5) ^ D4;
wire P4C = (D15 ^ D14) ^ (D13 ^ D12) ^ D11;
wire P5C = (D0 ^ D1) ^ (D2 ^ D3) ^ (D4 ^ D5) ^ (D6 ^ D7) ^ (D8 ^ D9) ^ (D10 ^ D11) ^ (D12 ^ D13) ^ (D14 ^ D15) ^ (P0C ^ P1C) ^ (P2C ^ P3C) ^ P4C;

// checking both parity bits
wire errP0 = P0C ^ P0;
wire errP1 = P1C ^ P1;
wire errP2 = P2C ^ P2;
wire errP3 = P3C ^ P3;
wire errP4 = P4C ^ P4;
wire errP5 = P5C ^ P5;

// xor of all bits becomes 1 for a single error
wire P01xor = errP0 ^ errP1;
wire P23xor = errP2 ^ errP3;
wire P45xor = errP4 ^ errP5;

wire P12xor = errP1 ^ errP2;
wire P34xor = errP3 ^ errP4;
wire P50xor = errP5 ^ errP0;

wire P2xor = P01xor ^ errP2;
wire P3xor = P2xor ^ errP3;
wire P4xor = P3xor ^ errP4;

wire singleerr = P4xor ^ errP5;
wire ORnet = P01xor | P23xor | P45xor | P12xor | P34xor | P50xor;
wire doubleerr = (ORnet & !(singleerr));

reg mem_err_double,mem_err_single;
reg [7:0] count_double,count_single;

always @(posedge mclk or posedge puc_rst)
	begin
	if (puc_rst)
		count_double = 8'h00;
	else if (doubleerr)
		count_double = count_double + 8'h01;
	else 
		count_double = count_double;
	end

always @(posedge mclk or posedge puc_rst)
	if (puc_rst)
		pmem_addr1= 12'h000;
	else 
		pmem_addr1=pmem_addr;


always @(posedge mclk or posedge puc_rst)
	begin
	if (puc_rst)
		count_single = 8'h00;
	else if (singleerr)
		count_single = count_single + 8'h01;
	else 
		count_single = count_single;
	end
	
always @(posedge mclk or posedge puc_rst) //uncommented the always block to make sure secded interrupt is triggered
	if (puc_rst) 
		begin	
			mem_err_double = 1'b0;
			mem_err_single = 1'b0;
		end
	else 
		begin
			mem_err_double = doubleerr;
			mem_err_single = singleerr;
		end
			
// need to correct for single errors. in case of single errors, P5 will change and xor of P4-P0 gives the location


always @(puc_rst or correctdataout or gen_din or mem_err_double or doubleerr)
	if (puc_rst)
		begin
			pmem_dout = 16'b0000000000000000;
			mem_din = 22'b0000000000000000000000;
		end
	else 
		if (doubleerr)
			begin
				pmem_dout = 16'b0000000000000000;
				mem_din = gen_din;
            end
		else
			begin
				pmem_dout = correctdataout;
				mem_din = gen_din;
            end
wire [4:0] oldparity = {P4,P3,P2,P1,P0};
wire [4:0] newparity = {P4C,P3C,P2C,P1C,P0C};
wire [4:0] error_location = oldparity ^ newparity;
reg [21:0] correction;

always @(mclk or error_location or puc_rst)
begin
if (puc_rst)
	correction = 22'b0000000000000000000000;
else
case (error_location)
	5'b00001: correction = 22'b0000000000000000000001;
	5'b00010: correction = 22'b0000000000000000000010;
    5'b00011: correction = 22'b0000000000000000000100;
    5'b00100: correction = 22'b0000000000000000001000;
    5'b00101: correction = 22'b0000000000000000010000;
    5'b00110: correction = 22'b0000000000000000100000;
    5'b00111: correction = 22'b0000000000000001000000;
	5'b01000: correction = 22'b0000000000000010000000;
	5'b01001: correction = 22'b0000000000000100000000;
	5'b01010: correction = 22'b0000000000001000000000;
	5'b01011: correction = 22'b0000000000010000000000;
	5'b01100: correction = 22'b0000000000100000000000;
	5'b01101: correction = 22'b0000000001000000000000;
	5'b01110: correction = 22'b0000000010000000000000;
	5'b01111: correction = 22'b0000000100000000000000;
	5'b10000: correction = 22'b0000001000000000000000;
	5'b10001: correction = 22'b0000010000000000000000;
	5'b10010: correction = 22'b0000100000000000000000;
	5'b10011: correction = 22'b0001000000000000000000;
	5'b10100: correction = 22'b0010000000000000000000;
	5'b10101: correction = 22'b0100000000000000000000;
	5'b10110: correction = 22'b1000000000000000000000;
	default   correction = 22'b0000000000000000000000;
endcase
end

wire [21:0] correctdata = mem_dout ^ correction;
assign correctdataout = {correctdata[20:16],correctdata[14:8],correctdata[6:4],correctdata[2]};

// peripheral related code

parameter       [14:0] BASE_ADDR   = 15'h0094; 
// 0000 0001 1000 0000
// address to be communicated is 0000 0011 0000 0000
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

reg  [15:0] cntrl1;
wire        cntrl1_wr = reg_rd[CNTRL1];
reg [15:0] data1;
wire [1:0] errorsign = {doubleerr,singleerr};

/*always @(mem_err_single or mem_err_double or pmem_addr or errorsign or puc_rst)
        if (puc_rst) 
           data1=16'h0000;
        else
         begin
	  if (|errorsign)
		data1 = {6'h00,pmem_addr};
	  else 
		data1 = 16'h0000;
         end

*/
always @(posedge mclk or posedge puc_rst)
        if (puc_rst) 
           data1=16'h0000;
        else
         begin
	  if (|errorsign)
		data1 = {6'h00,pmem_addr1};
	  else 
		data1 = data1;
	 end

  always@(*) 
	begin
	if (puc_rst) cntrl1=16'h0000;
	else cntrl1 =  data1;
	end
			
reg  [15:0] cntrl2;
wire        cntrl2_wr = reg_rd[CNTRL2];
reg [15:0] data2;

always @(count_single or count_double or puc_rst)
      if(puc_rst) 
         data2=16'h0000;
      else
	data2 = {count_double,count_single};
      


   always@(*)	
	begin
	if (puc_rst) cntrl2=16'h0000;
	else cntrl2 =  data2;
	end
			
wire [15:0] cntrl1_rd  = cntrl1  & {16{reg_rd[CNTRL1]}};
wire [15:0] cntrl2_rd  = cntrl2  & {16{reg_rd[CNTRL2]}};
	
wire [15:0] per_dout   =  cntrl1_rd | cntrl2_rd;

endmodule
