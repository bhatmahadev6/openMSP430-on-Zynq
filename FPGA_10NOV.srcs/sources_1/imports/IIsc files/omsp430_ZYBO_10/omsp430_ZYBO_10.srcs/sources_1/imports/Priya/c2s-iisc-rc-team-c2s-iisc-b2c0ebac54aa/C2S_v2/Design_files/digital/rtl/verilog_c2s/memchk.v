`timescale 1ns / 1ps
// START UP MEMORY CHECK MODULE

module memorytest (cpu_en,dbg_en,dbg_uart_txd,dbg_uart_rxd,memory_error,memory_check_done,test,mclk_int,test_reset);
// after first reset, will start loading program and data memory. once its checked, will raise memory done signal
// in case of error, will raise memory error signal
// the program is based on a state machine and program goes through each state
input cpu_en,dbg_en;
//input mclk;
//input reset; //reset from clock_divder
input dbg_uart_txd;//,dbg_uart_rxd_in;
output dbg_uart_rxd;
output memory_error;
output memory_check_done;
input test,test_reset;
input mclk_int;

//wire dbg_uart_rxd_in;
reg dbg_uart_rxd;
reg memory_check_done;
reg [5:0] mem_state;
reg memory_error;


// declaration of states
parameter init = 0;
parameter dbg_setup = 1;
parameter zero_fill_1 = 2;
parameter zero_fill_2 = 3;
parameter zero_fill_3 = 4;
parameter zero_fill_4 = 5;
parameter zero_fill_5 = 6;
parameter zero_fill_6 = 7;
parameter zero_fill_7 = 8;
parameter zero_fill_8 = 9;
parameter zero_fill_9 = 10;
parameter zero_fill_10 = 11;
parameter zero_fill_11 = 12;
parameter zero_fill_12 = 13;
parameter zero_fill_13 = 14;
parameter zero_fill_14 = 15;
parameter zero_fill_15 = 16;
parameter zero_fill_16 = 17;
parameter zero_fill_17 = 18;
parameter zero_fill_18 = 19;
parameter zero_fill_19 = 20;
parameter zero_fill_20 = 21;
parameter zero_fill_21 = 22;
parameter zero_fill_22 = 23;
parameter zero_fill_23 = 24;
parameter zero_fill_24 = 25;
parameter zero_fill_25 = 26;
parameter zero_fill_26 = 27;
parameter zero_fill_27 = 28;
parameter zero_fill_28 = 29;
parameter zero_fill_29 = 30;
parameter zero_fill_30 = 31;
parameter zero_fill_31 = 32;
parameter zero_fill_32 = 33;
parameter zero_fill_33 = 34;
parameter zero_fill_34 = 35;
parameter zero_fill_35 = 36;
parameter zero_fill_36 = 37;
parameter no_error = 38;
parameter error_state = 39;
parameter setup = 40;

reg [15:0] addr_countp;
reg [15:0] addr_countd;
reg program_mem,data_mem;
integer i;
reg [6:0] inte;
reg [21:0] received;
//reg mclk_int;
//reg dbg_uart_rxd_mem;
reg [15:0] addrpgm;
reg [15:0] addrdat;
//wire neg_reset = ~reset_n;

/*
// the debug receive input is muxed, during the memory self test, the output is disabled, and is released once the memory check is done
always @(memory_check_done or dbg_uart_rxd_in or dbg_uart_rxd_mem)
	if (memory_check_done)
		dbg_uart_rxd = dbg_uart_rxd_in;
	else 
		dbg_uart_rxd = dbg_uart_rxd_mem;
*/
	
/*always @(posedge mclk or posedge neg_reset)
	if (neg_reset)
		mclk_int = 1'b0;
	else 	
		mclk_int = ~mclk_int;*/

// declaration of data to be send through the debug interface as part of memory test
// sync pattern
wire [10:0] sync = {1'b1,1'b1,8'h80,1'b0};
wire [21:0] commandwrite_memctl = {1'b1,1'b1,8'b00000011,1'b0,  1'b1,1'b1,1'b1,1'b1,6'b000100,1'b0}; // write command for mem_data
wire [21:0] commandread_memctl = {1'b1,1'b1,8'b00000001,1'b0,  1'b1,1'b1,1'b1,1'b1,6'b000100,1'b0}; // write command for mem_data
wire [10:0] commandread_memdata = {1'b1,1'b1,1'b0,1'b0,6'b000110,1'b0}; // write command for mem_data
wire [32:0] command_writememcnt = {1'b1,1'b1,8'b00000000,1'b0,  1'b1,1'b1,8'b00000000,1'b0,	1'b1,1'b1,1'b1,1'b0,6'b000111,1'b0}; // for burst write, write memcnt with number of burst
wire [32:0] command_writememaddrp = {1'b1,1'b1,addrpgm[15:8],1'b0,	1'b1,1'b1,addrpgm[7:0],1'b0,	1'b1,1'b1,1'b1,1'b0,6'b000101,1'b0}; // write command for mem_addr program
wire [32:0] command_writememaddrd = {1'b1,1'b1,addrdat[15:8],1'b0,	1'b1,1'b1,addrdat[7:0],1'b0,	1'b1,1'b1,1'b1,1'b0,6'b000101,1'b0}; // write command for mem_addr data
wire [32:0] command_writememdata0 = {1'b1,1'b1,8'h00,1'b0,	1'b1,1'b1,8'h00,1'b0,	1'b1,1'b1,1'b1,1'b0,6'b000110,1'b0}; // write command for mem_data
wire [32:0] command_writememdata1 = {1'b1,1'b1,8'hFF,1'b0,	1'b1,1'b1,8'hFF,1'b0,	1'b1,1'b1,1'b1,1'b0,6'b000110,1'b0}; // write command for mem_data

wire test;//wire test = neg_reset & dbg_en;
wire test_reset;

always @(posedge mclk_int or posedge test_reset)//always @(posedge mclk_int or posedge test) 
begin
	
if (test_reset)
	begin
	inte = 7'h00;
	i=0;
	addr_countp = 16'h0000; // number of memory locations to be checked for program memory
	addr_countd = 16'h0000; // number of memory locations to be checked for data memory
	addrpgm = 16'h0000; // starting address of program memory, counted down from here
	addrdat = 16'h0000; // starting address of data memory, counted up from here
	received = 22'h000000;
	dbg_uart_rxd = 1'b1;
	program_mem = 1'b0;
	data_mem = 1'b0;
	mem_state = setup;
	memory_check_done = 1'b0;//sooshini
	memory_error = 1'b0;//sooshini post synthesis
	end
	

else if (test==1'b1)
begin
	case (mem_state)
		setup: begin
			mem_state = init;
            memory_check_done = 1'b0;//sooshini
			memory_error = 1'b0;//sooshini post synthesis
			end

		init: // state 0
		begin
//			go to state for setting up debug 
			mem_state = dbg_setup;
			inte = 7'hB; // stores the number of bits to be send over the debug interface
			i = 0;
			addr_countp = 16'h1FFF; // number of memory locations to be checked for program memory
			addr_countd = 16'h03FF; // number of memory locations to be checked for data memory
			addrpgm = 16'hFFFF; // starting address of program memory, counted down from here
			addrdat = 16'h0200; // starting address of data memory, counted up from here
			received = 22'h00000;
			dbg_uart_rxd = 1'b1;
			program_mem = 1'b0;
			data_mem = 1'b0;
			memory_check_done = 1'b0;
			memory_error = 1'b0;
		end

		dbg_setup: // state 1 for synchronising the data rate for debug interface
		begin
			if (|inte)
				begin
					mem_state = dbg_setup;
					dbg_uart_rxd = sync[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_1;
					inte = 7'h21;
					i = 0;
				end
		end
		
		zero_fill_1:  // state 2 memory write command
		begin
			if (|inte)
				begin
					mem_state = zero_fill_1;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_2;
					inte = 7'h21;
					i = 0;
				end
		end
		
		zero_fill_2:	// state 3 memory address to write 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_2;
					dbg_uart_rxd = command_writememaddrp[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_3;
					inte = 7'h21;
					i = 0;
				end
		end
		
		zero_fill_3:	// state 4 memory data, writes 0 first
		begin
			if (|inte)
				begin
					mem_state = zero_fill_3;
					dbg_uart_rxd = command_writememdata0[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_4;
					inte = 7'h16;
					i = 0;
				end
		end

		zero_fill_4:	// state 5 memory write command to do a write
		begin
			if (|inte)
				begin
					mem_state = zero_fill_4;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					if (|addr_countp)
						begin
							mem_state = zero_fill_1; // when zero is written to all locations, going to next state for writing 1
							inte = 7'h21;
							i = 0;
							addrpgm = addrpgm - 16'h0002;
							addr_countp = addr_countp - 16'h0001;
						end
					else 
						begin
							mem_state = zero_fill_5;
							i = 0;
							addrpgm = 16'hFFFF;
							addr_countp = 16'h1FFF;
							inte = 7'h21;
						end
				end
		end
		
		zero_fill_5: // state 6 writing for 1
					 // zero is filled in whole memory, now every memory bit, 1 is to be written, then read back and then move on. 
					 // will need three states, one to write a one, then to read a one, if correct, go back to writing state, else move on // to an error state. 
		begin // writing the command
			if (|inte)
				begin
					mem_state = zero_fill_5;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_6;
					inte = 7'h21;
					i = 0;
				end
		end
			
		zero_fill_6: // state 7 writing the address
		begin
			if (|inte)
				begin
					mem_state = zero_fill_6;
					dbg_uart_rxd = command_writememaddrp[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_7;
					inte = 7'h21;
					i = 0;
				end
		end

		zero_fill_7: // state 8 writing data as 1
		begin
			if (|inte)
				begin
					mem_state = zero_fill_7;
					dbg_uart_rxd = command_writememdata1[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_8;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_8: // writing the command
		begin
			if (|inte)
				begin
					mem_state = zero_fill_8;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_9;
					inte = 7'h16;
					i = 0;
				end
		end
// finished filling with 1 at one location, now to read from the location

		zero_fill_9: // command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_9;
					dbg_uart_rxd = commandread_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_10;
					inte = 7'hB;
					i = 0;
				end
		end
		
		zero_fill_10: // giving command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_10;
					dbg_uart_rxd = commandread_memdata[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_11;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_11:	// reading in the data and checking with the expected correct pattern
		begin
			if (|inte)
				begin
					mem_state = zero_fill_11;
					received[i] = dbg_uart_txd;
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else 
				begin
					if ((received) == 22'h3FEFFD)
						begin
							if (|addr_countp)
								begin
									mem_state = zero_fill_5;
									addrpgm = addrpgm - 16'h0002;
									addr_countp = addr_countp - 16'h0001;
									i = 0;
									inte = 7'h21;
								end
							else 
								begin
									mem_state = zero_fill_12; // checking done for bit 1
									i = 0;
									inte = 7'h21;
									addrpgm = 16'hFFFF;
									addr_countp = 16'h1FFF;
								end
						end
					else
						begin
							mem_state = error_state; // going to an error state in case of an error
						end
				end
		end
		
// finished checking for 1. now checking for 0, 
		zero_fill_12: // sending the command
		begin
			if (|inte)
				begin
					mem_state = zero_fill_12;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_13;
					inte = 7'h21;
					i = 0;
				end
		end
			
		zero_fill_13: // address to write into
		begin
			if (|inte)
				begin
					mem_state = zero_fill_13;
					dbg_uart_rxd = command_writememaddrp[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_14;
					inte = 7'h21;
					i = 0;
				end
		end

		zero_fill_14: // sending the data 0
		begin
			if (|inte)
				begin
					mem_state = zero_fill_14;
					dbg_uart_rxd = command_writememdata0[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_15;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_15:
		begin
			if (|inte)
				begin
					mem_state = zero_fill_15;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_16;
					inte = 7'h16;
					i = 0;
				end
		end

		zero_fill_16: // command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_16;
					dbg_uart_rxd = commandread_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_17;
					inte = 7'hB;
					i = 0;
				end
		end
		
		zero_fill_17: // giving command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_17;
					dbg_uart_rxd = commandread_memdata[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_18;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_18:	// reading in the data and comparing with expected pattern
		begin
			if (|inte)
				begin
					mem_state = zero_fill_18;
					received[i] = dbg_uart_txd;
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else 
				begin
					if ((received) == 22'h200C01)
						begin
							if (|addr_countp)
								begin
									mem_state = zero_fill_12;
									addrpgm = addrpgm - 16'h0002;
									addr_countp = addr_countp - 16'h0001;
									i = 0;
									inte = 7'h21;
								end
							else 
								begin
									mem_state = zero_fill_19;
									i = 0;
									inte = 7'h21;
								end
						end
					else
						begin
							mem_state = error_state;
							program_mem = 1'b1;
						end
				end
		end
		
// program memory check over going to start data memory check
// starts with writing zero into all locations

		zero_fill_19:  // state 2
		begin
			if (|inte)
				begin
					mem_state = zero_fill_19;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_20;
					inte = 7'h21;
					i = 0;
				end
		end
		
		zero_fill_20:	// state 3
		begin
			if (|inte)
				begin
					mem_state = zero_fill_20;
					dbg_uart_rxd = command_writememaddrd[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_21;
					inte = 7'h21;
					i = 0;
				end
		end
		
		zero_fill_21:	// state 4
		begin
			if (|inte)
				begin
					mem_state = zero_fill_21;
					dbg_uart_rxd = command_writememdata0[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_22;
					inte = 7'h16;
					i = 0;
				end
		end

		zero_fill_22:	// state 5
		begin
			if (|inte)
				begin
					mem_state = zero_fill_22;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					if (|addr_countd)
						begin
							mem_state = zero_fill_19;
							inte = 7'h21;
							i = 0;
							addrdat = addrdat + 16'h0002;
							addr_countd = addr_countd - 16'h0001;
						end
					else 
						begin
							mem_state = zero_fill_23;
							i = 0;
							addrdat = 16'h0200;
							addr_countd = 16'h03FF;
							inte = 7'h21;
						end
				end
		end
		
		zero_fill_23: // state for writing for 1
					 // zero is filled in whole memory, now every memory bit, 1 is to be written, then read back and then move on. 
					 // will need three states, one to write a one, then to read a one, if correct, go back to writing state, else move on // to an error state. 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_23;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_24;
					inte = 7'h21;
					i = 0;
				end
		end
			
		zero_fill_24: // state 7 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_24;
					dbg_uart_rxd = command_writememaddrd[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_25;
					inte = 7'h21;
					i = 0;
				end
		end

		zero_fill_25: // state 8
		begin
			if (|inte)
				begin
					mem_state = zero_fill_25;
					dbg_uart_rxd = command_writememdata1[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_26;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_26:
		begin
			if (|inte)
				begin
					mem_state = zero_fill_26;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_27;
					inte = 7'h16;
					i = 0;
				end
		end
// finished filling with 1, now to read 		

		zero_fill_27: // cnt for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_27;
					dbg_uart_rxd = commandread_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_28;
					inte = 7'hB;
					i = 0;
				end
		end
		
		zero_fill_28: // giving command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_28;
					dbg_uart_rxd = commandread_memdata[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_29;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_29:	// reading in the data and checking with pattern
		begin
			if (|inte)
				begin
					mem_state = zero_fill_29;
					received[i] = dbg_uart_txd;
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else 
				begin
					if ((received) == 22'h3FEFFD)
						begin
							if (|addr_countd)
								begin
									mem_state = zero_fill_23;
									addrdat = addrdat + 16'h0002;
									addr_countd = addr_countd - 16'h0001;
									i = 0;
									inte = 7'h21;
								end
							else 
								begin
									mem_state = zero_fill_30;
									i = 0;
									inte = 7'h21;
									addrdat = 16'h0200;
									addr_countd = 16'h03FF;
								end
						end
					else
						begin
							mem_state = error_state;
						end
				end
		end
// finished checking for 1. now checking for 0
		zero_fill_30: 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_30;
					dbg_uart_rxd = command_writememcnt[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_31;
					inte = 7'h21;
					i = 0;
				end
		end
			
		zero_fill_31: 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_31;
					dbg_uart_rxd = command_writememaddrd[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_32;
					inte = 7'h21;
					i = 0;
				end
		end

		zero_fill_32: 
		begin
			if (|inte)
				begin
					mem_state = zero_fill_32;
					dbg_uart_rxd = command_writememdata0[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_33;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_33:
		begin
			if (|inte)
				begin
					mem_state = zero_fill_33;
					dbg_uart_rxd = commandwrite_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_34;
					inte = 7'h16;
					i = 0;
				end
		end

		zero_fill_34: // cnt for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_34;
					dbg_uart_rxd = commandread_memctl[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_35;
					inte = 7'hB;
					i = 0;
				end
		end
		
		zero_fill_35: // giving command for reading mem data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_35;
					dbg_uart_rxd = commandread_memdata[i];	
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else
				begin
					mem_state = zero_fill_36;
					inte = 7'h16;
					i = 0;
				end
		end
		
		zero_fill_36:	// reading in the data
		begin
			if (|inte)
				begin
					mem_state = zero_fill_36;
					received[i] = dbg_uart_txd;
					i = i + 1;
					inte = inte - 7'b0000001;
				end
			else 
				begin
					if ((received) == 22'h200C01)
						begin
							if (|addr_countd)
								begin
									mem_state = zero_fill_30;
									addrdat = addrdat + 16'h0002;
									addr_countd = addr_countd - 16'h0001;
									i = 0;
									inte = 7'h21;
								end
							else 
								begin
									mem_state = no_error;
									i = 0;
									inte = 7'h10;
								end
						end
					else
						begin
							mem_state = error_state;
							data_mem = 1'b1;
						end
				end
		end
		
		no_error:
			begin
				memory_error = 1'b0;
				memory_check_done = 1'b1;
				mem_state = no_error;
			end	
				
		error_state:
			begin
				memory_error = 1'b1;                  //changed to 1'b0
				memory_check_done = 1'b1;
				mem_state = error_state;
			end

		default: mem_state = setup;
	endcase
end

else mem_state = mem_state;

end

endmodule