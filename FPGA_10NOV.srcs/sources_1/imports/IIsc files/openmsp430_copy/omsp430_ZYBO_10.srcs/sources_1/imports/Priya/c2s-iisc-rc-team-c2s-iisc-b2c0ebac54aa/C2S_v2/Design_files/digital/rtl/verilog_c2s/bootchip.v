`timescale 1ns / 1ps

// chip has two modes, flash program mode and normal mode.
// normal mode - first do memory self test, and then load the program from flash to SRAM
// flash program mode - first do memory self test, then allow SRAM to be loaded thru debug interface and once its loaded, transfer program to flash

module bootchip(
    dbg_en,cpu_en,mclk,reset_n,poweron_reset,dma_addr,dma_en,dma_priority,dma_we,dma_wkup,dma_ready,
    dma_resp,memory_check_done,memory_error,flash_addr,flash_wen,flash_cen,flash_oen,flash_program_mode,
    normal_mode,program_loaded,start_load,dbg_uart_rxd_in,dbg_uart_rxd,flash_busy,done,sync,data_width,busy_normal,test,mclk_int);

input           mclk;
output  [14:0]  flash_addr;
output          dbg_en;
output          cpu_en;
output          reset_n;
input           poweron_reset; // reset to the chip
input           flash_program_mode,normal_mode; // the two modes of operation
input           program_loaded; // this reads in the status of SRAM programming in flash program mode
output          start_load; // this indicates the external programming of SRAM to start

output  [15:1] dma_addr;               // Direct Memory Access address
output         dma_en;                 // Direct Memory Access enable (high active)
output         dma_priority;           // Direct Memory Access priority (0:low / 1:high)
output  [1:0]  dma_we;                 // Direct Memory Access write byte enable (high active)
output         dma_wkup;
input          dma_ready;              // Direct Memory Access is complete
input          dma_resp;               // Direct Memory Access response (0:Okay / 1:Error)
output         flash_wen,flash_oen,flash_cen;
input          memory_check_done;
input          memory_error;
input dbg_uart_rxd_in;
output reg sync;
  reg sync1;

// new added input and outputs
input         flash_busy;
 input busy_normal;
output     reg   dbg_uart_rxd;                      // added to write in flash registers using serial debug interface
output  reg   done;
 output reg [6:0] data_width;
output reg test; //test_reset signal is not used and will be optimised in synthesis. was initially added to reset memcheck module
input wire mclk_int;

wire program_loaded;
reg  dma_en, dma_priority;
reg  cpu_en,dbg_en,reset_n;
reg [1:0] dma_we;
reg dma_wkup;
wire mclk;
//reg mclk_int;
reg write_status;
reg [15:0] dma_count;
reg [14:0] dma_start_address;
reg [15:0] dma_start_address1;
reg [15:0] dma_start_address_rev;
reg [14:0] flash_addr;
reg flash_cen,flash_oen,flash_wen;
wire flash_program_mode,normal_mode;
reg [15:1] dma_addr;
reg start_load;
reg dbg_uart_rxd_boot;
reg [3:0]sync_count;

// at power on reset, enable cpu_en, enable dbg_en, this will start memory self test. 
// once we get memory check done, if no error, then need to start dma addr. for this high priority dma transfer. 
// 
// dma_wkup - to be kept high during transfer
// dma_addr - to be generated [15:1]
// dma_din - 16 bits
// dma_dout - 16 bits
// dma_en - to be kept high when dma transfer is to be used
// dma_we [1:0] high for write transfer
// dma_priority - high for high priority transfer
// dma_ready - high means transfer is done. 
// dma_resp - should be low to indicate transfer is ok

reg  [4:0] core_state;
reg  [4:0] core_state_nxt;
//reg [9:0] count_inst_clk;
reg [6:0] count1;
reg [3:0] count2;
reg [5:0] count_bit;
reg [3:0] count_inst;
reg send_read_inst;
reg send_write_inst;
reg [32:0] inst;
reg [14:0] dma_addr1;
always@(*) 
begin
 dma_addr1= dma_addr;
end

//  ** states ** //

parameter  init_state     		  =5'h00;
parameter  reset_low_state 		  =5'h01;
parameter  mem_selftest_state    	  =5'h02;
parameter  dma_transfer_state_init    	  =5'h03;
parameter  dma_transfer_state 		  =5'h04;
parameter  program_transfer_state	  =5'h05;
parameter  program_write_state		  =5'h06;
parameter  data_transfer_state_init   	  =5'h07;
parameter  dma_data_transfer_state    	  =5'h08;
parameter  data_transfer_state		  =5'h09;
parameter  data_write_state		  =5'h0A;
parameter  normal_state1		  =5'h0B;
parameter  normal_state2		  =5'h0C;
parameter  normal_state3		  =5'h0D;
parameter  mem_error_state        	  =5'h0E;
parameter  intermediate_state		  =5'h0F;
parameter  flash_program_state1		  =5'h10;
parameter  flash_program_state2		  =5'h11;
parameter  flash_loading1_init        	  =5'h12;
parameter  flash_loading1		  =5'h13;
parameter  program_flash_transfer	  =5'h14;
parameter  program_flash_write		  =5'h15;
parameter  data_flash_state_init     	  =5'h16;
parameter  data_flash_transfer		  =5'h17;
parameter  data_flash_write		  =5'h18;
parameter  dbg_sync_state		  =5'h19;
parameter  memerror_check_state		  =5'h1A;
parameter  reset_high_state1               =5'h1B;
parameter  reset_high_state2               =5'h1C;

// Parameters used in flash writing //

parameter mem_cnt_addr 	= 6'b111000;	// 000111;
parameter mem_addr_addr	= 6'b101000;	// 000101;
parameter mem_data_addr	= 6'b011000;	// 000110;
parameter mem_ctl_addr	= 6'b001000;	// 000100;
parameter addr_FMA_high = 16'h0980;	// 0000000110010000;
parameter addr_FMA_low 	= 16'h4980;	// 0000000110010010;
parameter addr_FMC 	= 16'h2980;	// 0000000110010100;
parameter write_opcode  = 8'h40	;	// 00000010;
parameter read_opcode   = 8'hc0	;	// 00000011;
parameter FMC_data_read = 8'h44	;	// 00100010; 
parameter FMC_data_write= 8'b10000100; 
parameter size 		    = 8'hfe;	// 01111111;
parameter addr_high	    = 8'b0;
parameter write		    = 1'b1;
parameter read		    = 1'b0;
parameter bit8		    = 1'b1;
parameter bit16		    = 1'b0;
parameter memory	    = 1'b0;
parameter regist	    = 1'b1;
parameter start		    = 1'b1;
parameter single_access = 16'h0;



/*always@(*)
begin
   if((dma_start_address ==  15'h0100) |(dma_start_address ==  15'h6000) )
      dma_start_address1 = dma_start_address;
   else 
        dma_start_address1 = dma_start_address +  15'h007F;
end
*/

//   reversing dma_start_address //

always@(*) 
begin
	if (poweron_reset)
		dma_start_address_rev = 16'h0;
	else
		begin
			dma_start_address_rev[0] = dma_start_address1[15];
			dma_start_address_rev[1] = dma_start_address1[14];
			dma_start_address_rev[2] = dma_start_address1[13];
			dma_start_address_rev[3] = dma_start_address1[12];
			dma_start_address_rev[4] = dma_start_address1[11];
			dma_start_address_rev[5] = dma_start_address1[10];
			dma_start_address_rev[6] = dma_start_address1[9];
			dma_start_address_rev[7] = dma_start_address1[8];
			dma_start_address_rev[8] = dma_start_address1[7];
			dma_start_address_rev[9] = dma_start_address1[6];
			dma_start_address_rev[10] = dma_start_address1[5];
			dma_start_address_rev[11] = dma_start_address1[4];
			dma_start_address_rev[12] = dma_start_address1[3];
			dma_start_address_rev[13] = dma_start_address1[2];
			dma_start_address_rev[14] = dma_start_address1[1];
			dma_start_address_rev[15] = dma_start_address1[0];
		end

end

/*always@(*) dma_start_address_rev[0] = dma_start_address1[15];
always@(*) dma_start_address_rev[1] = dma_start_address1[14];
always@(*) dma_start_address_rev[2] = dma_start_address1[13];
always@(*) dma_start_address_rev[3] = dma_start_address1[12];
always@(*) dma_start_address_rev[4] = dma_start_address1[11];
always@(*) dma_start_address_rev[5] = dma_start_address1[10];
always@(*) dma_start_address_rev[6] = dma_start_address1[9];
always@(*) dma_start_address_rev[7] = dma_start_address1[8];
always@(*) dma_start_address_rev[8] = dma_start_address1[7];
always@(*) dma_start_address_rev[9] = dma_start_address1[6];
always@(*) dma_start_address_rev[10] = dma_start_address1[5];
always@(*) dma_start_address_rev[11] = dma_start_address1[4];
always@(*) dma_start_address_rev[12] = dma_start_address1[3];
always@(*) dma_start_address_rev[13] = dma_start_address1[2];
always@(*) dma_start_address_rev[14] = dma_start_address1[1];
always@(*) dma_start_address_rev[15] = dma_start_address1[0];
*/
// 	*********************		//

//  Instructions for flash writing //

wire [10:0] sync_dbg_uart = {1'b1,1'b0,8'h01,1'b1}; 

// ** writing in flash addr_FMA_high **  //
wire [32:0] inst1  = {1'b1,1'b0,mem_cnt_addr,bit16, write,1'b1,1'b1,1'b0,single_access[15:8],1'b1,1'b1,1'b0,single_access[7:0],1'b1};
wire [32:0] inst2  = {1'b1,1'b0,mem_addr_addr,bit16,write,1'b1,1'b1,1'b0,addr_FMA_high[15:8],1'b1,1'b1,1'b0,addr_FMA_high[7:0],1'b1};
wire [32:0] inst3  = {1'b1,1'b0,mem_data_addr,bit16,write,1'b1,1'b1,1'b0,addr_high,1'b1,1'b1,1'b0,size,1'b1};
wire [32:0] inst4  = {1'b1,1'b0,mem_ctl_addr, bit8, write,1'b1,1'b1,1'b0,start,write,memory,bit16,4'b0000,1'b1,11'h7FF};  
//	 *********************		//

// ** writing in flash addr_FMA_low **  //
wire [32:0] inst5  = {1'b1,1'b0,mem_cnt_addr,bit16, write,1'b1,1'b1,1'b0,single_access[15:8],1'b1,1'b1,1'b0,single_access[7:0],1'b1};
wire [32:0] inst6  = {1'b1,1'b0,mem_addr_addr,bit16,write,1'b1,1'b1,1'b0,addr_FMA_low[15:8],1'b1,1'b1,1'b0,addr_FMA_low[7:0],1'b1};
wire [32:0] inst7  = {1'b1,1'b0,mem_data_addr,bit16,write,1'b1,1'b1,1'b0,dma_start_address_rev[15:8],1'b1,1'b1,1'b0,dma_start_address_rev[7:0],1'b1};
wire [32:0] inst8  = {1'b1,1'b0,mem_ctl_addr, bit8, write,1'b1,1'b1,1'b0,start,write,memory,bit16,4'b0000,1'b1,11'h7FF};  
//	 *********************		//

// ** writing in flash FMC for read **  //
wire [32:0] inst9  = {1'b1,1'b0,mem_cnt_addr,bit16, write,1'b1,1'b1,1'b0,single_access[15:8],1'b1,1'b1,1'b0,single_access[7:0],1'b1};
wire [32:0] inst10 = {1'b1,1'b0,mem_addr_addr,bit16,write,1'b1,1'b1,1'b0,addr_FMC[15:8],1'b1,1'b1,1'b0,addr_FMC[7:0],1'b1};
wire [32:0] inst11 = {1'b1,1'b0,mem_data_addr,bit16,write,1'b1,1'b1,1'b0,read_opcode,1'b1,1'b1,1'b0,FMC_data_read,1'b1};
wire [32:0] inst12 = {1'b1,1'b0,mem_ctl_addr, bit8, write,1'b1,1'b1,1'b0,start,write,memory,bit16,4'b0000,1'b1,11'h7FF};  
//	 *********************		//
 
// ** writing in flash FMC for write **  //
wire [32:0] inst13  = {1'b1,1'b0,mem_cnt_addr,bit16, write,1'b1,1'b1,1'b0,single_access[15:8],1'b1,1'b1,1'b0,single_access[7:0],1'b1};
wire [32:0] inst14 = {1'b1,1'b0,mem_addr_addr,bit16,write,1'b1,1'b1,1'b0,addr_FMC[15:8],1'b1,1'b1,1'b0,addr_FMC[7:0],1'b1};
wire [32:0] inst15 = {1'b1,1'b0,mem_data_addr,bit16,write,1'b1,1'b1,1'b0,write_opcode,1'b1,1'b1,1'b0,FMC_data_write,1'b1};
wire [32:0] inst16 = {1'b1,1'b0,mem_ctl_addr, bit8, write,1'b1,1'b1,1'b0,start,write,memory,bit16,4'b0000,1'b1,11'h7FF};  
//	 *********************		//


///////   **************  //////////



// the debug receive input is muxed, during the memory self test, the output is disabled, and is released once the memory check is done
//always @(*)
//	if (memory_check_done)
//		dbg_uart_rxd = dbg_uart_rxd_in;
//	else 
//		dbg_uart_rxd = dbg_uart_rxd_boot;
    //assign dbg_uart_rxd = memory_check_done ?  dbg_uart_rxd_boot : dbg_uart_rxd_in; // CONFIRM
    
// ** generating half clock for debug interface ** //

always@(*) begin
	if (poweron_reset) dbg_uart_rxd = 1'b1;
	else if (memory_check_done) begin
			if((core_state == flash_program_state1)|(core_state == flash_program_state2))
				dbg_uart_rxd = dbg_uart_rxd_in;
			else if (core_state == normal_state3)
				dbg_uart_rxd = dbg_uart_rxd_in;
			else
	            			dbg_uart_rxd = dbg_uart_rxd_boot;
                        end
	else 
		dbg_uart_rxd = dbg_uart_rxd_in;
end

/*always @(posedge mclk)
  if (poweron_reset) 
		mclk_int <= 1'b1;
  else         
		mclk_int <= ~(mclk_int);*/

//Input to SPI
//sooshini

always @(posedge mclk_int)//always @(negedge mclk_int)
  begin
    if (poweron_reset==1'b1)
	sync1=1'b0;
    else if(dma_addr1 == 15'h04FF)
       sync1=1'b0;
    else if((dma_addr1 >= 15'h6000)|(dma_addr1 >= 15'h0100))
      sync1=1'b1;
  
    else
       sync1=1'b0;

  end
always@(*)
 begin
    if (poweron_reset==1'b1)
	sync<=1'b0;
     else if(normal_mode & ((core_state ==dma_transfer_state )| (core_state ==dma_data_transfer_state )) )
           sync<=1'b1;
    else if(flash_program_mode)
        sync<=sync1;
    else
       sync<=1'b0;
    
 end 





// ** sending data to debug interface ** //

always @(posedge mclk_int)
  if (poweron_reset) begin 
		dbg_uart_rxd_boot <= 1'b1;
		count_bit <= 6'b100001;
		count_inst <= 4'b0;
	end
/*  else if (~(reset_n)) begin
		dbg_uart_rxd_boot <= 1'b1;
		count_bit <= 6'b100001;
		count_inst <= 4'b0;
	end
*/
  else if(sync_count!=4'd0)
   begin
    dbg_uart_rxd_boot <= sync_dbg_uart[sync_count-4'd1] ;
   end
  else if(send_read_inst | send_write_inst) begin
		if (count_bit == 6'b100001)
			begin
				dbg_uart_rxd_boot <= 1'b1;
				count_bit <= count_bit - 6'b1;
			end
		else if (|count_bit)
			begin
				dbg_uart_rxd_boot <= inst[count_bit];
				count_bit <= count_bit - 6'b1;
			end
		else
			begin
				dbg_uart_rxd_boot <= inst[count_bit];	
				count_bit <= 6'b100000;
			end 
		if (count_bit == 6'b1)
			count_inst <=  count_inst + 4'b1; 
		else
			count_inst <= count_inst;
	end
  else if ((core_state == flash_program_state1) | (core_state == flash_program_state2))  //sooshini& Priya
   begin
      dbg_uart_rxd_boot <= dbg_uart_rxd_in;
   end
/*	else if(send_write_inst) begin
		if (|count_bit)
			begin
				dbg_uart_rxd_boot <= inst[count_bit];	
				count_bit <= 6'b0;
				count_inst <=  count_inst - 4'b1;
			end
		else
			begin
				dbg_uart_rxd_boot <= inst[count_bit];	
				count_bit <= count_bit + 6'b1;
				count_bit <= 6'b100000;
			end  
		end
*/
    else begin
        dbg_uart_rxd_boot <= 1'b1;
        count_bit <= 6'b100001;
        count_inst <= 4'b0;
    end

// ** selection of instruction for sending to debug interface ** //

always @(posedge mclk_int)
	if(poweron_reset)
		inst <= 33'b111111111111111111111111111111111;
	//		else if (~(reset_n)) 
//		inst <= 33'b111111111111111111111111111111111;
	else begin
		if(send_read_inst) begin
			case (count_inst) 
				4'b0: inst  <= inst1;
				4'd1: inst  <= inst2;
				4'd2: inst  <= inst3;
				4'd3: inst  <= inst4;
				4'd4: inst  <= inst5;
				4'd5: inst  <= inst6;
				4'd6: inst  <= inst7;
				4'd7: inst  <= inst8;
				4'd8: inst  <= inst9;
				4'd9: inst  <= inst10;
				4'd10: inst <= inst11;
				4'd11: inst <= inst12;
				default: inst <= 33'b111111111111111111111111111111111;
			endcase
		end
		else if(send_write_inst) begin
			case (count_inst)
				4'b0: inst  <= inst1;
				4'd1: inst  <= inst2;
				4'd2: inst  <= inst3;
				4'd3: inst  <= inst4;
				4'd4: inst  <= inst5;
				4'd5: inst  <= inst6;
				4'd6: inst  <= inst7;
				4'd7: inst  <= inst8;
				4'd8: inst  <= inst13;
				4'd9: inst  <= inst14;
				4'd10: inst <= inst15;
				4'd11: inst <= inst16;
				default: inst <= 33'b111111111111111111111111111111111;
			endcase
        	end 
		else	inst <= 33'b111111111111111111111111111111111;
	end

// ** state change ** //

always @(posedge mclk)
  if (poweron_reset) 
	core_state <= init_state;
  else         
	core_state <= core_state_nxt;

// **  Output logic ** //

always @(posedge mclk)
  if (poweron_reset) begin
      //  write_status <= 1'b1;
        cpu_en <= 1'b0;
        start_load <= 1'b0;
        dbg_en <= 1'b0;
        reset_n <= 1'b1;
       // flash_cen <= 1'b0;
      //  flash_oen <= 1'b0;
       // flash_wen <= 1'b0;
        flash_addr <= 15'h0000;	
        send_read_inst <= 1'b0;
        done <= 1'b0;
        dma_start_address <= 15'h0000;
        dma_start_address1 <= 16'h0000;//dma_start_address1 <= 15'h0000;
        //count1 <= 6'b111111;
          count1 <= 7'b1000000;
         	
        //count2 <= 4'b0111;
        count2 <= 4'b1000;
        
        send_read_inst <= 1'b0;	
        dma_we <= 2'b11;
        dma_priority <= 1'b1;
        dma_wkup <= 1'b0;//dma_wkup <= 1'b1; //PRIYA AND DEVANJAN
        dma_count <= 16'h0000;	
        dma_addr <= 15'h0000; // this is to be routed to flash and dma interface of core. 
        flash_wen <= 1'b0; // indicating read status
        flash_cen <= 1'b1; // flash enable
        flash_oen <= 1'b1; // flash output enable
        dma_en <= 1'b1;
        send_write_inst <= 1'b0;
	test<=1'b0;
	//test_reset=1'b1; //to reset memtest module	
		
  end
  else if(core_state ==  init_state ) begin
      //  write_status <= 1'b1;
        cpu_en <= 1'b0;
        start_load <= 1'b0;
       dbg_en <= 1'b1; //to avoid uncertainty in dma_resp signal
        reset_n <= 1'b0;//reset_n <= 1'b1; //giving double reset to the core to ensure no x propagation happens on startup
        //flash_cen <= 1'b0;
        //flash_oen <= 1'b0;
        //flash_wen <= 1'b0;
        flash_addr <= 15'h0000;	
        send_read_inst <= 1'b0;
        done <= 1'b0;
        dma_start_address <= 15'h6000;
        dma_start_address1 <=16'h6000;//dma_start_address1 <=15'h6000;
        //count1 <= 6'b111111;
        count1 <= 7'b1000000;	
        //count2 <= 4'b0111;
        count2 <= 4'b1000;
       
        send_read_inst <= 1'b0;	
        dma_we <= 2'b11;
        dma_priority <= 1'b1;
        dma_wkup <= 1'b0;//dma_wkup <= 1'b1;//PRIYA AND DEVANJAN
        dma_count <= 16'h0000;	
        dma_addr <= 15'h0000; // this is to be routed to flash and dma interface of core. 
        flash_wen <= 1'b0; // indicating read status
        flash_cen <= 1'b1; // flash enable
        flash_oen <= 1'b1; // flash output enable
        dma_en <= 1'b1;
        send_write_inst <= 1'b0;	
	//test_reset=1'b0;
    end

else if(core_state ==  reset_high_state1 ) begin
        reset_n <= 1'b1;
     end


  else if(core_state ==  reset_low_state ) begin
        reset_n <= 1'b0;
      dbg_en <= 1'b0; // dbg_en <= 1'b1;
	//test_reset=1'b0;
    end

else if(core_state ==  reset_high_state2 ) begin
        reset_n <= 1'b1;
     end

  else if(core_state ==  mem_selftest_state ) begin
        cpu_en <= 1'b1;	
        dbg_en <= 1'b1;
        reset_n <= 1'b1;
	test<=1'b1;
	//test_reset=1'b0;
    end
  else if(core_state ==  intermediate_state ) begin
        dma_start_address <= 15'h6000;
        dma_start_address1 <= 16'h6000;//dma_start_address1 <= 15'h6000;
        //cpu_en <= 1'b0;			// to generate dma_ready 
	test<=1'b0;
	//test_reset=1'b0;
    end
  else if(core_state ==  dma_transfer_state_init ) begin
        send_read_inst <= 1'b1;	
        done <= 1'b0;
        //done <= 1'b1;//sooshini
//      count1 <= count1 - 6'b1;
    end  
  else if(core_state ==  dma_transfer_state ) begin
        send_read_inst <= 1'b0;
        dma_we <= 2'b11;
        dma_priority <= 1'b1;
        dma_wkup <= 1'b1;
           if(count1 == 7'h00) 
             dma_count <= 16'h003F;
           else dma_count <= 16'h007F;		//changed to new from 1FFF
        dma_start_address <= dma_start_address;  
        done <= 1'b0;
    end
  else if(core_state ==  program_transfer_state ) 
	begin
		if(dma_count<=16'h0001)
		    done<=1'b1;
		else
		    done<=done;
		if (~(|dma_count)) 
		begin
		     count1 <= count1 - 6'b1;
		     if (dma_start_address == 15'h7F41) dma_count <= 16'h003F; //if (dma_start_address == 16'h7F41) dma_count <= 16'h003F; 
		     else   dma_count <= 16'h007F;		//changed to new from 3FF
		     if(~(|count1))
		     begin 
		        dma_start_address <= 15'h0100;
		        dma_start_address1 <= 16'h0100;//dma_start_address1 <= 15'h0100;
		     end
		     else
		     begin
		        dma_start_address <= dma_start_address + 15'h007F;//sooshini
		        dma_start_address1 <= dma_start_address1 +16'h00FE;//dma_start_address1 <= dma_start_address1 +15'h00FE;
		     end
		end
		else 
		begin
			if (write_status) 
			begin
		        dma_addr <= dma_start_address; // this is to be routed to flash and dma interface of core. 
		        flash_addr <= dma_start_address;
		    	end
		    	else 
			begin
		        dma_addr <= dma_addr + 15'h0001;
		        flash_addr <= flash_addr + 15'h0001;
		        dma_count <= dma_count -16'h0001;
		    	end
		end 
    	end     
  else if(core_state ==  program_write_state ) 
	begin
		if(dma_count<=16'h0001)
		    done<=1'b1;
		else 
		    done<=done;
	   	if (~dma_ready) 
		begin
		        flash_wen <= 1'b0; // indicating read status
		        flash_cen <= 1'b1; // flash enable
		        flash_oen <= 1'b1; // flash output enable
		        dma_en <= 1'b1;
		end
		else
		begin
			flash_wen <= flash_wen;
		        flash_cen <= flash_cen;
		        flash_oen <= flash_oen;
		        dma_en <= dma_en;
		end
	end
  else if(core_state ==  data_transfer_state_init ) begin
        send_read_inst <= 1'b1;	
        done <= 1'b0;				
//        count2 <= count2 - 4'b1;
    end
  else if(core_state ==  dma_data_transfer_state ) begin
        send_read_inst <= 1'b0;
        dma_we <= 2'b11;
        dma_priority <= 1'b1;
        dma_wkup <= 1'b1;
        if(dma_start_address==15'h04F8) dma_count<=7'h07;//if(dma_start_address==16'h04F8) dma_count<=7'h07; 
        else dma_count<=7'h7F;
      end

  else if(core_state ==  data_transfer_state ) 
	begin
		if (~(|dma_count)) 
			if(~(|count2)) 
			dma_en <= 1'b0;	//sooshini 1st july
			else
			dma_en <=dma_en;
		else
		     dma_en <=dma_en;	

		if ((dma_count<=16'h0001)|(~(|dma_count)) )
			done<=1'b1;
		else done<=done;
		 

		/*if(dma_count<=16'h0001)
		        done<=1'b1;
		else done<=done;*/

		if (~(|dma_count)) 
		begin
			count2 <= count2 - 4'b1;
		    //	if(~(|count2)) 
		       // done <= 1'b1;				
		    //	else 
			if ((|count2)) begin
		        	if(dma_start_address==15'h04F8) 
				dma_count<=7'h07;//if(dma_start_address==16'h04F8) dma_count<=7'h07;
		         	else 
				dma_count<=7'h7F;
			//done <= 1'b1;
			dma_start_address <= dma_start_address + 15'h007F;
			dma_start_address1 <= dma_start_address1 +16'h00FE;//dma_start_address1 <= dma_start_address1 +15'h00FE;
			end
			else 
			begin
				dma_start_address <= dma_start_address;
				dma_start_address1 <= dma_start_address1;
				dma_count <= dma_count;
			end
		end

		else begin
		count2 <= count2;
		    if (write_status) 
			begin
		        dma_addr <= dma_start_address; // this is to be routed to flash and dma interface of core. 
		        flash_addr <= dma_start_address;
		        end
		    else begin
		        dma_addr <= dma_addr + 15'h0001;
		        flash_addr <= flash_addr + 15'h0001;
		        dma_count <= dma_count - 16'h0001;
			end
		     end
    	end

  else if(core_state ==  data_write_state ) 
	begin
        if(dma_count<=16'h0001)
                done<=1'b1;
	else
		done<=done;
	if (~dma_ready) 
		begin //   else begin
                flash_wen <= 1'b0; // indicating read status
                flash_cen <= 1'b1; // flash enable
                flash_oen <= 1'b1; // flash output enable
                dma_en <= 1'b1;
        	end
	else
		begin //   else begin
                flash_wen <= flash_wen;
                flash_cen <= flash_cen;
                flash_oen <= flash_oen;
                dma_en <= dma_en;
        	end
    	end
  else if(core_state ==  normal_state1 ) begin
        reset_n <= 1'b0;
        dbg_en <= 1'b0;
	dma_en <= 1'b0;
        flash_wen <= 1'b0;
        flash_cen <= 1'b0;
        flash_oen <= 1'b0;
        done	  <=1'b0; 
        cpu_en <= 1'b1;
    end
  else if(core_state ==  normal_state2 ) begin
        reset_n <= 1'b1;
        dbg_en <= 1'b0;
    end
  else if(core_state ==  normal_state3 ) begin
        dbg_en <= 1'b0;
	 dma_en <= 1'b0;
        dma_priority <= 1'b0;//sooshini
    end
  else if(core_state ==  mem_error_state ) begin
        cpu_en <= 1'b0;
    end
  else if(core_state ==  flash_program_state1 ) begin
        dbg_en<=1'b0;
	send_write_inst <= 1'b0;	
        start_load <= 1'b1;
    end
   else if(core_state ==  flash_program_state2 ) begin
        send_write_inst <= 1'b0;
        if(program_loaded)
           dbg_en<=1'b1;	
    end
  else if(core_state ==  dbg_sync_state ) begin
        dbg_en<=1'b1;
    end
  else if(core_state ==  flash_loading1_init ) begin
  // dbg_en<=1'b1;
	send_write_inst <= 1'b1;	
        done <= 1'b0;
//        count1 <= count1 - 6'b1;
    end
  else if(core_state ==  flash_loading1 ) begin
        send_write_inst <= 1'b0	;
        flash_wen <= 1'b1; // indicating write status
        flash_cen <= 1'b1; // flash enable
        flash_oen <= 1'b0; // flash output enable
        dma_en <= 1'b1;
        dma_we <= 2'b00; // dma_we for read transfer
        dma_priority <= 1'b1;
        dma_wkup <= 1'b1;
       if (dma_start_address == 15'h7FC0) dma_count <= 16'h003F;//if (dma_start_address == 16'h7FC0) dma_count <= 16'h003F; //sooshini
             else   dma_count <= 16'h007F;	// changed
        dma_start_address <= dma_start_address;	// first time it will be 0x6000
     //   write_status <= 1'b1;
    end
  else if(core_state ==  program_flash_transfer ) 
	begin
        if (~(|dma_count)) 
	begin
        	count1 <= count1 - 6'b1;
	   	if (dma_start_address == 15'h7F41) 
			dma_count <= 16'h003F;//if (dma_start_address == 16'h7F41) dma_count <= 16'h003F;
             	else   dma_count <= 16'h007F;
    		done <= 1'b1;
                if (~(|count1)) 
                begin
		       dma_start_address <= 15'h0100; //dma_start_address <= 16'h0100; //sooshini
		       dma_start_address1 <= 16'h0100;
                end
            	else
                begin
                dma_start_address <= dma_start_address + 15'h007F;
                dma_start_address1 <= dma_start_address1 +16'h00FE;//dma_start_address1 <= dma_start_address1 +15'h00FE;
                end
        end
        else begin
	count1 <= count1;
	done <= done;
            if (write_status) begin// if hi, then repeat transfer
                    dma_addr <= dma_start_address; // this is to be routed to flash, dma interface of core. 
                   flash_addr <= dma_start_address;
            end
            else begin	// if lo, means the next address location is being tried
                    dma_addr <= dma_addr + 15'h0001;
                    flash_addr <= flash_addr + 15'h0001;
                    dma_count <= dma_count -16'h0001;
		end	
        end
    end
  else if(core_state ==  program_flash_write ) begin
      /*  if (dma_ready) begin
            if (dma_resp)	// high means transfer is not normal
                    write_status <= 1'b1;
            else 
                  if(~(flash_busy))
                    write_status <= 1'b0;
        end*/
       if (~dma_ready) begin //else begin
            // here the data is being written into flash
                flash_wen <= 1'b1; // indicating write status
                flash_cen <= 1'b1; // flash enable
                flash_oen <= 1'b0; // flash output enable
                dma_en <= 1'b1;
        end
	else 
	begin
		flash_wen <= flash_wen;
                flash_cen <= flash_cen;
                flash_oen <= flash_oen;
                dma_en <= dma_en;
	end
    end
  else if(core_state ==  data_flash_state_init )
	begin
        send_write_inst <= 1'b1;	
        done <= 1'b0;
        if(dma_addr == 15'h7FFF) dma_addr<=15'h0100;
	else dma_addr<=dma_addr;				
//        count2 <= count2 - 4'b1;
    	end
  else if(core_state ==  data_flash_transfer ) 
	begin
        send_write_inst <= 1'b0;
       	if (~(|dma_count)) 
	begin
		count2 <= count2 - 4'b1;
                if(~(|count2)) 
                    done <= 1'b1;				
                else 
		begin
                    if(count2 == 1'b1) dma_count <= 16'h0007;
                    else dma_count <= 16'h007F;
                  //  write_status <= 1'b1;
                 done <= 1'b1;
                 dma_start_address <= dma_start_address + 15'h007F;
                 dma_start_address1 <= dma_start_address1 +16'h00FE;//dma_start_address1 <= dma_start_address1 +15'h00FE;
                end
        end
        else begin
               	count2<=count2;
		done<=done; 
                if ((write_status)  &( dma_addr != 15'h0100) ) begin
                        dma_addr <= dma_start_address; // this is to be routed to flash and dma interface of core. 
                      flash_addr <= dma_start_address;
                end
                else begin
                        dma_addr <= dma_addr + 15'h0001;
                        flash_addr <= flash_addr + 15'h0001;
                        dma_count <= dma_count -16'h0001;
			end																	
             end
    	end
  else if(core_state ==  data_flash_write ) begin
    
       if (~dma_ready) begin//  else begin
                flash_wen <= 1'b1; // indicating read status
                flash_cen <= 1'b1; // flash enable
                flash_oen <= 1'b0; // flash output enable
                dma_en <= 1'b1;
        end	
	else begin//  else begin
                flash_wen <= flash_wen;
                flash_cen <= flash_cen;
                flash_oen <= flash_oen;
                dma_en <=dma_en;
        end	
    end

else begin
	//write_status <= write_status;
        cpu_en <= cpu_en;
        start_load <= start_load;
        dbg_en <= dbg_en;
        reset_n <= reset_n;
       // flash_cen <= 1'b0;
      //  flash_oen <= 1'b0;
       // flash_wen <= 1'b0;
        flash_addr <= flash_addr;	
        send_read_inst <= send_read_inst;
        done <= done;
        dma_start_address <= dma_start_address ;
        dma_start_address1 <= dma_start_address1;//dma_start_address1 <= 15'h0000;
        //count1 <= 6'b111111;
          count1 <=  count1;
         	
        //count2 <= 4'b0111;
        count2 <=  count2 ;
        
        send_read_inst <= send_read_inst;	
        dma_we <= dma_we;
        dma_priority <= dma_priority;
        dma_wkup <= dma_wkup;//dma_wkup <= 1'b1; //PRIYA AND DEVANJAN
        dma_count <= dma_count;	
        dma_addr <= dma_addr; // this is to be routed to flash and dma interface of core. 
        flash_wen <= flash_wen; // indicating read status
        flash_cen <= flash_cen; // flash enable
        flash_oen <=flash_oen; // flash output enable
        dma_en <=  dma_en;
        send_write_inst <=send_write_inst;
	test<=test;
//	test_reset=test_reset; //to reset memtest module	
end



    
// ** Next state logic ** //

always @(*)
if (poweron_reset)
		core_state_nxt = init_state;
else
  case (core_state)
	init_state			: // state 0
							begin
								core_state_nxt = reset_high_state1;//core_state_nxt = reset_low_state;
									//sync_count=4'd0;
							end

	reset_high_state1		: // state 1
							begin
								core_state_nxt = reset_low_state;
                            				end

	reset_low_state		: // state 1
							begin
								core_state_nxt = reset_high_state2;
							end
	reset_high_state2		: // state 1
							begin
								core_state_nxt = mem_selftest_state;

                            				end
	mem_selftest_state  : begin // state 2
						if (memory_check_done) begin
								if (memory_error)
									core_state_nxt = mem_error_state;
								else 
									core_state_nxt = intermediate_state;					
					    end
						else 
								core_state_nxt = mem_selftest_state;
		    	  end
	


					  
	// depending on programming or execution, accordingly load next state
	intermediate_state	: begin //state 0F
								if (normal_mode) 
									core_state_nxt = dma_transfer_state_init;
								else if (flash_program_mode) 
                                    core_state_nxt = flash_program_state1;
                            else
                                    core_state_nxt = intermediate_state;
						  end


	dma_transfer_state_init	: begin // state 3  write in flash registers
						if(count_inst == 4'b1100) // instead try using	if (count_inst == 4'b1100)
							core_state_nxt = dma_transfer_state;
						else
							core_state_nxt = dma_transfer_state_init;					
				  end				

	dma_transfer_state	: begin // state 4 initialize variable for dma transfer
						if(~(busy_normal))	core_state_nxt = program_transfer_state;
						else			core_state_nxt = dma_transfer_state;
				  end
						  
	program_transfer_state	: 	begin // state 5
						if (~(|dma_count)) begin
								if(~(|count1)) 
									core_state_nxt = data_transfer_state_init;
								else
									core_state_nxt = dma_transfer_state_init;
						end
						else begin
									core_state_nxt = program_write_state;
						end
					end
	
	program_write_state		:	begin // state 6
							if (dma_ready) begin
								if (dma_resp)
										core_state_nxt = program_transfer_state;
								else 
										core_state_nxt = program_transfer_state;
					        end 
							else
									core_state_nxt = program_write_state;
						end

	data_transfer_state_init:	begin								
						if(count_inst == 4'b1100)	//(~(|count_inst_clk)) 
							core_state_nxt = dma_data_transfer_state;
						else
							core_state_nxt = data_transfer_state_init;					
							
					end

	dma_data_transfer_state	: begin // state 3 initialize variable for dma transfer
    					if(~(busy_normal))	
    					   core_state_nxt = data_transfer_state;
						else			
						  core_state_nxt = dma_data_transfer_state;
				  end

	data_transfer_state	: 	begin // state 6
						if (~(|dma_count)) begin
								if(~(|count2)) 
									core_state_nxt = normal_state1;
								else 
									core_state_nxt = data_transfer_state_init;
						end
						else begin
								if (write_status)
										core_state_nxt = data_write_state;
								else 
										core_state_nxt = data_write_state;
						end
					end

	data_write_state		:	begin // state 7
							if (dma_ready) begin
								if (dma_resp)
										core_state_nxt = data_transfer_state;
								else 
										core_state_nxt = data_transfer_state;
							end
							else
									core_state_nxt = data_write_state;
						end
								
// write is over, now dbgen is to be made low and reset to be given again
	normal_state1			:	begin // state 7 reset made low
							core_state_nxt = normal_state2;
						end
						
	normal_state2			:	begin // state 8 reset made high
							core_state_nxt = normal_state3;
						end
								
	normal_state3			:	begin // state 9 // debug enabled, normal mode completed
							core_state_nxt = normal_state3;
						end

	mem_error_state			:	begin	 // state 0F cpu execution disabled in case of an error
                            core_state_nxt = mem_error_state;
                    
						end
	
	flash_program_state1	:   begin // state 10 raises the signal start_load for external programming of the SRAM
						core_state_nxt = flash_program_state2;
					end
	// waiting for external debug interface to load program into the core, program_loaded indicates whether its done
	flash_program_state2	:   begin // state 11
						if (program_loaded)
							begin
							core_state_nxt = dbg_sync_state;
							//sync_count=4'd11;
							//dbg_en=1'b1;
							end
						else
							core_state_nxt = flash_program_state2;
					end

	 dbg_sync_state		:   begin
						if(sync_count==4'd0)
						core_state_nxt = flash_loading1_init;
						else
						begin
							core_state_nxt = dbg_sync_state ;
							//dbg_uart_rxd_boot = sync_dbg_uart[sync_count-4'd1] ;
						end
							/*begin
							dbg_uart_rxd_boot = sync_dbg_uart[sync_count-4'd1] ;
							sync_count=sync_count-4'd1;
							core_state_nxt = dbg_sync_state ;
							end*/

				    end

	flash_loading1_init	: begin //  state 12  write in flash registers
						if (count_inst == 4'b1100)	 //(~(|count_inst_clk)) 
							core_state_nxt = flash_loading1;
						else 
							core_state_nxt = flash_loading1_init;					

				  end				


	// SRAM loaded with program, now to transfer the program to flash
	flash_loading1			:	begin   // state 13
							core_state_nxt = program_flash_transfer;
						end

	program_flash_transfer	: 	begin // state 14
									if (~(|dma_count)) begin
											if (~(|count1)) 
												core_state_nxt = data_flash_state_init;
											else
												core_state_nxt = flash_loading1_init;
									end
									else begin
											if (write_status) // if hi, then repeat transfer
													core_state_nxt = program_flash_write;
											else 
													core_state_nxt = program_flash_write;
									end
								end
	
	program_flash_write		:	begin // state 15
							if (dma_ready) begin
								if (dma_resp)	// high means transfer is not normal
										core_state_nxt = program_flash_transfer;
								else 
										core_state_nxt = program_flash_transfer;
							end
							else
									core_state_nxt = program_flash_write;
						end

	data_flash_state_init:	begin   // state 16
					//if(count_inst == 4'b1100)	//(~(|count_inst_clk)) //sooshini
                                         if(~(flash_busy))
						core_state_nxt = data_flash_transfer;
					else
						core_state_nxt = data_flash_state_init;					
				end

	data_flash_transfer	: 	begin // state 17
						if (~(|dma_count)) begin
                                                                //if((~(|count2))
								if(((count2 == 4'hF)) &(~(flash_busy))) //sooshini
									core_state_nxt = normal_state1;
								else 
									core_state_nxt = data_flash_state_init;
						end
						else begin
								if (write_status)
										core_state_nxt = data_flash_write;
								else 
										core_state_nxt = data_flash_write;
						end
					end
	
	data_flash_write		:	begin // state 18
									if (dma_ready) begin
										if (dma_resp)
												core_state_nxt = data_flash_transfer;
										else 
												core_state_nxt = data_flash_transfer;
									end
									else begin 
											core_state_nxt = data_flash_write;
								    end
								end
	default      		: begin
	       core_state_nxt = init_state;
	       end
    
    endcase


//data_width
//sooshini
always @(*)
  begin
         
         //if ((dma_start_address == 15'h7FC0)& (~(flash_busy))) data_width<=7'h3F;//if ((dma_start_address == 16'h7FC0)& (~(flash_busy))) data_width<=7'h3F;
         if(poweron_reset == 1'b1) data_width<=7'h00;
         else if ((dma_start_address == 15'h7FC0)) data_width<=7'h3F;
         else if(dma_start_address==15'h04F8) data_width<=7'h08;// else if(dma_start_address==16'h04F8) data_width<=7'h08;
         else if ((dma_start_address == 15'h0100) & ( (core_state == 5'h08) |(~(flash_busy)) |((dma_addr > 16'h0100) & (dma_addr < 16'h04F8 )))) data_width<=7'h7F;// else if ((dma_start_address == 16'h0100) & ( (core_state == 5'h08) |(~(flash_busy)) |((dma_addr > 16'h0100) & (dma_addr < 16'h04F8 )))) data_width<=7'h7F;
         else if ((dma_start_address == 15'h0100) )data_width<=7'h3F;// else if ((dma_start_address == 16'h0100) )data_width<=7'h3F;
         else data_width<=7'h7F;
  end

always@(posedge mclk_int)
begin
 if(poweron_reset)
   sync_count<=4'd0;
else if(core_state == flash_program_state2)
    sync_count <= 4'd11;
else if(core_state ==dbg_sync_state)
    sync_count <= sync_count -4'd1;
else 
   sync_count<=4'd0;	
end


always @(posedge mclk)
	if (poweron_reset) 
        	write_status <= 1'b1;

	else if(core_state ==  init_state )
       		write_status <= 1'b1;

 	else if(core_state ==  program_transfer_state ) 
	begin
		if (~(|dma_count)) 
	    	write_status <= 1'b1;
		else
		write_status <= write_status;
	end	

       	else if(core_state ==  program_write_state ) 
	begin
        	if (dma_ready) 
		begin
            		if (dma_resp)
                    	write_status <= 1'b1;
            		else 
                    	write_status <= 1'b0;
       		end 
       		else
			write_status <= write_status;
        end

	else if(core_state ==  data_transfer_state ) 
	begin
	if (~(|dma_count)) 
		begin
		if(~(|count2)) 
                	write_status <= write_status;					
            	else 
                	write_status <= 1'b1;
               	end
	else
		write_status <= write_status;	
	end
       
	else if(core_state ==  data_write_state ) 
	begin
        	if (dma_ready) 
		begin
            		if (dma_resp)
                    	write_status <= 1'b1;
            		else 
                    	write_status <= 1'b0;
        	end
        	else
			write_status <= write_status;
    	end

	else if(core_state ==  flash_loading1 ) 
	begin
		write_status <= 1'b1;
	end

	else if(core_state ==  program_flash_transfer ) 
	begin
		if (~(|dma_count)) 
			write_status <= 1'b1;
         	else
			write_status <= write_status;
    	end

  	else if(core_state ==  program_flash_write ) 
	begin
        	if (dma_ready) 
		begin
            		if (dma_resp)	// high means transfer is not normal
                    		write_status <= 1'b1;
            		else if(~(flash_busy))
                    		write_status <= 1'b0;
			else
				write_status <= write_status;
        	end
        	else 
			write_status <= write_status;
    	end

	else if(core_state ==  data_flash_transfer ) 
	begin
      		if (~(|dma_count)) 
		begin
			if(~(|count2)) 
                    		write_status <= write_status;		
                	else 
			begin
                    		write_status <= 1'b1;
                        end
        	end
		else
			write_status <= write_status;
        end

	else if(core_state ==  data_flash_write ) 
	begin
        	if (dma_ready) 
		begin
            		if (dma_resp)
                    	write_status <= 1'b1;
            		else 
                    	write_status <= 1'b0;
        	end
        	else 
			write_status <= write_status;
    	end

	else 
	begin
		write_status <= write_status;
       	end



endmodule
