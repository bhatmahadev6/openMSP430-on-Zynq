`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: IISc
// Engineer: PRADEEP GUPTA
// 
// Create Date: 11/27/2016 02:31:29 PM
// Design Name: SPI_interface 
// Module Name: interface_fsm
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


module spi_intermediate(clk_spi, rst, flash_cen, flash_wen, FMC,
                     FMA_high,  FMA_low, data_in, flash_data_out, done, busy, 
                     wr_tx_fifo, rd_tx_fifo,wr_rx_fifo, rd_rx_fifo,
                     flash_en,  flash_clk, flash_data_in, state, count1, fifo_addr,enable,rddata,data_width,busy_normal,flash_addr
 

    );
    
    input clk_spi; //25MHz clock
    input rst;
    input flash_cen;
    input flash_wen;
    input [15:0] FMC;
    input [15:0] FMA_high;
    input [15:0] FMA_low;
    input [15:0] data_in;    
    input flash_data_out;    
    input done; 
    input [15:0] flash_addr;   
    
    output reg busy;
    output reg busy_normal;
    output reg wr_tx_fifo;
    output reg rd_tx_fifo;
    output reg wr_rx_fifo;
    output reg rd_rx_fifo;
    output reg flash_en;
    output reg flash_clk;
    output reg flash_data_in;
    output reg[3:0] state;
    output reg[7:0] count1;
    reg [7:0] count1f;
    //output reg[8:0] count1;
    output reg[7:0] fifo_addr;
    output reg[15:0] rddata;    
    parameter idle = 4'b0000;
    parameter get_ready = 4'b0001;  
    parameter write_fifo = 4'b0010;
    parameter send_opcode_addr_high = 4'b0011;
    parameter send_addr_low = 4'b0100;
    parameter transfer_data = 4'b0101;
    parameter read_data = 4'b0110;
    parameter output_data = 4'b0111;
    
    reg[3:0] next_state;
    reg[3:0] count2;
    reg[3:0] count2f;
    reg[15:0] read_reg[1:0];
    reg[15:0] read_reg1;//sooshini for trial
    reg[15:0] write_reg[1:0];
    reg read_reg_toggle;
    reg toggle;
    reg togglef;
    
    //wire[6:0] data_width;
    //wire enable;
    wire read_enable;
    wire send_data;
    wire write_enable;
    wire write_enable_fsm;
   //reg  clk_spi;
   input enable;
    


 /*  always @(posedge clk_spi50)
      if (~rst) 
		clk_spi <= 1'b1;
      else         
		clk_spi <= ~(clk_spi);*/

    
    //assign data_width = FMA_high[15:8];
    input [6:0] data_width;
    
    //assign enable = FMC[13];
    assign write_enable_fsm = (FMC[13] && FMC[8]);
    assign write_enable =enable;//sooshini
    //assign read_enable = FMC[5] && FMC[1];
    assign read_enable = FMC[9] && FMC[1];
    assign send_data = (flash_cen && ~(flash_wen)) && (FMC[13] && FMC[9]);
    
    
    always @(posedge clk_spi )
    begin
		if(~rst) 
			state<=idle;
		else
			state<=next_state;
    end
    
    always @(posedge clk_spi )
    begin
		if(~rst) begin
			//fifo_addr <= 8'b0; Sooshini		
			count2 <= 4'b1111;
                        if(data_width == 7'h3F) count1 <= {1'b0,data_width}; //sooshini
                        else count1 <= data_width -7'h01;
                        //count1 <= data_width; pradeep
			toggle <= 1'b0;
			write_reg[0] <= 16'b0; 
			write_reg[1] <= 16'b0; 	
			read_reg_toggle <= 1'b0;
			read_reg[0] <= 16'b0;
			read_reg[1] <= 16'b0;
                        wr_rx_fifo <= 1'b0;
			
		end
		else if(state == idle )	begin
			//fifo_addr <= fifo_addr + 1;Sooshini	
			//fifo_addr <= 8'b0;Sooshini			
			count2 <= 4'b1111;
			if(data_width == 7'h3F) count1 <= {1'b0,data_width}; //sooshini
                        else count1 <= data_width -7'h01;
                        //count1 <= data_width; pradeep
			toggle <= 1'b0;
			write_reg[0] <= 16'b0; 
			write_reg[1] <= 16'b0; 	
			read_reg_toggle <= 1'b0;
			read_reg[0] <= 16'b0;
			read_reg[1] <= 16'b0;
                        wr_rx_fifo <= 1'b0;
		end
		else if(state == write_fifo )	begin
			//fifo_addr <= fifo_addr + 1;Sooshini	
			count1 <= count1 - 1;
                        wr_rx_fifo <= 1'b0;
		end
		else if(state == get_ready )	begin
			//fifo_addr <= 8'b0;Sooshini
                         wr_rx_fifo <= 1'b0;	
			if(data_width == 7'h3F) count1 <= {1'b0,data_width}; //sooshini
                        else count1 <= data_width -7'h01;
                        //count1 <= data_width; pradeep
			write_reg[0] <= {{FMC[7:0]} , {FMA_high[7:0]} }; 
			write_reg[1] <= FMA_low; 
		end
		else if(state == send_opcode_addr_high ) begin
                        wr_rx_fifo <= 1'b0;
                        if(flash_addr ==  15'h00FC) count1<= 8'h7F;//sooshini
                        write_reg[0] <= { {write_reg[0][14:0]} , {1'b0} };
			if(count2)		count2 <= count2 - 1;
			else 			count2 <= 8'h0F;			
						
		end
		else if(state == send_addr_low )	begin
                        wr_rx_fifo <= 1'b0;
			write_reg[1] <= { {write_reg[1][14:0]} , {1'b0} };
			write_reg[0] <= data_in;
			if(count2)		count2 <= count2 - 1;
			else 			count2 <= 8'h0F;
		end
		else if(state == transfer_data )	
                  begin	
                        wr_rx_fifo <= 1'b0;  			
			if (toggle == 1'b0)
                           begin
				if(count2)
                                  begin 
					write_reg[0] <= { {write_reg[0][14:0]} , {1'b0} };
					write_reg[1] <= data_in;
					count2 <= count2 - 1; 
				  end
				else 
                                  begin   
					write_reg[0] <= { {write_reg[0][14:0]} , {1'b0} };
					count1 <= count1 - 1; 
					toggle <= ~(toggle);
					count2 <= 8'h0F;	
					//fifo_addr <= //fifo_addr + 1;	Sooshini					
				  end
			 end
			else 
                             begin
				if(count2 )
                                  begin 
					write_reg[1] <= { {write_reg[1][14:0]} , {1'b0} };
					write_reg[0] <= data_in;
					count2 <= count2 - 1; 
				end
				else 
                                  begin    
					write_reg[1] <= { {write_reg[1][14:0]} , {1'b0} };
					count1 <= count1 - 1; 
					toggle <= ~(toggle);
					count2 <= 8'h0F;			
					//fifo_addr <= //fifo_addr + 1;	Sooshini					
				end
			end
		end
		else if(state == read_data )	begin
                        
			toggle <= 1'b0;
			write_reg[0] <= 16'b0; 
			write_reg[1] <= 16'b0; 
			read_reg[read_reg_toggle] <= { {read_reg[read_reg_toggle][14:0]} , {flash_data_out}};
                        read_reg1 <= { {read_reg1[14:0]} , {flash_data_out}};//sooshini
                         
                        if (!count2) begin
				count1 <= count1 - 1;
				read_reg_toggle <= !(read_reg_toggle);
                                   
			       
                                               
				
			end
                        if((data_width == 7'h08 )& (count1f == 8'h01) & (count2f ==8'h0C) )wr_rx_fifo <= 1'b0;//sooshini 1 july
                        else if(!count1f ) wr_rx_fifo <= 1'b0;
                        else if(count2f == 4'b1110)
                        //if(!count2)
                         begin
                        rddata <= read_reg[!read_reg_toggle];
                        
                     
                             wr_rx_fifo <= 1'b1;
                         end
                        
                        count2<= count2-8'h01;//sooshini added
			
		end
		else if(state == output_data )
              	begin
			read_reg_toggle <= 1'b0;
			read_reg[0] <= 16'b0;
			read_reg[1] <= 16'b0;
                        count1 <=  {1'b0,data_width} ;
                        wr_rx_fifo <= 1'b0;
			//count1 <= data_width -7'h01 ;
			//count2 <= 4'b1111; 	
			//if(send_data) begin
				///fifo_addr <= fifo_addr + 1;	
			//end				
		end
		
		else
                begin
                        count2 <= 4'b1111;
			if(data_width == 7'h3F) count1 <= {1'b0,data_width}; //sooshini
                        else count1 <= data_width -7'h01;
                        //count1 <= data_width; pradeep
			toggle <= 1'b0;
			write_reg[0] <= 16'b0; 
			write_reg[1] <= 16'b0; 	
			read_reg_toggle <= 1'b0;
			read_reg[0] <= 16'b0;
			read_reg[1] <= 16'b0;
                        wr_rx_fifo <= 1'b0;
                end

	end





  
  

  
// sooshini for FIFO synchronisation
	always @(negedge clk_spi )
          begin
		if(~rst) begin
			fifo_addr <= 8'b0;		
			count2f <= 4'b1111;
			
			togglef <= 1'b0;
			
		end
		else if(state == idle )	begin
			//fifo_addr <= fifo_addr + 1;Sooshini	
			fifo_addr <= 8'b0;		
			count2f <= 4'b1111;
			count1f <= {1'b0,data_width};
			togglef <= 1'b0;
			
		end
		else if(state == write_fifo )	begin
                         if((count1 == 15'h7E) & (flash_addr >= 15'h00FC) & (flash_addr <= 15'h09FE)) fifo_addr <= 0;
                         else if((count1 == 15'h07) & (flash_addr >= 15'h09F0)  & (flash_addr <= 15'h09FE) )  fifo_addr <= 0;
                         else 
                         begin
                          fifo_addr <= fifo_addr + 1;
			  count1f <= count1f - 1;
                        end
                        
		end
		else if(state == get_ready )	begin
			fifo_addr <= 8'b0;
			count1f <= data_width +1 ;
                       
			
		end
		else if(state == send_opcode_addr_high ) begin
                      
			if(count2f)		count2f <= count2f - 1;
			else 			count2f <= 8'h0F;			
						
		end
		else if(state == send_addr_low )	begin
			//fifo_addr <= 8'b1;
                         
			 if( (count2 == 4'b0000)  & read_enable)
                         begin
                          fifo_addr<=0;   
                         end
                         else fifo_addr<=1;  
			if(count2f)		count2f <= count2f - 1;
			else 			count2f <= 8'h0F;
		end
		else if(state == transfer_data )	
                  begin					
			if (togglef == 1'b0)
                           begin
				if(count2f )
                                  begin 
					
					count2f <= count2f - 1; 
				  end
				else 
                                  begin   
					
					count1f <= count1f - 1; 
					togglef <= ~(togglef);
					count2f <= 8'h0F;	
					fifo_addr <= fifo_addr + 1;					
				  end
			 end
			else 
                             begin
				if(count2f )
                                  begin 
					
					count2f <= count2f - 1; 
				end
				else 
                                  begin    
					
					count1f <= count1f - 1; 
					togglef <= ~(togglef);
					count2f<= 8'h0F;			
					fifo_addr <= fifo_addr + 1;					
				end
			end
		end
		else if(state == read_data )	
                  begin
			togglef <= 1'b0;
			count2f <= count2f - 1;
			//if( !count1f ) fifo_addr<=8'b0 ;
                         
                        //if ((data_width == 7'h08)& (count1f==8'h09))
                        // begin
                         //fifo_addr <= 8'h00;
                         // count1f <= count1f - 1;
                         //end
			//else
                        if((data_width == 7'h08) & (count1f == 8'h01) & (count2 == 4'b1101) )  fifo_addr <=8'h09;//sooshini 30 june
                        else if (count2f == 4'b1100) 
                         begin
                        //if(count2 == 4'b1111)
                                 if(count1f <=8'h7F)
                                  begin
                                  	
                                    if((data_width == 7'h08) & (count1f == 8'h09))  fifo_addr <=8'h00;
                                    
                                    else    fifo_addr <= fifo_addr + 1;
                                  end
				   count1f <= count1f - 1;
                         end
                        else if((!count1f )& (count2f == 4'b0000)) count1f<={1'b0,data_width};
                        if (!(count1f))
                         begin
                          fifo_addr<=0;
                           //count1f <= data_width ;
			    //count2f <= 4'b1111;
                          end 	  
		end
		else if(state == output_data )	
                   begin
			
			
			if(send_data)
                          begin
                                if((count1f == 8'hFF)|(!count1f)) count1f <={1'b0,data_width};
                                else 
                                 begin
                                 //if((data_width==7'h08) &(count1f==8'h01)) fifo_addr<=0;
				//else 
                               fifo_addr <= fifo_addr + 1;
                                
				count1f <= count1f - 1;//sooshini
                                end
                        end
 
 			
		end

		 else

                begin
                  fifo_addr <= 8'b0;		
		 count2f <= 4'b1111;
		 count1f <= {1'b0,data_width};
		togglef <= 1'b0;
                end
		

	end
  

	
    
    always@(*)
    begin
    
		if(~rst)
		begin
			next_state<= idle;
			busy <= 1'b0;
                         busy_normal<= 1'b1;
			wr_tx_fifo <= 1'b0;
			rd_tx_fifo <= 1'b0;
			//wr_rx_fifo <= 1'b0;
			rd_rx_fifo <= 1'b0;
			flash_en <= 1'b0;
			flash_clk <= 1'b0;
			flash_data_in <= 1'b0;	
		end
		else    begin
			case(state)
			
				idle:
				begin
					busy <= 1'b0;
					busy_normal<= 1'b1;
					rd_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1'b0;
					flash_clk <= clk_spi;
					flash_data_in <= 1'b0;				
					if ( enable ) begin //Sooshini
						//if (write_enable) 
                                                 if (write_enable_fsm)//flash Program mode
								next_state <= write_fifo;
						else if(read_enable)
                                                 //else 
                                                   next_state <= get_ready;
                                                
                                                   //next_state <= get_ready;
					end
                                         
					else
						next_state<= idle;// ???
                                        if(next_state == write_fifo)
                                           wr_tx_fifo <= 1'b1;
                                        else 
                                           wr_tx_fifo <= 1'b1;
                                         
					
				end

				write_fifo:
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b1;
					rd_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1'b0;
					flash_clk <= clk_spi;
					flash_data_in <= 1'b0;
					
					if( count1 == 8'b0 )
						next_state <= get_ready;
					else
						next_state <= write_fifo;						
				end
				
				get_ready:
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					rd_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1'b0;
					flash_clk <= clk_spi;
					flash_data_in <= 1'b0;	
					next_state <= send_opcode_addr_high;
							
				end
				
				send_opcode_addr_high:
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1'b1;
					flash_clk <= clk_spi;
					flash_data_in <= write_reg[0][15];
                                        
					
					if( count2 == 4'b0000 ) begin
						next_state <= send_addr_low;
						rd_tx_fifo <= 1'b1;
					end
					else begin
						next_state <= send_opcode_addr_high;
						rd_tx_fifo <= 1'b0;
					end       
				end
				
				send_addr_low:     
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1;
					flash_clk <= clk_spi;
					flash_data_in <= write_reg[1][15];
					
					if( (count2 == 4'b0000)  & write_enable_fsm) begin
                                          //if( (count2 == 4'b0001)  & write_enable_fsm) begin
						next_state <= transfer_data;
						rd_tx_fifo <= 1'b1;
					end
					else if( (count2 == 4'b0000)  & read_enable) begin
						next_state <= read_data;
						rd_tx_fifo <= 1'b0;
					end
					else if (count2 == 4'b0000) begin
						next_state <= idle;
						rd_tx_fifo <= 1'b0;
					end
					else begin
						next_state <= send_addr_low;  
						rd_tx_fifo <= 1'b0;
					end
				end
				
				transfer_data:
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 1;
					flash_clk <= clk_spi;
					
					if (toggle == 1'b0) 
						flash_data_in <= write_reg[0][15];
					else
						flash_data_in <= write_reg[1][15];						
					if( ! count2 )  
						rd_tx_fifo <= 1'b1;
					else    
					begin	rd_tx_fifo <= 1'b0;
                                                
                                         end
						
					if ((count1 == 8'b0) & (count2 == 4'b0))
						next_state <= idle;
					else
						next_state <= transfer_data;
						
				end
				
				read_data:
				begin
					busy <= 1'b1;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					rd_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
                                        
                                         flash_en <= 1;
					flash_clk <= clk_spi;
					flash_data_in <= 1'b0;
                                        
                                       				
					/*if(data_width ==7'h8)
                                        begin
					 if((!count1 ) & (!count2f)) 
						 next_state <= output_data;
					 else 
						 next_state <= read_data; 
					end
                                        else
                                        begin
                                        */
					if((!count1f )& (count2f == 4'b0000)) 
						 next_state <= output_data;
					 else
						 next_state <= read_data;  
                                        //end        
				end
				
				output_data:
				begin
				busy <= 1'b0;
                                 busy_normal<= 1'b0;
				wr_tx_fifo <= 1'b0;
				rd_tx_fifo <= 1'b0;
				//wr_rx_fifo <= 1'b0;
				flash_en <= 0;
				flash_clk <= 0;
				flash_data_in <= 1'b0;					
				if(send_data) begin
						rd_rx_fifo <= 1'b1;
				end
				else    rd_rx_fifo <= 1'b0;
                                if((data_width == 7'h08) & (count1 >8'h00))next_state <= output_data; //sooshini june 29
                                 else if((data_width == 7'h08) & (count1f ==8'h01) &( count2f== 4'hD))next_state <= idle;  //sooshini june 29				
				else if(done)    next_state <= idle;
                                 
				else if( (!count1f ) )
                                begin
                                  if( !enable ) next_state <=idle ; //sooshini 
                                  else
                                  next_state <= read_data; //sooshini 
                                  
                                end
                                else  next_state <= output_data;  

				end
				   
				
				default:
				begin
					/*busy <= 1'b0;
                                        busy_normal<= 1'b1;
					wr_tx_fifo <= 1'b0;
					rd_tx_fifo <= 1'b0;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= 1'b0;
					flash_en <= 0;
					flash_clk <= 0;           
					flash_data_in <= 1'b0;
					next_state <= idle;*/
		
					busy <= busy;
                                        busy_normal<= busy_normal;
					wr_tx_fifo <= wr_tx_fifo;
					rd_tx_fifo <= rd_tx_fifo;
					//wr_rx_fifo <= 1'b0;
					rd_rx_fifo <= rd_rx_fifo;
					flash_en <= flash_en;
					flash_clk <= flash_clk;           
					flash_data_in <= flash_data_in;
					next_state <= idle;
			
				end
			endcase
		end
    end
 
 


   
endmodule

