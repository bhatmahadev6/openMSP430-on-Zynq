//module mux(mclk, reset_n,dbg_rxd_memchk,dbg_rxd_boot,memory_check_done, dbg_uart_rxd);
module mux(dbg_rxd_memchk,dbg_rxd_boot,memory_check_done, dbg_uart_rxd);

input dbg_rxd_memchk,dbg_rxd_boot,memory_check_done;
output reg dbg_uart_rxd;

//always @(posedge mclk or negedge reset_n)
always @(memory_check_done or dbg_rxd_memchk or dbg_rxd_boot)
	begin
		if (memory_check_done ==1'b0)
		dbg_uart_rxd = dbg_rxd_memchk;
		else //(memory_check_done ==1'b1)
		dbg_uart_rxd = dbg_rxd_boot;
	end
	
endmodule
