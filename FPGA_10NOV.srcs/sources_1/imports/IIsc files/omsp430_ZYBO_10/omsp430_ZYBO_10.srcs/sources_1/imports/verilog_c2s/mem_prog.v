`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// Copyright (C) 2001 Authors
//
// This source file may be used and distributed without restriction provided
// that this copyright statement is not removed from the file and that any
// derivative work contains the original copyright notice and the associated
// disclaimer.
//
// This source file is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This source is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
// License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this source; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//----------------------------------------------------------------------------
// 
// *File Name: pmem.v
// 
// *Module Description:
//                      Scalable pmem model
//
// *Author(s):
//              - Olivier Girard,    olgirard@gmail.com
//
//----------------------------------------------------------------------------
// $Rev: 103 $
// $LastChangedBy: olivier.girard $
// $LastChangedDate: 2011-03-05 15:44:48 +0100 (Sat, 05 Mar 2011) $
//----------------------------------------------------------------------------



module pmem (

// OUTPUTs
    pmem_dout,                      // pmem data output

// INPUTs
    pmem_addr,                      // pmem address
    pmem_cen,                       // pmem chip enable (low active)
    pmem_clk,                       // pmem clock
    pmem_din,                       // pmem data input
    pmem_wen                        // pmem write enable (low active)
);

// PARAMETERs
//============
parameter ADDR_MSB   =  12;         // MSB of the address bus
parameter MEM_SIZE   =  16384;       // Memory size in bytes

// OUTPUTs
//============
output    [35:0] pmem_dout;       // pmem data output

// INPUTs
//============
input [ADDR_MSB:0] pmem_addr;       // pmem address
input              pmem_cen;        // pmem chip enable 
input              pmem_clk;        // pmem clock
input       [35:0] pmem_din;        // pmem data input
input              pmem_wen;        // pmem write enable 


// pmem
//============

reg         [35:0] mem [0:(MEM_SIZE/2)-1];
reg   [ADDR_MSB:0] pmem_addr_reg;

wire        [35:0] mem_val = mem[pmem_addr];
   
  
always @(posedge pmem_clk)
  if (~pmem_cen & pmem_addr<(MEM_SIZE/2))
    begin
      if      (pmem_wen==1'b0) mem[pmem_addr] <= pmem_din;
     // else if (pmem_wen==1'b0) pmem_dout<=mem[pmem_addr];


      /*else if (pmem_wen==2'b01) mem[pmem_addr] <= {pmem_din[15:8], mem_val[7:0]};
      else if (pmem_wen==2'b10) mem[pmem_addr] <= {mem_val[15:8], pmem_din[7:0]};*/
      pmem_addr_reg <= pmem_addr;
    end

assign pmem_dout = mem[pmem_addr_reg];

    


endmodule // pmem
