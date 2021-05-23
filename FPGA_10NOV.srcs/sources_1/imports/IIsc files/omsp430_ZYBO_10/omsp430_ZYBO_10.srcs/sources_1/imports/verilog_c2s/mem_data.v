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
// *File Name: dmem.v
// 
// *Module Description:
//                      Scalable dmem model
//
// *Author(s):
//              - Olivier Girard,    olgirard@gmail.com
//
//----------------------------------------------------------------------------
// $Rev: 103 $
// $LastChangedBy: olivier.girard $
// $LastChangedDate: 2011-03-05 15:44:48 +0100 (Sat, 05 Mar 2011) $
//----------------------------------------------------------------------------



module dmem (

// OUTPUTs
    dmem_dout,                      // dmem data output

// INPUTs
    dmem_addr,                      // dmem address
    dmem_cen,                       // dmem chip enable (low active)
    dmem_clk,                       // dmem clock
    dmem_din,                       // dmem data input
    dmem_wen                        // dmem write enable (low active)
);

// PARAMETERs
//============
parameter ADDR_MSB   =  9;         // MSB of the address bus
parameter MEM_SIZE   =  2048;       // Memory size in bytes

// OUTPUTs
//============
output    [35:0] dmem_dout;       // dmem data output

// INPUTs
//============
input [ADDR_MSB:0] dmem_addr;       // dmem address
input              dmem_cen;        // dmem chip enable 
input              dmem_clk;        // dmem clock
input       [35:0] dmem_din;        // dmem data input
input              dmem_wen;        // dmem write enable 


// dmem
//============

reg         [35:0] mem [0:(MEM_SIZE/2)-1];
reg   [ADDR_MSB:0] dmem_addr_reg;

wire        [35:0] mem_val = mem[dmem_addr];
   
  
always @(posedge dmem_clk)
  if (~dmem_cen & dmem_addr<(MEM_SIZE/2))
    begin
    if      (dmem_wen==1'b0) mem[dmem_addr]<=dmem_din;
    //  else if (dmem_wen==1'b1) dmem_dout<=mem[dmem_addr];


  /*  if      (dmem_wen==2'b00) mem[dmem_addr] <= dmem_din;
      else if (dmem_wen==2'b01) mem[dmem_addr] <= {dmem_din[15:8], mem_val[7:0]};
      else if (dmem_wen==2'b10) mem[dmem_addr] <= {mem_val[15:8], dmem_din[7:0]};*/
      dmem_addr_reg <= dmem_addr;
    end

assign dmem_dout = mem[dmem_addr_reg];
     
  


endmodule // dmem
