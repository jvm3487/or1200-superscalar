//////////////////////////////////////////////////////////////////////
////                                                              ////
////  Generic Single-Port Synchronous RAM                         ////
////                                                              ////
////  This file is part of memory library available from          ////
////  http://www.opencores.org/cvsweb.shtml/generic_memories/     ////
////                                                              ////
////  Description                                                 ////
////  This block is a wrapper with common single-port             ////
////  synchronous memory interface for different                  ////
////  types of ASIC and FPGA RAMs. Beside universal memory        ////
////  interface it also provides behavioral model of generic      ////
////  single-port synchronous RAM.                                ////
////  It should be used in all OPENCORES designs that want to be  ////
////  portable accross different target technologies and          ////
////  independent of target memory.                               ////
////                                                              ////
////  Author(s):                                                  ////
////      - Michael Unneback, unneback@opencores.org              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: or1200_dpram_32x32.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// New 
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_spram_modified64
  (
`ifdef OR1200_BIST
   // RAM BIST
   mbist_si_i, mbist_so_o, mbist_ctrl_i,
`endif
   // Generic synchronous single-port RAM interface
   clk, ce, we, addr, di, doq
   );
   
   //
   // Default address and data buses width
   //
   parameter aw = 10;
   parameter dw = 512;
   
`ifdef OR1200_BIST
   //
   // RAM BIST
   //
   input mbist_si_i;
   input [`OR1200_MBIST_CTRL_WIDTH - 1:0] mbist_ctrl_i;
   output 				  mbist_so_o;
`endif
   
   //
   // Generic synchronous single-port RAM interface
   //
   input 				  clk;	// Clock
   input 				  ce;	// Chip enable input
   input 				  we;	// Write enable input
   //input 				  oe;	// Output enable input
   input [aw-1:0] 			  addr;	// address bus inputs
   input [(dw/16)-1:0] 			  di;	// input data bus - modified to 31 because size in size is now different than data size
   output [(dw/8)-1:0] 			  doq;	// output data bus - changed to dw from dw/2 after test
   
   //
   // Internal wires and registers
   //

   //
   // Generic single-port synchronous RAM model
   //
   
   //
   // Generic RAM's registers and wires
   //
`ifdef OR1200_GENERIC   
   reg [dw-1:0] 			  mem [(1<<(aw-4))-1:0] /*synthesis syn_ramstyle = "no_rw_check"*/; // modified because cache is twice as wide so one less bit to index
`else
   reg [dw-1:0] 			  mem [(1<<(aw-4))-1:0];
`endif
   reg [aw-1:0] 			  addr_reg;		// RAM address register
   reg [(dw/8)-1:0] 			  doq_intermediate; //changed to dw from dw/2 after test
   
   
   //
   // Data output drivers
   //
   //assign doq = (oe) ? mem[addr_reg] : {dw{1'b0}};
   
   // The following logic has been modified to make insn cache 64 bits wide
   // It was then modified to make it 128 bits wide
   assign doq = doq_intermediate; 
   
   
   always @(*) begin
      case ({addr_reg[3], addr_reg[2], addr_reg[1], addr_reg[0]})
	{4'b0000}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][(dw/8)-1:0];
	{4'b0001}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((3*dw)/16)-1:dw/16];
	{4'b0010}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((4*dw)/16)-1:(2*dw)/16];
	{4'b0011}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((5*dw)/16)-1:(3*dw)/16];
	{4'b0100}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((6*dw)/16)-1:(4*dw)/16];
	{4'b0101}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((7*dw)/16)-1:(5*dw)/16];
	{4'b0110}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((8*dw)/16)-1:(6*dw)/16];
	{4'b0111}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((9*dw)/16)-1:(7*dw)/16];
	{4'b1000}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((10*dw)/16)-1:(8*dw)/16];
	{4'b1001}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((11*dw)/16)-1:(9*dw)/16];
	{4'b1010}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((12*dw)/16)-1:(10*dw)/16];
	{4'b1011}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((13*dw)/16)-1:(11*dw)/16];
	{4'b1100}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((14*dw)/16)-1:(12*dw)/16];
	{4'b1101}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((15*dw)/16)-1:(13*dw)/16];
	{4'b1110}:
	  doq_intermediate <= mem[addr_reg[aw-1:4]][((16*dw)/16)-1:(14*dw)/16];
	{4'b1111}: begin
	   doq_intermediate[(dw/8)-1:dw/16] <= {(dw/16){1'b0}};
	   doq_intermediate[(dw/16)-1:0] <= mem[addr_reg[aw-1:4]][dw-1:(15*dw)/16];
	end
/*if (addr_reg[0] == 1'b1) begin //fetched in the middle of cache block
	 doq_intermediate[(dw/2)-1:0] = mem[addr_reg[aw-1:1]][(dw-1):(dw/2)];
	 doq_intermediate[dw-1:dw/2] = {(dw/2){1'b0}}; //upper half is all zeros
      end
      else begin
	 doq_intermediate = mem[addr_reg[aw-1:1]];
      end*/
      endcase // case ({addr_reg[1], addr_reg[0]})
   end
   //
   // RAM read address register
   //
   always @(posedge clk)
     if (ce) begin
       addr_reg <=  addr;
     end 
   //
   // RAM write
   //
   always @(posedge clk)
     if (we && ce) begin
	case({addr[3], addr[2], addr[1], addr[0]})
	  {4'b0000}: begin
	     mem[addr[aw-1:4]][(dw/16)-1:0] <= di;
	  end
	  {4'b0001}: begin
	     mem[addr[aw-1:4]][((2*dw)/16)-1:(dw/16)] <= di;
	  end
	  {4'b0010}: begin
	     mem[addr[aw-1:4]][((3*dw)/16)-1:((2*dw)/16)] <= di;
	  end
	  {4'b0011}: begin
	     mem[addr[aw-1:4]][((4*dw)/16)-1:((3*dw)/16)] <= di;
	  end
	  {4'b0100}: begin
	     mem[addr[aw-1:4]][((5*dw)/16)-1:((4*dw)/16)] <= di;
	  end
	  {4'b0101}: begin
	     mem[addr[aw-1:4]][((6*dw)/16)-1:((5*dw)/16)] <= di;
	  end
	  {4'b0110}: begin
	     mem[addr[aw-1:4]][((7*dw)/16)-1:((6*dw)/16)] <= di;
	  end
	  {4'b0111}: begin
	     mem[addr[aw-1:4]][((8*dw)/16)-1:((7*dw)/16)] <= di;
	  end
	  {4'b1000}: begin
	     mem[addr[aw-1:4]][((9*dw)/16)-1:((8*dw)/16)] <= di;
	  end
	  {4'b1001}: begin
	     mem[addr[aw-1:4]][((10*dw)/16)-1:((9*dw)/16)] <= di;
	  end
	  {4'b1010}: begin
	     mem[addr[aw-1:4]][((11*dw)/16)-1:((10*dw)/16)] <= di;
	  end
	  {4'b1011}: begin
	     mem[addr[aw-1:4]][((12*dw)/16)-1:((11*dw)/16)] <= di;
	  end
	  {4'b1100}: begin
	     mem[addr[aw-1:4]][((13*dw)/16)-1:((12*dw)/16)] <= di;
	  end
	  {4'b1101}: begin
	     mem[addr[aw-1:4]][((14*dw)/16)-1:((13*dw)/16)] <= di;
	  end
	  {4'b1110}: begin
	     mem[addr[aw-1:4]][((15*dw)/16)-1:((14*dw)/16)] <= di;
	  end
	  {4'b1111}: begin
	     mem[addr[aw-1:4]][dw-1:((15*dw)/16)] <= di;
	  end
	  
	endcase
     end
	
endmodule // or1200_spram
