//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Instruction decode                                 ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Majority of instruction decoding is performed here.         ////
////                                                              ////
////  To Do:                                                      ////
////   - make it smaller and faster                               ////
////                                                              ////
////  Author(s):                                                  ////
////      - Damjan Lampret, lampret@opencores.org                 ////
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


//This file was create to try and break the control down into better sizes that could be used for two insns

`include "timescale.v"
`include "or1200_defines.v"

module or1200_ctrl_if_decode
  (
   // Clock and reset
   clk, rst, if_insn, id_freeze, id_flushpipe, rf_addr1, rf_addr2, rf_rd1, rf_rd2, /*sel_imm, id_branch_op*/
   
   );

   input					clk;
   input					rst;
   input [31:0] 				if_insn;
   input 					id_freeze;
   input 					id_flushpipe;		
   output [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addr1; //changed to 1 and 2 because been using a, b, c, and d to refer to particular parts of the two wide insns
   output [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addr2;
   output 					rf_rd1;
   output 					rf_rd2;
   wire 					if_maci_op;
   //output 					sel_imm;
   //output [`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
   //reg 						sel_imm;
   //reg	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
   

//
// Register file read addresses
//
   assign rf_addr1 = if_insn[20:16];
   assign rf_addr2 = if_insn[15:11];
   assign rf_rd1 = if_insn[31] || if_maci_op;
   assign rf_rd2 = if_insn[30];

   
   //
   // l.maci in IF stage
   //
   //MAC is implemented by default
`ifdef OR1200_MAC_IMPLEMENTED
   assign if_maci_op = (if_insn[31:26] == `OR1200_OR32_MACI);
`else
   assign if_maci_op = 1'b0;
`endif

endmodule
