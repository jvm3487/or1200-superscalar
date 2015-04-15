//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's instruction fetch                                  ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  PC, instruction fetch, interface to IC.                     ////
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
//
// $Log: or1200_if.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update: 
// Structure reordered and bugs fixed. 

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_if(
	// Clock and reset
	clk, rst,

	// External i/f to IC
	icpu_dat_i, icpu_ack_i, icpu_err_i, icpu_adr_i, icpu_tag_i,

	// Internal i/f
	if_freeze, if_insn, if_pc, if_flushpipe, saving_if_insn, 
	if_stall, no_more_dslot, genpc_refetch, rfe,
	except_itlbmiss, except_immufault, except_ibuserr, half_insn_done_i, dependency_hazard_stall
);

//
// I/O
//

//
// Clock and reset
//
input				clk;
input				rst;

//
// External i/f to IC
//
input	[63:0]			icpu_dat_i; //modified for two insns
input				icpu_ack_i;
input				icpu_err_i;
input	[31:0]			icpu_adr_i;
input	[3:0]			icpu_tag_i;

//
// Internal i/f
//
input				if_freeze;
output	[63:0]			if_insn; //double width
output	[31:0]			if_pc;
input				if_flushpipe;
output				saving_if_insn;
output				if_stall;
input				no_more_dslot;
output				genpc_refetch;
input				rfe;
output				except_itlbmiss;
output				except_immufault;
output				except_ibuserr;
input    			half_insn_done_i;
input 	                	dependency_hazard_stall;
 			
//
// Internal wires and regs
//
wire			save_insn;
wire			if_bypass;
reg			if_bypass_reg;
reg	[63:0]		insn_saved; //modified for two insns
reg	[31:0]		addr_saved;
reg	[2:0]		err_saved;
reg			saved;
/*reg 			half_insn_done; this signal was used when parse happened during fetch cycle
reg [31:0] 		next_insn;
reg [31:0] 		next_addr;*/
	
//added the !half_insn_done so that another instruction would not be saved if it was in the middle of executing an instruction in a later stage
assign save_insn = (icpu_ack_i | icpu_err_i) & if_freeze & !saved /*& !half_insn_done_i*/; 
assign saving_if_insn = !if_flushpipe & save_insn;

//
// IF bypass 
//
assign if_bypass = icpu_adr_i[0] ? 1'b0 : if_bypass_reg | if_flushpipe;
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		if_bypass_reg <=  1'b0;
	else
		if_bypass_reg <=  if_bypass;

//
// IF stage insn
//
//half_insn_done leading to next insn was added to this logic - removed after testing in order to fetch a full insn
//added back to keep fetching the same insn if half of an instruction is completed
assign if_insn = no_more_dslot | rfe | if_bypass ? {2{`OR1200_OR32_NOP, 26'h041_0000}} /*: half_insn_done_i ? next_insn*/ : saved ? insn_saved : icpu_ack_i ? icpu_dat_i : {2{`OR1200_OR32_NOP, 26'h061_0000}}; //161 is used for exceptions 
//the following is just used for exceptions
//it has been modified to take into account the possibility of two insns
assign if_pc = dependency_hazard_stall ? {icpu_adr_i[31:2], 2'h0} : saved ? addr_saved : {icpu_adr_i[31:2], 2'h0};
//it appears if_stall seems to almost mirror if_freeze
//primary difference is that if if_freeze is high and an instruction is being saved, than if_stall will be low
//if_freeze is defined in the freeze.v file
//in the cases where they don't mirror each other it is due to the fact that the signal waiting_on = "11" which was latched when wait_on="11" and ex_freeze was low
//wait_on is defined in the ctrl logic based on what the instruction in the decode stage is - it looks like during instruction 0x30 - move to special purpose register this occurs
//however if_freeze is dependent on if_stall and will also be high if if_stall is high and lsu_unstall is low
//however if_freeze can go high for other reasons including a lsu_stall
//half_ins_done was added to this logic to stall the fetching of instructions
assign if_stall = !icpu_err_i & ((!icpu_ack_i & !saved) /*| half_insn_done_i*/);
//half_insn_done no longer needed after 2 wide if_insn pushed through
assign genpc_refetch = (saved /*| half_insn_done_i*/) & icpu_ack_i; //half_ins_done added to this logic to show not ready for next insn
//I haven't found a reason to change these for two insns yet (three exception lines below) since the two insns will never straddle a page
assign except_itlbmiss = no_more_dslot ? 1'b0 : saved ? err_saved[0] : icpu_err_i & (icpu_tag_i == `OR1200_ITAG_TE);
assign except_immufault = no_more_dslot ? 1'b0 : saved ? err_saved[1] : icpu_err_i & (icpu_tag_i == `OR1200_ITAG_PE);
assign except_ibuserr = no_more_dslot ? 1'b0 : saved ? err_saved[2] : icpu_err_i & (icpu_tag_i == `OR1200_ITAG_BE);
//Next Insn for two Insns - allows the second half of an insn to be sent through - lefgacy from testing 
/*always @(posedge clk or `OR1200_RST_EVENT rst) begin
  if (rst == `OR1200_RST_VALUE) begin
     half_insn_done <= 1'b0;
     next_insn <= {`OR1200_OR32_NOP, 26'h141_0000};
     next_addr <= 32'h4;  //only needed for exceptions
  end
  else if (!no_more_dslot & !rfe & !if_bypass) begin
     if (half_insn_done)
       half_insn_done <= 1'b0;
     else if (saved & !if_freeze) begin //if_freeze was added because the instruction did not get latched into id_insn if it is high
	//checks to see if sending a nop (type 141) that was added by the addition of two insns so as not to increase the PC
	if (insn_saved[63:32] != {`OR1200_OR32_NOP, 26'h141_0000}) begin
	   half_insn_done <= 1'b1;
	   next_insn <= insn_saved[63:32];
	   next_addr <= addr_saved + 32'h4; //this is only needed for an exception that has to choose which insn caused it
	end
     end
     else if (icpu_ack_i & !if_freeze) begin //if_freeze was added for the same reason as above
	//same as comment above - checks nop type
	if (icpu_dat_i[63:32] != {`OR1200_OR32_NOP, 26'h141_0000}) begin
	   half_insn_done <= 1'b1;
	   next_insn <= icpu_dat_i[63:32];
	   next_addr <= {icpu_adr_i[31:2], 2'b00} + 32'h4;  //only needed for exceptions
	end
     end
  end
  //the following is needed because of the delay slot in the processor
  //there is the possibility that an instruction is fetched just prior to a branch being executed 
   //in which case only half of the instruction (the delay slot part) should be executed and the other half ignored
   //without this, the processor will stall indefinitely or have very strange behavior
   else if(no_more_dslot & half_insn_done) begin
      half_insn_done <= 1'b0;
   end
   //default value is no change
end */
   
//
// Flag for saved insn/address
//
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		saved <=  1'b0;
	else if (if_flushpipe)
		saved <=  1'b0;
	else if (save_insn)
		saved <=  1'b1;
	else if (!if_freeze)
		saved <=  1'b0;

//
// Store fetched instruction - modified to ensure two insns saved and changed the nop type to 141
//
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		insn_saved <=  {2{`OR1200_OR32_NOP, 26'h141_0000}};
	else if (if_flushpipe)
		insn_saved <=  {2{`OR1200_OR32_NOP, 26'h141_0000}};
	else if (save_insn)
		insn_saved <=  icpu_err_i ? {2{`OR1200_OR32_NOP, 26'h141_0000}} : icpu_dat_i; //modified for two insns
	else if (!if_freeze)
		insn_saved <=  {2{`OR1200_OR32_NOP, 26'h141_0000}};

//
// Store fetched instruction's address
//
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		addr_saved <=  32'h00000000;
	else if (if_flushpipe)
		addr_saved <=  32'h00000000;
	else if (save_insn)
		addr_saved <=  {icpu_adr_i[31:2], 2'b00}; 
	else if (!if_freeze)
		addr_saved <=  {icpu_adr_i[31:2], 2'b00};
   
//
// Store fetched instruction's error tags 
//
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		err_saved <=  3'b000;
	else if (if_flushpipe)
		err_saved <=  3'b000;
	else if (save_insn) begin
		err_saved[0] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_TE);
		err_saved[1] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_PE);
		err_saved[2] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_BE);
	end
	else if (!if_freeze)
		err_saved <=  3'b000;


endmodule
