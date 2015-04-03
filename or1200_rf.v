//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's register file inside CPU                           ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Instantiation of register file memories                     ////
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
// $Log: or1200_rf.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update: 
// Bugs fixed, coding style changed. 
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_rf(
	// Clock and reset
	clk, rst,

	// Write i/f
	cy_we_i, cy_we_o, supv, wb_freeze, addrw, dataw, we, flushpipe,

	// Read i/f
	id_freeze, addra, addrb, addrc, addrd, dataa, datab, datac, datad, rda, rdb, rdc, rdd,

	// Debug
	spr_cs, spr_write, spr_addr, spr_dat_i, spr_dat_o, du_read,

	//Needed for test of two insn
	half_insn_done_i
);

parameter dw = `OR1200_OPERAND_WIDTH;
parameter aw = `OR1200_REGFILE_ADDR_WIDTH;

//
// I/O
//

//
// Clock and reset
//
input				clk;
input				rst;

//
// Write i/f
//
input				cy_we_i;
output				cy_we_o;
input				supv;
input				wb_freeze;
input	[aw-1:0]		addrw;
input	[dw-1:0]		dataw;
input				we;
input				flushpipe;

//
// Read i/f
//
input				id_freeze;
input	[aw-1:0]		addra;
input	[aw-1:0]		addrb;
input   [aw-1:0] 		addrc; //added for two insns
input   [aw-1:0] 		addrd; //added for two insns
output	[dw-1:0]		dataa;
output	[dw-1:0]		datab;
output	[dw-1:0]		datac;
output	[dw-1:0]		datad;
input				rda;
input				rdb;
input    			rdc; //added for two insns
input 	        		rdd; //added for two insns
   

//
// SPR access for debugging purposes
//
input				spr_cs;
input				spr_write;
input	[31:0]			spr_addr;
input	[31:0]			spr_dat_i;
output	[31:0]			spr_dat_o;
input    			du_read;
input 	         		half_insn_done_i;
   
   
//
// Internal wires and regs
//
wire	[dw-1:0]		from_rfa;
wire	[dw-1:0]		from_rfb;
wire    [dw-1:0] 		from_rfc; //added for two insns
wire    [dw-1:0] 		from_rfd; //added for two insns		
wire	[aw-1:0]		rf_addra;
wire	[aw-1:0]		rf_addrc;
wire	[aw-1:0]		rf_addrw;
wire	[dw-1:0]		rf_dataw;
wire				rf_we;
wire				spr_valid;
wire				rf_ena;
wire				rf_enb; 		
wire 			        rf_enc; //added for two insns
wire 		                rf_end; //added for two insns	
reg				rf_we_allow;
wire 				half_insn_done_next;
reg [dw-1:0]     		from_rfc_next;
reg [dw-1:0] 		        from_rfd_next;
				
   
   // Logic to restore output on RFA after debug unit has read out via SPR if.
   // Problem was that the incorrect output would be on RFA after debug unit
   // had read out  - this is bad if that output is relied upon by execute
   // stage for next instruction. We simply save the last address for rf A and
   // and re-read it whenever the SPR select goes low, so we must remember
   // the last address and generate a signal for falling edge of SPR cs.
   // -- Julius
   
   // Detect falling edge of SPR select 
   reg 				spr_du_cs;
   wire 			spr_cs_fe;
   // Track RF A's address each time it's enabled
   reg	[aw-1:0]		addra_last;

   //this is needed because half_ins_done (same thing with from_rfc_next) refers to the id stage and the other half of the insn is currently in the if stage
   /*always @(posedge clk)
     if (!id_freeze) //!id_freeze is needed in case a jump occurs in the second half of an insn so that the correct registers are selected depending on what half of the insn id_insn is in - probably can be removed after testing
       half_insn_done_next <= half_insn_done_i;*/
   //registered it in ctrl so no need to do it again
   //assign half_insn_done_next = half_insn_done_i;
     
   /*always @(posedge clk) begin
      from_rfc_next <= from_rfc;
      from_rfd_next <= from_rfd;
   end*/  
   always @(posedge clk)
     if (((rf_ena /*& !half_insn_done_next*/) | (rf_enc /*& half_insn_done_next*/)) & !(spr_cs_fe | (du_read & spr_cs)))
       addra_last <= addra;

   always @(posedge clk)
     spr_du_cs <= spr_cs & du_read;

   assign spr_cs_fe = spr_du_cs & !(spr_cs & du_read);

   
//
// SPR access is valid when spr_cs is asserted and
// SPR address matches GPR addresses
//
assign spr_valid = spr_cs & (spr_addr[10:5] == `OR1200_SPR_RF);

//
// SPR data output is always from RF A - unless you do two insns and then maybe not
//
assign spr_dat_o = /*half_insn_done_next ? from_rfc_next :*/ from_rfa;

//
// Operand A comes from RF or from saved A register - unless two insns
//
assign dataa = /*half_insn_done_next ? from_rfc_next :*/ from_rfa;

//
// Operand B comes from RF or from saved B register
//
assign datab = /*half_insn_done_next ? from_rfd_next :*/ from_rfb;

assign datac = from_rfc;   

assign datad = from_rfd;

   
//
// RF A read address is either from SPRS or normal from CPU control
//
assign rf_addra = (spr_valid & !spr_write) ? spr_addr[4:0] : 
		  spr_cs_fe ? addra_last : addra;

//similar format for two insns
assign rf_addrc = /*(spr_valid & !spr_write) ? spr_addr[4:0] : 
		  spr_cs_fe ? addra_last : */addrc;

  
//
// RF write address is either from SPRS or normal from CPU control
//
assign rf_addrw = (spr_valid & spr_write) ? spr_addr[4:0] : addrw;

//
// RF write data is either from SPRS or normal from CPU datapath
//
assign rf_dataw = (spr_valid & spr_write) ? spr_dat_i : dataw;

//
// RF write enable is either from SPRS or normal from CPU control
//
always @(`OR1200_RST_EVENT rst or posedge clk)
	if (rst == `OR1200_RST_VALUE)
		rf_we_allow <=  1'b1;
	else if (~wb_freeze)
		rf_we_allow <=  ~flushpipe;

assign rf_we = ((spr_valid & spr_write) | (we & ~wb_freeze)) & rf_we_allow;

assign cy_we_o = cy_we_i && ~wb_freeze && rf_we_allow;
   
//
// CS RF A asserted when instruction reads operand A and ID stage
// is not stalled
//
assign rf_ena = (rda & ~id_freeze) | (spr_valid & !spr_write) | spr_cs_fe;

//
// CS RF B asserted when instruction reads operand B and ID stage
// is not stalled
//
assign rf_enb = rdb & ~id_freeze;

//added for two insns - similar format as the original
assign rf_enc = (rdc & ~id_freeze) | (spr_valid & !spr_write) | spr_cs_fe;
assign rf_end = rdd & ~id_freeze;
   
   
   
`ifdef OR1200_RFRAM_TWOPORT
//
// Instantiation of register file two-port RAM A
//
or1200_tpram_32x32 rf_a(
	// Port A
	.clk_a(clk),
	.rst_a(rst),
	.ce_a(rf_ena),
	.we_a(1'b0),
	.oe_a(1'b1),
	.addr_a(rf_addra),
	.di_a(32'h0000_0000),
	.do_a(from_rfa),

	// Port B
	.clk_b(clk),
	.rst_b(rst),
	.ce_b(rf_we),
	.we_b(rf_we),
	.oe_b(1'b0),
	.addr_b(rf_addrw),
	.di_b(rf_dataw),
	.do_b()
);

//
// Instantiation of register file two-port RAM B
//
or1200_tpram_32x32 rf_b(
	// Port A
	.clk_a(clk),
	.rst_a(rst),
	.ce_a(rf_enb),
	.we_a(1'b0),
	.oe_a(1'b1),
	.addr_a(addrb),
	.di_a(32'h0000_0000),
	.do_a(from_rfb),

	// Port B
	.clk_b(clk),
	.rst_b(rst),
	.ce_b(rf_we),
	.we_b(rf_we),
	.oe_b(1'b0),
	.addr_b(rf_addrw),
	.di_b(rf_dataw),
	.do_b()
);

`else
//The following is the option that seems to be enabled
//Just enabled two more registers based on this
`ifdef OR1200_RFRAM_DUALPORT

//
// Instantiation of register file two-port RAM A
//
   or1200_dpram #
     (
      .aw(5),
      .dw(32)
      )
   rf_a
     (
      // Port A
      .clk_a(clk),
      .ce_a(rf_ena),
      .addr_a(rf_addra),
      .do_a(from_rfa),
      
      // Port B
      .clk_b(clk),
      .ce_b(rf_we),
      .we_b(rf_we),
      .addr_b(rf_addrw),
      .di_b(rf_dataw)
      );

   //
   // Instantiation of register file two-port RAM B
   //
   or1200_dpram #
     (
      .aw(5),
      .dw(32)
      )
   rf_b
     (
      // Port A
      .clk_a(clk),
      .ce_a(rf_enb),
      .addr_a(addrb),
      .do_a(from_rfb),
      
      // Port B
      .clk_b(clk),
      .ce_b(rf_we),
      .we_b(rf_we),
      .addr_b(rf_addrw),
      .di_b(rf_dataw)
      );
//For second instruction
   or1200_dpram #
     (
      .aw(5),
      .dw(32)
      )
   rf_c
     (
      // Port A
      .clk_a(clk),
      .ce_a(rf_enc),
      .addr_a(rf_addrc),
      .do_a(from_rfc),
      
      // Port B
      .clk_b(clk),
      .ce_b(rf_we),
      .we_b(rf_we),
      .addr_b(rf_addrw),
      .di_b(rf_dataw)
      );

//For second instruction
   or1200_dpram #
     (
      .aw(5),
      .dw(32)
      )
   rf_d
     (
      // Port A
      .clk_a(clk),
      .ce_a(rf_end),
      .addr_a(addrd),
      .do_a(from_rfd),
      
      // Port B
      .clk_b(clk),
      .ce_b(rf_we),
      .we_b(rf_we),
      .addr_b(rf_addrw),
      .di_b(rf_dataw)
      );
`else

`ifdef OR1200_RFRAM_GENERIC

//
// Instantiation of generic (flip-flop based) register file
//
or1200_rfram_generic rf_a(
	// Clock and reset
	.clk(clk),
	.rst(rst),

	// Port A
	.ce_a(rf_ena),
	.addr_a(rf_addra),
	.do_a(from_rfa),

	// Port B
	.ce_b(rf_enb),
	.addr_b(addrb),
	.do_b(from_rfb),

	// Port W
	.ce_w(rf_we),
	.we_w(rf_we),
	.addr_w(rf_addrw),
	.di_w(rf_dataw)
);

`else

//
// RFRAM type not specified
//
initial begin
	$display("Define RFRAM type.");
	$finish;
end

`endif
`endif
`endif

endmodule
