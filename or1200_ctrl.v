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
//
//
// $Log: or1200_ctrl.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update: 
// Structure reordered and bugs fixed. 

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_ctrl
  (
   // Clock and reset
   clk, rst,
   
   // Internal i/f
   except_flushpipe, extend_flush, if_flushpipe, id_flushpipe, ex_flushpipe, 
   wb_flushpipe,
   id_freeze, ex_freeze, wb_freeze, if_insn, id_insn, ex_insn, abort_mvspr, 
   id_branch_op, ex_branch_op, ex_branch_taken, pc_we, 
   rf_addra, rf_addrb, rf_addrc, rf_addrd, rf_rda, rf_rdb, rf_rdc, rf_rdd, alu_op, alu_op2, mac_op,
   comp_op, rf_addrw, rfwb_op, fpu_op,
   wb_insn, id_simma, id_simmc, ex_simm, /*id_branch_addrtarget,*/ ex_branch_addrtarget, sel_a,
   sel_b, sel_c, sel_d, id_lsu_op,
   cust5_op, cust5_limm, id_pc, ex_pc, du_hwbkpt, 
   multicycle, wait_on, wbforw_valid, sig_syscall, sig_trap,
   force_dslot_fetch, no_more_dslot, id_void, ex_void, ex_spr_read, 
   ex_spr_write, 
   /*id_mac_op,*/ id_macrc_op, ex_macrc_op, rfe, except_illegal, dc_no_writethrough, half_insn_done_o, half_insn_done_next_o
   );

//
// I/O
//
input					clk;
input					rst;
input					id_freeze;
input					ex_freeze /* verilator public */;
input					wb_freeze /* verilator public */;
output					if_flushpipe;
output					id_flushpipe;
output					ex_flushpipe;
output					wb_flushpipe;
input					extend_flush;
input					except_flushpipe;
input                           abort_mvspr ;
input	[63:0]			if_insn;
output	[63:0]			id_insn;
output	[31:0]			ex_insn /* verilator public */;
output	[`OR1200_BRANCHOP_WIDTH-1:0]		ex_branch_op;
output	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
input						ex_branch_taken;
output	[`OR1200_REGFILE_ADDR_WIDTH-1:0]	rf_addrw;
output	[`OR1200_REGFILE_ADDR_WIDTH-1:0]	rf_addra;
output	[`OR1200_REGFILE_ADDR_WIDTH-1:0]	rf_addrb;
output	[`OR1200_REGFILE_ADDR_WIDTH-1:0]	rf_addrc; //added for two insns
output [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrd; //added for two insns
output					rf_rda;
output					rf_rdb;
output 				        rf_rdc; //added for two insns
output 			                rf_rdd; //added for two insns 
output	[`OR1200_ALUOP_WIDTH-1:0]		alu_op;
output [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2;
output	[`OR1200_MACOP_WIDTH-1:0]		mac_op;
output	[`OR1200_RFWBOP_WIDTH-1:0]		rfwb_op;
output  [`OR1200_FPUOP_WIDTH-1:0] 		fpu_op;      
input					pc_we;
output	[31:0]				wb_insn;
//output	[31:2]				id_branch_addrtarget;
output	[31:2]				ex_branch_addrtarget;
output	[`OR1200_SEL_WIDTH-1:0]		sel_a;
output	[`OR1200_SEL_WIDTH-1:0]		sel_b;
output	[`OR1200_SEL_WIDTH-1:0]		sel_c;
output	[`OR1200_SEL_WIDTH-1:0]		sel_d;   
output	[`OR1200_LSUOP_WIDTH-1:0]		id_lsu_op;
output	[`OR1200_COMPOP_WIDTH-1:0]		comp_op;
output	[`OR1200_MULTICYCLE_WIDTH-1:0]		multicycle;
output  [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_on;   
output	[4:0]				cust5_op;
output	[5:0]				cust5_limm;
input   [31:0]                          id_pc;
input   [31:0]                          ex_pc;
output	[31:0]				id_simma;
output  [31:0] 			        id_simmc;
output	[31:0]				ex_simm;
input					wbforw_valid;
input					du_hwbkpt;
output					sig_syscall;
output					sig_trap;
output					force_dslot_fetch;
output					no_more_dslot;
   reg 					no_more_dslot_next;		
output					id_void;
output					ex_void;
output					ex_spr_read;
output					ex_spr_write;
//output	[`OR1200_MACOP_WIDTH-1:0]	id_mac_op; as far as i can tell, this is not needed
output					id_macrc_op;
output					ex_macrc_op;
output					rfe;
output					except_illegal;
output  				dc_no_writethrough;
output 				half_insn_done_o; //needed to be sent to if.v while superscalar is being built to stop fetch of instructions halfway through 
output 			half_insn_done_next_o;
	
   
				
//
// Internal wires and regs
//
reg	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
wire	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_opa;
wire	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_opc;
reg	[`OR1200_BRANCHOP_WIDTH-1:0]		ex_branch_op;
reg	[`OR1200_BRANCHOP_WIDTH-1:0]		ex_branch_op_next;
   wire [`OR1200_BRANCHOP_WIDTH-1:0] 		ex_branch_opc;
   wire [`OR1200_BRANCHOP_WIDTH-1:0] 		ex_branch_opa;

wire   [31:0]                          id_pc2;   
   
//wire					if_maci_op;
//wire 				if_maci_opc; //added for two insns  
//`ifdef OR1200_MAC_IMPLEMENTED
//reg	[`OR1200_MACOP_WIDTH-1:0]		id_mac_op;
reg	[`OR1200_MACOP_WIDTH-1:0]		mac_op;
//reg					ex_macrc_op;
//`else
//wire	[`OR1200_MACOP_WIDTH-1:0]		mac_op;
//wire					ex_macrc_op;
//`endif
reg	[63:0] 				id_insn /* verilator public */;
reg	[31:0]				ex_insn /* verilator public */;
reg	[31:0]				wb_insn /* verilator public */;

reg	[`OR1200_REGFILE_ADDR_WIDTH-1:0]	wb_rfaddrw;

//   reg [`OR1200_SEL_WIDTH-1:0] 			sel_a;   
//   reg [`OR1200_SEL_WIDTH-1:0] 			sel_b;

reg					sel_imm;
wire 					sel_imma;
wire					sel_immc;			
//wire [31:0] 				id_insn_for_test;
   

   
//reg 	[31:0]				id_simm;
reg 	[31:0]				ex_simm_next;
wire 	[31:0]				ex_simma;
   wire [31:0] 				ex_simmc;
   reg [31:0] 				ex_simm;
//reg					sig_syscall;

//reg					except_illegal;
//wire					id_void;
wire					ex_void;
wire                                    wb_void;
reg                                     ex_delayslot_dsi;
reg                                     ex_delayslot_nop;

reg     [31:2]				ex_branch_addrtarget;
reg     [31:2]				ex_branch_addrtarget_next;
wire     [31:2]				ex_branch_addrtargeta;
wire     [31:2]				ex_branch_addrtargetc;

//reg [63:0] 				id_insn_intermediate; two insns recieved from if.v
reg 					half_insn_done;
reg 					half_insn_done_next;
reg [63:0] 				if_insn_intermediate;
reg 					data_dependent;
reg 					multiply_stall;
reg 					load_or_store_stall;
reg 					fpu_hazard_stall;
reg 					branch_hazard_stall;
reg 					wait_hazard_stall;
reg 					multicycle_hazard_stall;
reg 					system_stall;
   
   
   
//wires coming out of the first and second id decode
   wire [31:0] 				id_simma;
   wire [31:0] 					id_insn_for_testa;
   wire 					sig_syscalla;
   wire 					id_macrc_opa;
   wire						ex_macrc_opa;		
   wire 					dc_no_writethrougha;
   wire 					id_voida;
   //wire [31:2] 					id_branch_addrtargeta;
   wire [`OR1200_FPUOP_WIDTH-1:0] 		fpu_opa;
   wire [`OR1200_MULTICYCLE_WIDTH-1:0] 		multicyclea;
   wire [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_ona;   
   wire [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrwa;
   wire 					except_illegala;
   wire [`OR1200_ALUOP_WIDTH-1:0] 		alu_opa;
   wire [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2a;
   wire 					spr_reada;
   wire						spr_writea;
   wire [`OR1200_MACOP_WIDTH-1:0] 		mac_opa;		
   wire [`OR1200_RFWBOP_WIDTH-1:0] 		rfwb_opa;
   wire [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_opa;
   wire [`OR1200_COMPOP_WIDTH-1:0] 		comp_opa;
   wire 					sig_trapa;
//second instruction
 wire [31:0] 				id_simmc;
   wire [31:0] 					id_insn_for_testc;
   wire 					sig_syscallc;
   wire 					id_macrc_opc;
   wire						ex_macrc_opc;		
   wire 					dc_no_writethroughc;
   wire 					id_voidc;
   //wire [31:2] 					id_branch_addrtargetc;
   wire [`OR1200_FPUOP_WIDTH-1:0] 		fpu_opc;
   wire [`OR1200_MULTICYCLE_WIDTH-1:0] 		multicyclec;
   wire [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_onc;   
   wire [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrwc;
   wire 					except_illegalc;
   wire [`OR1200_ALUOP_WIDTH-1:0] 		alu_opc;
   wire [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2c;
   wire 					spr_readc;
   wire						spr_writec;
   wire [`OR1200_MACOP_WIDTH-1:0] 		mac_opc;		
   wire [`OR1200_RFWBOP_WIDTH-1:0] 		rfwb_opc;
   wire [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_opc;
   wire [`OR1200_COMPOP_WIDTH-1:0] 		comp_opc;
   wire 					sig_trapc;

					
//added for testing and selecting between two register choices
   reg [31:0] 					id_simm;
   //reg [31:0] 					id_insn_for_test;
   reg 						sig_syscall;
   reg 						id_macrc_op;
   reg 						id_macrc_op_next;
   reg 						dc_no_writethrough;
   reg 						id_void;
   reg 						id_void_next;						
   //reg [31:2] 					id_branch_addrtarget;
   //reg [31:2] 					id_branch_addrtarget_next;
   reg [`OR1200_FPUOP_WIDTH-1:0] 		fpu_op;
   reg [`OR1200_FPUOP_WIDTH-1:0] 		fpu_op_next;
   reg [`OR1200_MULTICYCLE_WIDTH-1:0] 		multicycle;
   reg [`OR1200_MULTICYCLE_WIDTH-1:0] 		multicycle_next;
   reg [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_on;
   reg [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_on_next;   
   reg [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrw;
   wire						except_illegal;
   reg [`OR1200_ALUOP_WIDTH-1:0] 		alu_op;
   reg [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2;
   reg 						spr_read;
   reg 						spr_write;
   reg 						ex_macrc_op;
   reg 						ex_macrc_op_next;	
   reg [`OR1200_RFWBOP_WIDTH-1:0] 		rfwb_op;
   reg [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_op;
   reg [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_op_next;
   reg [`OR1200_COMPOP_WIDTH-1:0] 		comp_op;
   reg 						sig_trap;
   reg [`OR1200_MACOP_WIDTH-1:0] 		mac_op_next;		
   reg 						sel_imm_next;
   reg 	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op_next;
   reg 						half_insn_done_next_next;
   reg 						no_more_dslot_next_next;
   
//used for selecting between the two possibilities when id_insn is still split

   reg sig_syscall_next;
   reg dc_no_writethrough_next;
   reg [`OR1200_REGFILE_ADDR_WIDTH-1:0] rf_addrw_next;
   reg except_illegal_next;
   reg [`OR1200_ALUOP_WIDTH-1:0] alu_op_next;
   reg [`OR1200_ALUOP2_WIDTH-1:0] alu_op2_next;
   reg spr_read_next;
   reg spr_write_next;
   reg [`OR1200_RFWBOP_WIDTH-1:0] rfwb_op_next;
   reg [`OR1200_COMPOP_WIDTH-1:0] comp_op_next;
   reg sig_trap_next;
    
// used for testing   
reg [31:0] ex_insn_intermediate;
   

//
// Force fetch of delay slot instruction when jump/branch is preceeded by 
// load/store instructions
//
assign force_dslot_fetch = 1'b0;
assign no_more_dslot = (|ex_branch_op & !id_void & ex_branch_taken) | 
		       (ex_branch_op == `OR1200_BRANCHOP_RFE);

assign ex_void = (ex_insn[31:26] == `OR1200_OR32_NOP) & ex_insn[16];
assign wb_void = (wb_insn[31:26] == `OR1200_OR32_NOP) & wb_insn[16];

assign ex_spr_write = spr_write && !abort_mvspr;
assign ex_spr_read = spr_read && !abort_mvspr;

//
// ex_delayslot_dsi: delay slot insn is in EX stage
// ex_delayslot_nop: (filler) nop insn is in EX stage (before nops 
//                   jump/branch was executed)
//
//  ex_delayslot_dsi & !ex_delayslot_nop - DS insn in EX stage
//  !ex_delayslot_dsi & ex_delayslot_nop - NOP insn in EX stage, 
//       next different is DS insn, previous different was Jump/Branch
//  !ex_delayslot_dsi & !ex_delayslot_nop - normal insn in EX stage
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
        if (rst == `OR1200_RST_VALUE) begin
		ex_delayslot_nop <=  1'b0;
		ex_delayslot_dsi <=  1'b0;
	end
	else if (!ex_freeze & !ex_delayslot_dsi & ex_delayslot_nop) begin
		ex_delayslot_nop <=  id_void;
		ex_delayslot_dsi <=  !id_void;
	end
	else if (!ex_freeze & ex_delayslot_dsi & !ex_delayslot_nop) begin
		ex_delayslot_nop <=  1'b0;
		ex_delayslot_dsi <=  1'b0;
	end
	else if (!ex_freeze) begin
		ex_delayslot_nop <=  id_void && ex_branch_taken && 
				     (ex_branch_op != `OR1200_BRANCHOP_NOP) && 
				     (ex_branch_op != `OR1200_BRANCHOP_RFE);
	        ex_delayslot_dsi <=  !id_void && ex_branch_taken && 
				     (ex_branch_op != `OR1200_BRANCHOP_NOP) && 
				     (ex_branch_op != `OR1200_BRANCHOP_RFE);
	end
end

//
// Flush pipeline
//
assign if_flushpipe = except_flushpipe | pc_we | extend_flush;
assign id_flushpipe = except_flushpipe | pc_we | extend_flush;
assign ex_flushpipe = except_flushpipe | pc_we | extend_flush;
assign wb_flushpipe = except_flushpipe | pc_we | extend_flush;

//
// cust5_op, cust5_limm (L immediate)
//
assign cust5_op = ex_insn[4:0];
assign cust5_limm = ex_insn[10:5];

//
//
//
assign rfe = (id_branch_op == `OR1200_BRANCHOP_RFE) | 
	     (ex_branch_op == `OR1200_BRANCHOP_RFE);

//As far as I can tell, this is only needed for a certain simulator, so I did not modify it   
`ifdef verilator
   // Function to access wb_insn (for Verilator). Have to hide this from
   // simulator, since functions with no inputs are not allowed in IEEE
   // 1364-2001.
   function [31:0] get_wb_insn;
      // verilator public
      get_wb_insn = wb_insn;
   endfunction // get_wb_insn

   // Function to access id_insn (for Verilator). Have to hide this from
   // simulator, since functions with no inputs are not allowed in IEEE
   // 1364-2001.
   function [31:0] get_id_insn;
      // verilator public
      get_id_insn = id_insn;
   endfunction // get_id_insn

   // Function to access ex_insn (for Verilator). Have to hide this from
   // simulator, since functions with no inputs are not allowed in IEEE
   // 1364-2001.
   function [31:0] get_ex_insn;
      // verilator public
      get_ex_insn = ex_insn;
   endfunction // get_ex_insn
   
`endif 
   
//
// rf_addrw in wb stage (used in forwarding logic)
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		wb_rfaddrw <=  5'd0;
	else if (!wb_freeze)
		wb_rfaddrw <=  rf_addrw;
end

//
// Instruction latch in id_insn - modified to recieve two insns
//
//This signal is needed while the superscalar pipeline is being built
assign half_insn_done_o = half_insn_done;
assign half_insn_done_next_o = half_insn_done_next;
   

//This is needed to ensure that only one instruction is executed following the dslot rather than two if a branch occurs in the second half of a set of two insns
always @(*) begin
   if (no_more_dslot_next & half_insn_done_next) begin
      if_insn_intermediate <= {2{`OR1200_OR32_NOP,26'h141_0000}};
   end
   else
     if_insn_intermediate <= if_insn;
end

always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE) begin //modified to make nops type 141
	   id_insn <= {2{`OR1200_OR32_NOP, 26'h141_0000}};
	end
        else if (id_flushpipe) begin
           id_insn <= {2{`OR1200_OR32_NOP, 26'h141_0000}};
	end
	else if (!id_freeze) begin
	      id_insn <= if_insn_intermediate;
	 
`ifdef OR1200_VERBOSE
// synopsys translate_off
		$display("%t: id_insn <= %h", $time, if_insn);
// synopsys translate_on
`endif
	end
end

//data dependency check of two insns in the same stage of pipeline   
always @(id_insn) begin
   if (((id_insn[25:21] == id_insn[52:48]) | (id_insn[25:21] == id_insn[47:43])) & (id_insn[63:58] != `OR1200_OR32_NOP)) begin
      case (id_insn[31:26])
	`OR1200_OR32_JAL, `OR1200_OR32_JALR, `OR1200_OR32_MOVHI, `OR1200_OR32_MFSPR, `OR1200_OR32_LWZ, `OR1200_OR32_LWS, `OR1200_OR32_LBZ, `OR1200_OR32_LBS,`OR1200_OR32_LHZ, `OR1200_OR32_LHS, `OR1200_OR32_ADDI, `OR1200_OR32_ADDIC, `OR1200_OR32_ANDI, `OR1200_OR32_ORI,
`ifdef OR1200_MULT_IMPLEMENTED
		`OR1200_OR32_MULI,
`endif
		  `OR1200_OR32_SH_ROTI, `OR1200_OR32_ALU, 
`ifdef OR1200_ALU_IMPL_CUST5
		    `OR1200_OR32_CUST5,
`endif
`ifdef OR1200_FPU_IMPLEMENTED
		      `OR1200_OR32_FLOAT,
`endif
			`OR1200_OR32_XORI:
			  data_dependent <= 1'b1;
	default:
	  data_dependent <= 1'b0;
      endcase // case (id_insn[31:26])
   end // if ((id_insn[25:21] == id_insn[52:48]) | (id_insn[25:21] == id_insn[47:43]))
   else
     data_dependent <= 1'b0;
end

//structural hazard check for MAC unit
always @(id_macrc_opa or id_macrc_opc) begin
   if (id_macrc_opa | id_macrc_opc) begin
      multiply_stall <= 1'b1;
      if (id_macrc_opa)
	id_macrc_op <= 1'b1;
      else
	id_macrc_op <= 1'b0;
   end
   else begin
      multiply_stall <= 1'b0;
      id_macrc_op <= 1'b0;
   end 
end 

//structural hazard check for LSU
always @(id_lsu_opa or id_lsu_opc) begin
   if ((id_lsu_opa != `OR1200_LSUOP_NOP) | (id_lsu_opc != `OR1200_LSUOP_NOP)) begin
      load_or_store_stall <= 1'b1;
      if (id_lsu_opa)
	id_lsu_op <= id_lsu_opa;
      else
	id_lsu_op <= `OR1200_LSUOP_NOP;
   end
   else begin
      load_or_store_stall <= 1'b0;
      id_lsu_op <= `OR1200_LSUOP_NOP;
   end 
end

//structural hazard check for FPU
always @(fpu_opa or fpu_opc) begin
   if ((fpu_opa[`OR1200_FPUOP_WIDTH]) | (fpu_opc[`OR1200_FPUOP_WIDTH])) begin
      fpu_hazard_stall <= 1'b1;
      if ((fpu_opa[`OR1200_FPUOP_WIDTH]))
	fpu_op <= fpu_opa;
      else
	fpu_op <= {`OR1200_FPUOP_WIDTH{1'b0}};
   end
   else begin
      fpu_hazard_stall <= 1'b0;
      fpu_op <= {`OR1200_FPUOP_WIDTH{1'b0}};
   end 
end 

//structural hazard check for Branch Unit (genpc)
//only need to stall if second insn is a branch because other ALU can handle normal dslot instruction
always @(id_branch_opa or id_branch_opc) begin
   id_branch_op <= id_branch_opa;
   if (id_branch_opc != `OR1200_BRANCHOP_NOP) 
     branch_hazard_stall <= 1'b1;
   else
     branch_hazard_stall <= 1'b0; 
end 

//stall due to waiting on one of the structures to conclude
//this includes multiplication, lsu, fpu, or move to spr when dc writethrough
always @(wait_ona or wait_onc) begin
   if ((wait_ona != `OR1200_WAIT_ON_NOTHING) | (wait_onc != `OR1200_WAIT_ON_NOTHING)) begin
      wait_hazard_stall <= 1'b1;
      if ((wait_ona != `OR1200_WAIT_ON_NOTHING))
	wait_on <= wait_ona;
      else
	wait_on <= `OR1200_WAIT_ON_NOTHING;
   end
   else begin
      wait_hazard_stall <= 1'b0;
      wait_on <= `OR1200_WAIT_ON_NOTHING;
   end 
end 
 
//stall due to return from exception or move from special purpose register  
always @(multicyclea or multicyclec) begin
   if ((multicyclea != `OR1200_ONE_CYCLE) | (multicyclec != `OR1200_ONE_CYCLE)) begin
      multicycle_hazard_stall <= 1'b1;
      if ((multicyclea != `OR1200_ONE_CYCLE))
	multicycle <= multicyclea;
      else
	multicycle <= `OR1200_ONE_CYCLE;
   end
   else begin
      multicycle_hazard_stall <= 1'b0;
      multicycle <= `OR1200_ONE_CYCLE;
   end 
end

//stall due to system call or trap  
always @(id_insn) begin
   if ((id_insn[31:26] == `OR1200_OR32_XSYNC) | (id_insn[63:58] == `OR1200_OR32_XSYNC) | du_hwbkpt) begin
      system_stall <= 1'b1;
   end
   else begin
      system_stall <= 1'b0;
   end 
end 
   
   
//
// Instruction latch in ex_insn
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE) begin
	   ex_insn <=  {`OR1200_OR32_NOP, 26'h141_0000};
	   ex_insn_intermediate <= {`OR1200_OR32_NOP, 26'h141_0000};
	   half_insn_done <= 1'b0;
	end
	else if ((!ex_freeze & id_freeze) | ex_flushpipe ) begin
	   ex_insn <=  {`OR1200_OR32_NOP, 26'h141_0000};	// NOP -> ex_insn[16] must be 1
	   ex_insn_intermediate <= {`OR1200_OR32_NOP, 26'h141_0000};
	   half_insn_done <= 1'b0;
	end
	else if (!ex_freeze) begin
	   //half insn done used for data or structural hazard - second case is to handle delay slot if it occurs in the second half of instruction
	   if (!half_insn_done | (half_insn_done & no_more_dslot)) begin
	      ex_insn <= id_insn[31:0];
	      //added to keep half_insn_done staying high in the second case
	      if ((half_insn_done & no_more_dslot) | data_dependent | multiply_stall | load_or_store_stall | fpu_hazard_stall | branch_hazard_stall | wait_hazard_stall | multicycle_hazard_stall | system_stall) begin
		 half_insn_done <= 1'b0;
		 ex_insn_intermediate <= {`OR1200_OR32_NOP,26'h141_0000};
	      end
	      else if (id_insn[63:58] != `OR1200_OR32_NOP) begin
	         half_insn_done <= 1'b1;
		 ex_insn_intermediate <= id_insn[63:32];	 
	      end
	      //other cases dont matter for ex_insn_intermediate since it will not be executed due to half insn done
	   end
	   else begin
	      ex_insn <= ex_insn_intermediate; //half_insn_done so no change to id_insn
	      half_insn_done <= 1'b0;
	   end
	   
`ifdef OR1200_VERBOSE
// synopsys translate_off
	   $display("%t: ex_insn <= %h", $time, id_insn);
// synopsys translate_on
`endif
	end
end
   
//
// Instruction latch in wb_insn - This does nothing as far as I can tell except it is used by the simulator (or1200_monitor)
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		wb_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};
	// wb_insn should not be changed by exceptions due to correct 
	// recording of display_arch_state in the or1200_monitor! 
	// wb_insn changed by exception is not used elsewhere! 
	else if (!wb_freeze) begin
		wb_insn <=  ex_insn;
	end
end


//chooses between first and second insn during testing phase
always @(posedge clk or `OR1200_RST_EVENT rst) begin
  if ((rst == `OR1200_RST_VALUE) | id_flushpipe) begin
     half_insn_done_next <= 1'b0;
     half_insn_done_next_next <= 1'b0;
     no_more_dslot_next <= 1'b0;
     no_more_dslot_next_next <= 1'b0;
     
  end
  else if (!ex_freeze) begin
     half_insn_done_next <= half_insn_done;
     half_insn_done_next_next <= half_insn_done_next;
     no_more_dslot_next <= no_more_dslot;
     no_more_dslot_next_next <= no_more_dslot_next;
     if (!id_freeze) begin
     
	//id_branch_op_next <= id_branch_opc;
	id_void_next <= id_voidc;
        //fpu_op_next <= fpu_opc;
	//multicycle_next <= multicyclec;
	//wait_on_next <= wait_onc;
	//id_lsu_op_next <= id_lsu_opc;

	//insns below save signals during testing
	
	//ex_macrc_op_next <= ex_macrc_opc;
	//sig_syscall_next <= sig_syscallc;
	//dc_no_writethrough_next <= dc_no_writethroughc;
	rf_addrw_next <= rf_addrwc;
	//except_illegal_next <= except_illegalc;
	alu_op_next <= alu_opc;
	alu_op2_next <= alu_op2c;
	//spr_read_next <= spr_readc;
	//spr_write_next <= spr_writec;
	//mac_op_next <= mac_opc;
	rfwb_op_next <= rfwb_opc;
	comp_op_next <= comp_opc;
	//sig_trap_next <= sig_trapc;
	//ex_branch_op_next <= ex_branch_opc;
	//ex_simm_next <= ex_simmc;
	//ex_branch_addrtarget_next <= ex_branch_addrtargetc;
     end
  end
end // always @ (posedge clk or `OR1200_RST_EVENT rst)
   
/*always @(id_branch_op_next or id_branch_opa) begin
   if (half_insn_done)
     id_branch_op <= id_branch_op_next;
   else
     id_branch_op <= id_branch_opa;
end*/


//This is used for testing due the current split up in the execute phase
always @(*) begin
   if (!half_insn_done/*_next | no_more_dslot_next*/) begin
      id_void <= id_voida;
      //fpu_op <= fpu_opa;
      //multicycle <= multicyclea;
      //wait_on <= wait_ona;
      //id_lsu_op <= id_lsu_opa;
   end // if (!half_ins_done_next)
   else begin
      id_void <= id_void_next;
      //fpu_op <= fpu_op_next;
      //multicycle <= multicycle_next;
      //wait_on <= wait_on_next;
      //id_lsu_op <= id_lsu_op_next;
   end // else: !if(!half_insn_done_next)

   //only the first pipeline needs these signals
   ex_macrc_op <= ex_macrc_opa;
   mac_op <= mac_opa;
   ex_branch_op <= ex_branch_opa;
   ex_branch_addrtarget <= ex_branch_addrtargeta;
   dc_no_writethrough <= dc_no_writethrougha;
   spr_read <= spr_reada;
   spr_write <= spr_writea;
   ex_simm <= ex_simma;
   sig_syscall <= sig_syscalla;
   sig_trap <= sig_trapa;
   
   //the instructions below are registered while the ones above were not
   if (!half_insn_done_next) begin
      
      rf_addrw <= rf_addrwa;
      //except_illegal <= except_illegala;
      alu_op <= alu_opa;
      alu_op2 <= alu_op2a;
      rfwb_op <= rfwb_opa;
      comp_op <= comp_opa;

   end // if (!half_insn_done_next_next)
   else begin
      //ex_macrc_op <= ex_macrc_op_next;
      //sig_syscall <= sig_syscall_next;
      //dc_no_writethrough <= dc_no_writethrough_next;
      rf_addrw <= rf_addrw_next;
      //except_illegal <= except_illegal_next;
      alu_op <= alu_op_next;
      alu_op2 <= alu_op2_next;
      //spr_read <= spr_read_next;
      //spr_write <= spr_write_next;
      //mac_op <= mac_op_next;
      rfwb_op <= rfwb_op_next;
      comp_op <= comp_op_next;
      //sig_trap <= sig_trap_next;
      //ex_branch_op <= ex_branch_op_next;
      //ex_simm <= ex_simm_next;
      //ex_branch_addrtarget <= ex_branch_addrtarget_next;      
   end
end

//The abort instructions that rely on this do not seem to identify the address that caused the exception so no distinction was made
assign except_illegal = except_illegala | except_illegalc;   
   
//This is for the two insns being decoded in the id stage

or1200_ctrl_id_decode or1200_ctrl_id_decode1(
	.clk(clk),
	.rst(rst),
	.id_insn(id_insn[31:0]),
	.ex_freeze(ex_freeze),
	.id_freeze(id_freeze),
	.ex_flushpipe(ex_flushpipe),			     
	.id_pc(id_pc),
        .du_hwbkpt(du_hwbkpt),
	.abort_mvspr(abort_mvspr),
	.sel_imm(sel_imma),				     
        //.id_insn_for_test(id_insn_for_testa),
	.wb_rfaddrw(wb_rfaddrw),
	.wbforw_valid(wbforw_valid),
        .id_branch_op(id_branch_opa),
	.id_simm(id_simma),
	.id_macrc_op(id_macrc_opa),
        .ex_macrc_op(ex_macrc_opa), //sync
	.sig_syscall(sig_syscalla), //sync
        .dc_no_writethrough(dc_no_writethrougha), //sync
	.id_void(id_voida),
	//.id_branch_addrtarget(id_branch_addrtargeta),
	.fpu_op(fpu_opa),
	.multicycle(multicyclea),
	.wait_on(wait_ona),
	.rf_addrw(rf_addrwa),  //synchronous
	.except_illegal(except_illegala), //synchronous
	.alu_op(alu_opa), //sync
	.alu_op2(alu_op2a), //sync
	.spr_read(spr_reada), //sync
	.spr_write(spr_writea), //sync
	.mac_op(mac_opa), //sync
	.rfwb_op(rfwb_opa), //sync
	.id_lsu_op(id_lsu_opa),
	.comp_op(comp_opa), //sync
	.sig_trap(sig_trapa), //sync				     
	.sel_a(sel_a),
	.sel_b(sel_b),
	.ex_branch_op(ex_branch_opa), //sync
        .ex_simm(ex_simma), //sync
	.ex_branch_addrtarget(ex_branch_addrtargeta) //sync	     
);

//This is needed because the program counter of the second instruction is +4 from the first   
assign id_pc2 = {id_pc[31:2], 2'b0} + 32'h4;   
   
or1200_ctrl_id_decode or1200_ctrl_id_decode2(
	.clk(clk),
	.rst(rst),
	.id_insn(id_insn[63:32]),
	.ex_freeze(ex_freeze),
	.id_freeze(id_freeze),
	.ex_flushpipe(ex_flushpipe),			     
	.id_pc(id_pc2),
        .du_hwbkpt(du_hwbkpt),
	.abort_mvspr(abort_mvspr),
	.sel_imm(sel_immc),
        //.id_insn_for_test(id_insn_for_testc),
	.wb_rfaddrw(wb_rfaddrw),
	.wbforw_valid(wbforw_valid),
	.id_branch_op(id_branch_opc),
	.id_simm(id_simmc),
	.id_macrc_op(id_macrc_opc),
	.ex_macrc_op(ex_macrc_opc), //sync
        .sig_syscall(sig_syscallc), //sync
        .dc_no_writethrough(dc_no_writethroughc), //sync
	.id_void(id_voidc),
	//.id_branch_addrtarget(id_branch_addrtargetc),
	.fpu_op(fpu_opc),
	.multicycle(multicyclec),
	.wait_on(wait_onc),
	.rf_addrw(rf_addrwc),  //synchronous
	.except_illegal(except_illegalc), //synchronous
	.alu_op(alu_opc), //sync
	.alu_op2(alu_op2c), //sync
	.spr_read(spr_readc), //sync
	.spr_write(spr_writec), //sync
	.mac_op(mac_opc), //sync
	.rfwb_op(rfwb_opc), //sync
	.id_lsu_op(id_lsu_opc),
	.comp_op(comp_opc), //sync
	.sig_trap(sig_trapc), //sync
	.sel_a(sel_c),
	.sel_b(sel_d),
	.ex_branch_op(ex_branch_opc), //sync 	
	.ex_simm(ex_simmc), 			  
	.ex_branch_addrtarget(ex_branch_addrtargetc) //sync
);

   
//This is for the two insns being decoded in the if stage
   
or1200_ctrl_if_decode or1200_ctrl_if_decode1(
	.clk(clk),
	.rst(rst),
	.if_insn(if_insn_intermediate[31:0]),
	.id_freeze(id_freeze),
	.id_flushpipe(id_flushpipe),
	.rf_addr1(rf_addra),
	.rf_addr2(rf_addrb),
	.rf_rd1(rf_rda),
	.rf_rd2(rf_rdb),
	.sel_imm(sel_imma), //synchronous
	.id_branch_op(id_branch_opa) //sync
);

or1200_ctrl_if_decode or1200_ctrl_if_decode2(
	.clk(clk),
	.rst(rst),
	.if_insn(if_insn_intermediate[63:32]), 
	.id_freeze(id_freeze),
	.id_flushpipe(id_flushpipe),
	.rf_addr1(rf_addrc),
	.rf_addr2(rf_addrd),
	.rf_rd1(rf_rdc),
	.rf_rd2(rf_rdd),
	.sel_imm(sel_immc), //sync
	.id_branch_op(id_branch_opc) //sync
);

   
endmodule
