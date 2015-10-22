//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's CPU                                                ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Instantiation of internal CPU blocks. IFETCH, SPRS, FRZ,    ////
////  ALU, EXCEPT, ID, WBMUX, OPERANDMUX, RF etc.                 ////
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
// $Log: or1200_cpu.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update: 
// Structure reordered and bugs fixed. 

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_cpu(
	// Clk & Rst
	clk, rst,

	// Insn interface
	ic_en,
	icpu_adr_o, icpu_cycstb_o, icpu_sel_o, icpu_tag_o,
	icpu_dat_i, icpu_ack_i, icpu_rty_i, icpu_err_i, icpu_adr_i, icpu_tag_i,
	immu_en,

	// Debug unit
	id_void, id_insn, ex_void, 
	ex_insn, ex_freeze, wb_insn, wb_freeze, id_pc, ex_pc, wb_pc, branch_op,
	spr_dat_npc, rf_dataw, ex_flushpipe, 
	du_stall, du_addr, du_dat_du, du_read, du_write, du_except_stop, 
	du_except_trig, du_dsr, du_dmr1, du_hwbkpt, du_hwbkpt_ls_r, du_dat_cpu,
	du_lsu_store_dat, du_lsu_load_dat, 
	abort_mvspr, abort_ex,
	
	// Data interface
	dc_en,
	dcpu_adr_o, dcpu_cycstb_o, dcpu_we_o, dcpu_sel_o, dcpu_tag_o, 
        dcpu_dat_o, dcpu_dat_i, dcpu_ack_i, dcpu_rty_i, dcpu_err_i, dcpu_tag_i,
	sb_en, dmmu_en, dc_no_writethrough,

	// SR Interface
	boot_adr_sel_i,

	// Interrupt & tick exceptions
	sig_int, sig_tick,

	// SPR interface
	supv, spr_addr, spr_dat_cpu, spr_dat_pic, spr_dat_tt, spr_dat_pm,
	spr_dat_dmmu, spr_dat_immu, spr_dat_du, spr_cs, spr_we, mtspr_dc_done, if_two_insns_ic
);

parameter dw = `OR1200_OPERAND_WIDTH;
parameter aw = `OR1200_REGFILE_ADDR_WIDTH;
parameter boot_adr = `OR1200_BOOT_ADR;

//
// I/O ports
//

//
// Clk & Rst
//
input 				clk;
input 				rst;

//
// Insn (IC) interface
//
output				ic_en;
output	[31:0]			icpu_adr_o;
output				icpu_cycstb_o;
output	[3:0]			icpu_sel_o;
output	[3:0]			icpu_tag_o;
input	[63:0]			icpu_dat_i; //modified for two insns
input				icpu_ack_i;
input				icpu_rty_i;
input				icpu_err_i;
input	[31:0]			icpu_adr_i;
input	[3:0]			icpu_tag_i;

//
// Insn (IMMU) interface
//
output				immu_en;

//
// Debug interface
//
output                          id_void;
output	[63:0]			id_insn;
output                          ex_void;
output	[63:0]			ex_insn;
output				ex_freeze;
output	[31:0]			wb_insn;
output				wb_freeze;
output	[31:0]			id_pc;
output	[31:0]			ex_pc;
output	[31:0]			wb_pc;
output                          ex_flushpipe;
output	[`OR1200_BRANCHOP_WIDTH-1:0]	branch_op;

input				du_stall;
input	[dw-1:0]		du_addr;
input	[dw-1:0]		du_dat_du;
input				du_read;
input				du_write;
input	[`OR1200_DU_DSR_WIDTH-1:0]	du_dsr;
input	[24:0]			du_dmr1;
input				du_hwbkpt;
input				du_hwbkpt_ls_r;
output	[13:0]			du_except_trig;
output	[13:0]			du_except_stop;
output	[dw-1:0]		du_dat_cpu;
output	[dw-1:0]		rf_dataw;
output	[dw-1:0]		du_lsu_store_dat;
output	[dw-1:0]		du_lsu_load_dat;

//
// Data (DC) interface
//
output	[31:0]			dcpu_adr_o;
output				dcpu_cycstb_o;
output				dcpu_we_o;
output	[3:0]			dcpu_sel_o;
output	[3:0]			dcpu_tag_o;
output	[31:0]			dcpu_dat_o;
input	[31:0]			dcpu_dat_i;
input				dcpu_ack_i;
input				dcpu_rty_i;
input				dcpu_err_i;
input	[3:0]			dcpu_tag_i;
output				dc_en;
output  			dc_no_writethrough;
   
//
// Data (DMMU) interface
//
output				sb_en;
output				dmmu_en;
output				abort_ex;
output				abort_mvspr;

//
// SR Interface 
//
input				boot_adr_sel_i;

//
// SPR interface
//
output				supv;
input	[dw-1:0]		spr_dat_pic;
input	[dw-1:0]		spr_dat_tt;
input	[dw-1:0]		spr_dat_pm;
input	[dw-1:0]		spr_dat_dmmu;
input	[dw-1:0]		spr_dat_immu;
input	[dw-1:0]		spr_dat_du;
output	[dw-1:0]		spr_addr;
output	[dw-1:0]		spr_dat_cpu;
output	[dw-1:0]		spr_dat_npc;
output	[31:0]			spr_cs;
output				spr_we;
input   			mtspr_dc_done;
input   			if_two_insns_ic;
   
//
// Interrupt exceptions
//
input				sig_int;
input				sig_tick;

//
// Internal wires
//
wire	[63:0]			if_insn; //twice as wide for two insns
wire				saving_if_insn;
wire	[31:0]			if_pc;
wire	[aw-1:0]		rf_addrw;
wire	[aw-1:0]		rf_addrwc;
wire	[aw-1:0] 		rf_addra;
wire	[aw-1:0] 		rf_addrb;
wire    [aw-1:0] 		rf_addrc; //added for the second set of registers
wire [aw-1:0] 		        rf_addrd; //added for second set	
wire				rf_rda;
wire				rf_rdb;
wire          			rf_rdc; //added for second set 
wire     			rf_rdd; //added for second set
wire	[dw-1:0]		id_simma;
wire    [dw-1:0] 		id_simmc;   
wire	[dw-1:2]		ex_branch_addrtarget;
wire	[`OR1200_ALUOP_WIDTH-1:0]	alu_op;
wire	[`OR1200_ALUOP2_WIDTH-1:0]	alu_op2;
wire	[`OR1200_ALUOP_WIDTH-1:0]	alu_opc;
wire	[`OR1200_ALUOP2_WIDTH-1:0]	alu_op2c;   
wire	[`OR1200_COMPOP_WIDTH-1:0]	comp_op;
wire	[`OR1200_COMPOP_WIDTH-1:0]	comp_opc;
wire	[`OR1200_BRANCHOP_WIDTH-1:0]	pre_branch_op;
wire	[`OR1200_BRANCHOP_WIDTH-1:0]	branch_op;
wire	[`OR1200_LSUOP_WIDTH-1:0]	id_lsu_op;
wire				genpc_freeze;
wire				if_freeze;
wire				id_freeze;
wire				ex_freeze;
wire				wb_freeze;
wire	[`OR1200_SEL_WIDTH:0]	sel_a; //modified to add one bit to have more options
wire	[`OR1200_SEL_WIDTH:0]	sel_b;
wire	[`OR1200_SEL_WIDTH:0]	sel_c;
wire	[`OR1200_SEL_WIDTH:0]	sel_d;
wire	[`OR1200_RFWBOP_WIDTH-1:0]	rfwb_op;
wire	[`OR1200_RFWBOP_WIDTH-1:0]	rfwb_opc;
wire    [`OR1200_FPUOP_WIDTH-1:0]       fpu_op;
wire	[dw-1:0]		rf_dataw;
wire	[dw-1:0]		rf_datawc;   
wire	[dw-1:0]		rf_dataa;
wire	[dw-1:0]		rf_datab;
reg	[dw-1:0]		rf_dataa_inter;
reg	[dw-1:0]		rf_datab_inter;   
wire    [dw-1:0] 		rf_datac;
wire    [dw-1:0] 		rf_datad;
wire	[dw-1:0]		muxed_a;				
wire	[dw-1:0]		muxed_b;
wire	[dw-1:0]		muxed_c;
wire	[dw-1:0]		muxed_d;
wire	[dw-1:0]		wb_forw;
reg	[dw-1:0]		wb_forwc;
wire				wbforw_valid;
reg				wbforw_validc;
wire	[dw-1:0]		operand_a;
wire	[dw-1:0]		operand_b;
wire	[dw-1:0]		operand_a_inter;
wire	[dw-1:0]		operand_b_inter;   
wire	[dw-1:0]		operand_c;
wire	[dw-1:0]		operand_d;
reg	[dw-1:0]		operand_c_next;
reg	[dw-1:0]		operand_d_next;
wire     [dw-1:0] 		operand_branch;   
wire	[dw-1:0]		alu_dataout;
wire	[dw-1:0]		alu_dataoutc;
wire	[dw-1:0]		lsu_dataout;
wire	[dw-1:0]		sprs_dataout;
wire	[dw-1:0]		fpu_dataout;
wire     			fpu_done;
wire	[31:0]			ex_simm;
wire	[`OR1200_MULTICYCLE_WIDTH-1:0]	multicycle;
wire    [`OR1200_WAIT_ON_WIDTH-1:0]	wait_on;      
wire	[`OR1200_EXCEPT_WIDTH-1:0]	except_type;
wire	[4:0]			cust5_op;
wire	[5:0]			cust5_limm;
wire	[4:0]			cust5_opc;
wire	[5:0]			cust5_limmc;   
wire				if_flushpipe;
wire				id_flushpipe;
wire				ex_flushpipe;
wire				wb_flushpipe;
wire				extend_flush;
wire				ex_branch_taken;
wire				flag;
//wire     			flagc;		
wire				flagforw;
wire				flag_we;
wire				flagforw_alu;   
wire				flag_we_alu;
wire				flagforw_aluc;   
wire				flag_we_aluc;
wire				flagforw_fpu;
wire				flag_we_fpu;
wire				carry;
//wire				carryc;
wire				cyforw;
wire				cy_we_alu;
wire				cyforwa;
wire				cy_we_alua;
wire				cyforwc;
wire				cy_we_aluc;   
wire				ovforw;
wire				ov_we_alu;
wire				ovforwa;
wire				ov_we_alua;
wire				ovforwc;
wire				ov_we_aluc;   
wire				ovforw_mult_mac;
wire				ov_we_mult_mac;   
wire				cy_we_rf;
wire				lsu_stall;
wire				epcr_we;
wire				eear_we;
wire				esr_we;
wire				pc_we;
wire	[31:0]			epcr;
wire	[31:0]			eear;
wire	[`OR1200_SR_WIDTH-1:0]	esr;
wire 	[`OR1200_FPCSR_WIDTH-1:0]       fpcsr;
wire 				fpcsr_we;   
wire				sr_we;
wire	[`OR1200_SR_WIDTH-1:0]	to_sr;
wire	[`OR1200_SR_WIDTH-1:0]	sr;
wire    			dsx;
wire				except_flushpipe;
wire				except_start;
wire				except_started;
wire    			fpu_except_started;   
wire	[31:0]			wb_insn;
wire				sig_syscall;
wire				sig_trap;
wire    			sig_range;
wire				sig_fp;
wire	[31:0]			spr_dat_cfgr;
wire	[31:0]			spr_dat_rf;
wire    [31:0]                  spr_dat_npc;
wire	[31:0]			spr_dat_ppc;
wire	[31:0]			spr_dat_mac;
wire [31:0] 			spr_dat_fpu;
wire     			mtspr_done;
wire				force_dslot_fetch;
wire				no_more_dslot;
wire				ex_void;
wire				ex_spr_read;
wire				ex_spr_write;
wire				if_stall;
wire				id_macrc_op;
wire				ex_macrc_op;
wire	[`OR1200_MACOP_WIDTH-1:0] mac_op;
wire	[31:0]			mult_mac_result;
wire				mult_mac_stall;
wire	[13:0]			except_trig;
wire	[13:0]			except_stop;
wire				genpc_refetch;
wire				rfe;
wire				lsu_unstall;
wire				except_align;
wire				except_dtlbmiss;
wire				except_dmmufault;
wire				except_illegala;
wire				except_illegalc;
wire				except_itlbmiss;
wire				except_immufault;
wire				except_ibuserr;
wire				except_dbuserr;
wire				abort_ex;
wire				abort_mvspr;
wire     			ex_two_insns;
wire     			ex_two_insns_next;
wire     			if_two_insns;   
wire    			flagforwa;
wire    			dependency_hazard_stall;
   wire 			half_insn_done;
   wire 			same_stage_dslot;
	
   //Used for or1200-monitor
   wire 	flag1;
   wire 	over1;
   wire 	carry1;
   reg 	flag1_next;
   reg 	over1_next;
   reg 	carry1_next;  
   wire  flag_branch;
   
//
// Send exceptions to Debug Unit
//
assign du_except_trig = except_trig;
assign du_except_stop = except_stop;
assign du_lsu_store_dat = operand_b;
assign du_lsu_load_dat  = lsu_dataout;
   
//
// Data cache enable
//
`ifdef OR1200_NO_DC
assign dc_en = 1'b0;
`else
   assign dc_en = sr[`OR1200_SR_DCE];
`endif

//
// Instruction cache enable
//
`ifdef OR1200_NO_IC
assign ic_en = 1'b0;
`else
   assign ic_en = sr[`OR1200_SR_ICE];
`endif

//
// SB enable
//
`ifdef OR1200_SB_IMPLEMENTED
//assign sb_en = sr[`OR1200_SR_SBE]; // SBE not defined  -- jb
`else
assign sb_en = 1'b0;
`endif

//
// DMMU enable
//
`ifdef OR1200_NO_DMMU
assign dmmu_en = 1'b0;
`else
   assign dmmu_en = sr[`OR1200_SR_DME];
`endif

//
// IMMU enable
//
`ifdef OR1200_NO_IMMU
assign immu_en = 1'b0;
`else
   assign immu_en = sr[`OR1200_SR_IME] & ~except_started;
`endif

//
// SUPV bit
//
assign supv = sr[`OR1200_SR_SM];


//
// Flag for any MTSPR instructions, that must block execution, to indicate done
//
assign mtspr_done = mtspr_dc_done;

//
// Range exception
//
assign sig_range = sr[`OR1200_SR_OV];
   
   
   
//
// Instantiation of instruction fetch block
//
// Mux to determine which operand is the branch instruction
assign operand_branch = operand_b;
assign flag_branch = flag;
   
or1200_genpc #(.boot_adr(boot_adr)) or1200_genpc(
	.clk(clk),
	.rst(rst),
	.icpu_adr_o(icpu_adr_o),
	.icpu_cycstb_o(icpu_cycstb_o),
	.icpu_sel_o(icpu_sel_o),
	.icpu_tag_o(icpu_tag_o),
	.icpu_rty_i(icpu_rty_i),
	.icpu_adr_i(icpu_adr_i),
	.pre_branch_op(pre_branch_op),
	.branch_op(branch_op),
	.except_type(except_type),
	.except_start(except_start),
	.except_prefix(sr[`OR1200_SR_EPH]),
	.ex_branch_addrtarget(ex_branch_addrtarget),
	.muxed_b(muxed_b),  //not used
	.operand_b(operand_branch),
	.flag(flag_branch),
	.flagforw(flagforwa),
	.ex_branch_taken(ex_branch_taken),
	.epcr(epcr),
	.spr_dat_i(spr_dat_cpu),
	.spr_pc_we(pc_we),
	.genpc_refetch(genpc_refetch),
	.genpc_freeze(genpc_freeze),
	.no_more_dslot(no_more_dslot),
	.lsu_stall(lsu_stall),
	.ex_two_insns(ex_two_insns),
	.if_two_insns(if_two_insns),
	.dependency_hazard_stall(dependency_hazard_stall)		 
);

//
// Instantiation of instruction fetch block
//
or1200_if or1200_if(
	.clk(clk),
	.rst(rst),
	.icpu_dat_i(icpu_dat_i),
	.icpu_ack_i(icpu_ack_i),
	.icpu_err_i(icpu_err_i),
	.icpu_adr_i(icpu_adr_i),
	.icpu_tag_i(icpu_tag_i),
	.if_freeze(if_freeze),
	.if_insn(if_insn),
	.if_pc(if_pc),
	.saving_if_insn(saving_if_insn),
	.if_flushpipe(if_flushpipe),
	.if_stall(if_stall),
	.no_more_dslot(no_more_dslot),
	.genpc_refetch(genpc_refetch),
	.rfe(rfe),
	.except_itlbmiss(except_itlbmiss),
	.except_immufault(except_immufault),
	.except_ibuserr(except_ibuserr),
	.dependency_hazard_stall(dependency_hazard_stall),
	.same_stage_dslot(same_stage_dslot),
	.if_two_insns_ic(if_two_insns_ic),
	.if_two_insns(if_two_insns)
);
//
// Instantiation of instruction decode/control logic
//
or1200_ctrl or1200_ctrl(
	.clk(clk),
	.rst(rst),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.wb_freeze(wb_freeze),
	.if_flushpipe(if_flushpipe),
	.id_flushpipe(id_flushpipe),
	.ex_flushpipe(ex_flushpipe),
	.wb_flushpipe(wb_flushpipe),
	.extend_flush(extend_flush),
	.except_flushpipe(except_flushpipe),
	.abort_mvspr(abort_mvspr),
	.if_insn(if_insn),
	.id_insn(id_insn),
	.ex_insn(ex_insn),
	.id_branch_op(pre_branch_op),
	.ex_branch_op(branch_op),
	.ex_branch_taken(ex_branch_taken),
	.rf_addra(rf_addra),
	.rf_addrb(rf_addrb),
	.rf_addrc(rf_addrc),
	.rf_addrd(rf_addrd),
	.rf_rda(rf_rda),
	.rf_rdb(rf_rdb),
	.rf_rdc(rf_rdc),
	.rf_rdd(rf_rdd),
	.alu_op(alu_op),
	.alu_op2(alu_op2),
	.alu_opc_out(alu_opc),
	.alu_op2c(alu_op2c),			
	.mac_op(mac_op),
	.comp_op(comp_op),
	.comp_opc(comp_opc),
	.rf_addrw(rf_addrw),
	.rf_addrw2(rf_addrwc),
	.rfwb_op(rfwb_op),
	.rfwb_op2(rfwb_opc),
	.fpu_op(fpu_op),			
	.pc_we(pc_we),
	.wb_insn(wb_insn),
	.id_simma(id_simma),
	.id_simmc(id_simmc),
	.ex_branch_addrtarget(ex_branch_addrtarget),
	.ex_simm(ex_simm),
	.ex_two_insns(ex_two_insns),
	.ex_two_insns_next(ex_two_insns_next),
        //.if_two_insns(if_two_insns),
        .dependency_hazard_stall(dependency_hazard_stall),
	.abort_ex(abort_ex),
	.sel_a(sel_a),
	.sel_b(sel_b),
	.sel_c(sel_c),
	.sel_d(sel_d),
	.id_lsu_op(id_lsu_op),
	.cust5_op(cust5_op),
	.cust5_limm(cust5_limm),
	.cust5_opc(cust5_opc),
	.cust5_limmc(cust5_limmc),
	.id_pc(id_pc),
	.multicycle(multicycle),
        .wait_on(wait_on),			
	.wbforw_valid(wbforw_valid),
	.wbforw_valid2(wbforw_validc),		
	.sig_syscall(sig_syscall),
	.sig_trap(sig_trap),
	.force_dslot_fetch(force_dslot_fetch),
	.no_more_dslot(no_more_dslot),
	.id_void(id_void),
	.ex_void(ex_void),
	.ex_spr_read(ex_spr_read),
	.ex_spr_write(ex_spr_write),
	.id_macrc_op(id_macrc_op),
	.ex_macrc_op(ex_macrc_op),
	.rfe(rfe),
	.du_hwbkpt(du_hwbkpt),
	.except_illegala(except_illegala),
	.except_illegalc(except_illegalc),		
	.dc_no_writethrough(dc_no_writethrough),
        .half_insn_done(half_insn_done),
	.same_stage_dslot(same_stage_dslot)
);

//
// Instantiation of register file
//
or1200_rf or1200_rf(
	.clk(clk),
	.rst(rst),
	.cy_we_i(cy_we_alu),
	.cy_we_o(cy_we_rf),
	.supv(sr[`OR1200_SR_SM]),
	.wb_freeze(wb_freeze),
	.addrw(rf_addrw),
	.addrw2(rf_addrwc),
	.dataw(rf_dataw),
	.dataw2(rf_datawc),
	.id_freeze(id_freeze),
	.we(rfwb_op[0]),
	.we2(rfwb_opc[0]),
	.flushpipe(wb_flushpipe),
	.addra(rf_addra),
	.rda(rf_rda),
	.dataa(rf_dataa),
	.addrb(rf_addrb),
	.rdb(rf_rdb),
	.datab(rf_datab),
	.datac(rf_datac),
	.rdc(rf_rdc),
	.addrc(rf_addrc),
	.datad(rf_datad),
	.rdd(rf_rdd),
	.addrd(rf_addrd),
	.spr_cs(spr_cs[`OR1200_SPR_GROUP_SYS]),
	.spr_write(spr_we),
	.spr_addr(spr_addr),
	.spr_dat_i(spr_dat_cpu),
	.spr_dat_o(spr_dat_rf),
	.du_read(du_read),
	.half_insn_done_i(half_insn_done)
);

or1200_operandmuxes or1200_operandmuxes1(
	.clk(clk),
	.rst(rst),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.rf_dataa(rf_dataa_inter),
	.rf_datab(rf_datab_inter),
	.ex_forw(rf_dataw),
	.ex_forw2(rf_datawc),
	.wb_forw(wb_forw),
	.wb_forw2(wb_forwc),
	.simm(id_simma),
	.sel_a(sel_a),
	.sel_b(sel_b),
	.operand_a(operand_a), //synchronous
	.operand_b(operand_b), //sync
	.muxed_a(muxed_a),
	.muxed_b(muxed_b)
);

// id stage
always @(*) begin
   if (!half_insn_done) begin
      rf_dataa_inter <= rf_dataa;
      rf_datab_inter <= rf_datab;
   end
   else begin
      rf_dataa_inter <= operand_c;
      rf_datab_inter <= operand_d;
   end
end
   
or1200_operandmuxes or1200_operandmuxes2(
	.clk(clk),
	.rst(rst),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.rf_dataa(rf_datac),
	.rf_datab(rf_datad),
	.ex_forw(rf_dataw),
	.ex_forw2(rf_datawc),
	.wb_forw(wb_forw),
	.wb_forw2(wb_forwc),
	.simm(id_simmc),
	.sel_a(sel_c),
	.sel_b(sel_d),
	.operand_a(operand_c),
	.operand_b(operand_d),
	.muxed_a(muxed_c),
	.muxed_b(muxed_d)					 
);

//
// Instantiation of CPU's ALU - two for two insns
//
or1200_alu or1200_alu(
	.a(operand_a),
	.b(operand_b),
	.mult_mac_result(mult_mac_result),
	.macrc_op(ex_macrc_op),
	.alu_op(alu_op),
	.alu_op2(alu_op2),		      
 	.comp_op(comp_op),
	.cust5_op(cust5_op),
	.cust5_limm(cust5_limm),
	.result(alu_dataout),
	.flagforw(flagforw_alu),
	.flag_we(flag_we_alu),
	.cyforw(cyforwa),
	.cy_we(cy_we_alua),
	.ovforw(ovforwa),
	.ov_we(ov_we_alua),		      
	.flag(flag),
	.carry(carry)
);

//
// FLAG write enable
//
//This is needed for a branch which should not be affected by second instruction
assign flagforwa = (flag_we_alu & flagforw_alu) | (flagforw_fpu & flag_we_fpu);

//second write dominates if second ALU instruction is valid
assign flagforw = flag_we_aluc ? flagforw_aluc : (flag_we_alu & flagforw_alu) | (flagforw_fpu & flag_we_fpu);  
assign flag_we = !ex_freeze & (flag_we_alu | flag_we_fpu | flag_we_aluc) & ~abort_mvspr; //ex_freeze added to keep from failing if stall in cycle with branch in first slot and an instruction that changed the flag in the second slot
//this would cause the instruction following a branch that depended on a flag to change the status of the flag

assign ovforw = ov_we_aluc ? ovforwc : ovforwa;
assign ov_we_alu = ov_we_alua | ov_we_aluc;
assign cyforw = cy_we_aluc ? cyforwc : cyforwa;
assign cy_we_alu = cy_we_alua | cy_we_aluc;   
   
//This is needed from the previous instruction   
//assign flagc = flag_we_alu ? flagforw_alu : flag;
//assign carryc = cy_we_alua ? cyforw : carry;  
   
//Used for or1200-monitor
   assign flag1 = (flag_we_alu & flagforw_alu) | (flagforw_fpu & flag_we_fpu);
   assign over1 = ov_we_alua & ovforwa;
   assign carry1 = cy_we_alua & cyforwa;

   //Used for or1200-monitor
always @(posedge clk) begin
   if (!ex_freeze) begin
      flag1_next <= flag1;
      over1_next <= over1;
      carry1_next <= carry;
   end
end
   

or1200_alu or1200_alu2(
	.a(operand_c),
	.b(operand_d),
	.mult_mac_result(mult_mac_result),
	.macrc_op(ex_macrc_op),
	.alu_op(alu_opc),
	.alu_op2(alu_op2c),		      
 	.comp_op(comp_opc),
	.cust5_op(cust5_opc),
	.cust5_limm(cust5_limmc),
	.result(/*alu_dataoutc*/ rf_datawc),
	.flagforw(flagforw_aluc),
	.flag_we(flag_we_aluc),
	.cyforw(cyforwc),
	.cy_we(cy_we_aluc),
	.ovforw(ovforwc),
	.ov_we(ov_we_aluc)
	//.flag(1'b0), `OR1200_ALUOP_CMOV should stall pipeline
	//.carry(1'b0) `OR1200_ALUOP_ADDC should stall pipeline
);

   
//
// FPU's exception is being dealt with
//    
assign fpu_except_started = except_started && (except_type == `OR1200_EXCEPT_FLOAT);
   
//
// Instantiation of FPU
//
or1200_fpu or1200_fpu(
	.clk(clk),
	.rst(rst),
	.ex_freeze(ex_freeze),
	.a(operand_a),
	.b(operand_b),
	.fpu_op(fpu_op),
	.result(fpu_dataout),
	.done(fpu_done),
	.flagforw(flagforw_fpu),
	.flag_we(flag_we_fpu),
        .sig_fp(sig_fp),
	.except_started(fpu_except_started),
	.fpcsr_we(fpcsr_we),
	.fpcsr(fpcsr),		      
	.spr_cs(spr_cs[`OR1200_SPR_GROUP_FPU]),
	.spr_write(spr_we),
	.spr_addr(spr_addr),
	.spr_dat_i(spr_dat_cpu),
	.spr_dat_o(spr_dat_fpu)
);

   
//
// Instantiation of CPU's multiply unit
//
or1200_mult_mac or1200_mult_mac(
	.clk(clk),
	.rst(rst),
	.ex_freeze(ex_freeze),
	.id_macrc_op(id_macrc_op),
	.macrc_op(ex_macrc_op),
	.a(operand_a),
	.b(operand_b),
	.mac_op(mac_op),
	.alu_op(alu_op),
	.result(mult_mac_result),
	.ovforw(ovforw_mult_mac), 
	.ov_we(ov_we_mult_mac),
	.mult_mac_stall(mult_mac_stall),
	.spr_cs(spr_cs[`OR1200_SPR_GROUP_MAC]),
	.spr_write(spr_we),
	.spr_addr(spr_addr),
	.spr_dat_i(spr_dat_cpu),
	.spr_dat_o(spr_dat_mac)
);

//
// Instantiation of CPU's SPRS block
//
or1200_sprs or1200_sprs(
	.clk(clk),
	.rst(rst),
	.addrbase(operand_a),
	.addrofs(ex_simm[15:0]),
	.dat_i(operand_b),
	.ex_spr_read(ex_spr_read),
	.ex_spr_write(ex_spr_write),
	.flagforw(flagforw),
	.flag_we(flag_we),
	.flag(flag),
	.cyforw(cyforw),
	.cy_we(cy_we_rf),
	.carry(carry),
	.ovforw(ovforw | ovforw_mult_mac),
	.ov_we(ov_we_alu | ov_we_mult_mac),
	.to_wbmux(sprs_dataout),

	.du_addr(du_addr),
	.du_dat_du(du_dat_du),
	.du_read(du_read),
	.du_write(du_write),
	.du_dat_cpu(du_dat_cpu),
	.boot_adr_sel_i(boot_adr_sel_i),
	.spr_addr(spr_addr),
	.spr_dat_pic(spr_dat_pic),
	.spr_dat_tt(spr_dat_tt),
	.spr_dat_pm(spr_dat_pm),
	.spr_dat_cfgr(spr_dat_cfgr),
	.spr_dat_rf(spr_dat_rf),
	.spr_dat_npc(spr_dat_npc),
        .spr_dat_ppc(spr_dat_ppc),
	.spr_dat_mac(spr_dat_mac),
	.spr_dat_dmmu(spr_dat_dmmu),
	.spr_dat_immu(spr_dat_immu),
	.spr_dat_du(spr_dat_du),
	.spr_dat_o(spr_dat_cpu),
	.spr_cs(spr_cs),
	.spr_we(spr_we),

	.epcr_we(epcr_we),
	.eear_we(eear_we),
	.esr_we(esr_we),
	.pc_we(pc_we),
	.epcr(epcr),
	.eear(eear),
	.esr(esr),
	.except_started(except_started),

	.fpcsr(fpcsr),
	.fpcsr_we(fpcsr_we),			
	.spr_dat_fpu(spr_dat_fpu),
			
	.sr_we(sr_we),
	.to_sr(to_sr),
	.sr(sr),
	.branch_op(branch_op),
	.dsx(dsx)
			
);
   
//
// Instantiation of load/store unit
//
or1200_lsu or1200_lsu(
	.clk(clk),
	.rst(rst),
	.id_addrbase(muxed_a),
	.id_addrofs(id_simma),
	.ex_addrbase(operand_a),
	.ex_addrofs(ex_simm),
	.id_lsu_op(id_lsu_op),
	.lsu_datain(operand_b),
	.lsu_dataout(lsu_dataout),
	.lsu_stall(lsu_stall),
	.lsu_unstall(lsu_unstall),
	.du_stall(du_stall),
	.except_align(except_align),
	.except_dtlbmiss(except_dtlbmiss),
	.except_dmmufault(except_dmmufault),
	.except_dbuserr(except_dbuserr),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.flushpipe(ex_flushpipe),

	.dcpu_adr_o(dcpu_adr_o),
	.dcpu_cycstb_o(dcpu_cycstb_o),
	.dcpu_we_o(dcpu_we_o),
	.dcpu_sel_o(dcpu_sel_o),
	.dcpu_tag_o(dcpu_tag_o),
	.dcpu_dat_o(dcpu_dat_o),
	.dcpu_dat_i(dcpu_dat_i),
	.dcpu_ack_i(dcpu_ack_i),
	.dcpu_rty_i(dcpu_rty_i),
	.dcpu_err_i(dcpu_err_i),
	.dcpu_tag_i(dcpu_tag_i)
);

//
// Instantiation of write-back muxes
//
or1200_wbmux or1200_wbmux(
	.clk(clk),
	.rst(rst),
	.wb_freeze(wb_freeze),
	.rfwb_op(rfwb_op),
	.muxin_a(alu_dataout),
	.muxin_b(lsu_dataout),
	.muxin_c(sprs_dataout),
	.muxin_d(ex_pc),
        .muxin_e(fpu_dataout),
	.muxout(rf_dataw),
	.muxreg(wb_forw),
	.muxreg_valid(wbforw_valid)
);

// Do not really need a 2nd write-back mux since only one possible input
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE) begin
	   wb_forwc <=  32'd0;
	   wbforw_validc <=  1'b0;
	end
	else if (!wb_freeze) begin
	   wb_forwc <=  rf_datawc;
	   wbforw_validc <=  rfwb_opc[0];
	end
end
   
/*or1200_wbmux or1200_wbmux2(
	.clk(clk),
	.rst(rst),
	.wb_freeze(wb_freeze),
	.rfwb_op(rfwb_opc),
	.muxin_a(alu_dataoutc),
	.muxin_b(lsu_dataout),
	.muxin_c(sprs_dataout),
	.muxin_d((ex_pc + 32'h4)),
        .muxin_e(fpu_dataout),
	.muxout(rf_datawc),
	.muxreg(wb_forwc),
	.muxreg_valid(wbforw_validc)
);*/   
//
// Instantiation of freeze logic
//
or1200_freeze or1200_freeze(
	.clk(clk),
	.rst(rst),
	.multicycle(multicycle),
        .wait_on(wait_on),
	.fpu_done(fpu_done),
	.mtspr_done(mtspr_done),
	.flushpipe(wb_flushpipe),
	.extend_flush(extend_flush),
	.lsu_stall(lsu_stall),
	.if_stall(if_stall),
	.lsu_unstall(lsu_unstall),
	.force_dslot_fetch(force_dslot_fetch),
	.abort_ex(abort_ex),
	.du_stall(du_stall),
	.mac_stall(mult_mac_stall),
	.saving_if_insn(saving_if_insn),
	.genpc_freeze(genpc_freeze),
	.if_freeze(if_freeze),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.wb_freeze(wb_freeze),
	.icpu_ack_i(icpu_ack_i),
	.icpu_err_i(icpu_err_i),
	.half_insn_done(half_insn_done)		    
);

//
// Instantiation of exception block
//
or1200_except or1200_except(
	.clk(clk),
	.rst(rst),
	.sig_ibuserr(except_ibuserr),
	.sig_dbuserr(except_dbuserr),
	.sig_illegala(except_illegala),
	.sig_illegalc(except_illegalc),
        .sig_align(except_align),
	.sig_range(sig_range),
	.sig_dtlbmiss(except_dtlbmiss),
	.sig_dmmufault(except_dmmufault),
	.sig_int(sig_int),
	.sig_syscall(sig_syscall),
	.sig_trap(sig_trap),
	.sig_itlbmiss(except_itlbmiss),
	.sig_immufault(except_immufault),
	.sig_tick(sig_tick),
	.sig_fp(sig_fp),
	.fpcsr_fpee(fpcsr[`OR1200_FPCSR_FPEE]),
	.ex_branch_taken(ex_branch_taken),
	.icpu_ack_i(icpu_ack_i),
	.icpu_err_i(icpu_err_i),
	.dcpu_ack_i(dcpu_ack_i),
	.dcpu_err_i(dcpu_err_i),
	.genpc_freeze(genpc_freeze),
        .id_freeze(id_freeze),
        .ex_freeze(ex_freeze),
        .wb_freeze(wb_freeze),
	.if_stall(if_stall),
	.if_pc(if_pc),
	.id_pc(id_pc),
	.ex_pc(ex_pc),
	.wb_pc(wb_pc),
	.id_flushpipe(id_flushpipe),
	.ex_flushpipe(ex_flushpipe),
	.extend_flush(extend_flush),
	.except_flushpipe(except_flushpipe),
	.abort_mvspr(abort_mvspr),
	.except_type(except_type),
	.except_start(except_start),
	.except_started(except_started),
	.except_stop(except_stop),
	.except_trig(except_trig),
	.ex_void(ex_void),
	.spr_dat_ppc(spr_dat_ppc),
	.spr_dat_npc(spr_dat_npc),
	.datain(spr_dat_cpu),
	.branch_op(branch_op),
	.du_dsr(du_dsr),
	.du_dmr1(du_dmr1),
	.du_hwbkpt(du_hwbkpt),
	.du_hwbkpt_ls_r(du_hwbkpt_ls_r),
	.epcr_we(epcr_we),
	.eear_we(eear_we),
	.esr_we(esr_we),
	.pc_we(pc_we),
        .epcr(epcr),
	.eear(eear),
	.esr(esr),
	.lsu_addr(dcpu_adr_o),
	.sr_we(sr_we),
	.to_sr(to_sr),
	.sr(sr),
	.abort_ex(abort_ex),
	.dsx(dsx),
	.ex_two_insns(ex_two_insns),
	.ex_two_insns_next(ex_two_insns_next),
	.half_insn_done(half_insn_done),
	.same_stage_dslot(same_stage_dslot),
	.dependency_hazard_stall(dependency_hazard_stall)
);

//
// Instantiation of configuration registers
//
or1200_cfgr or1200_cfgr(
	.spr_addr(spr_addr),
	.spr_dat_o(spr_dat_cfgr)
);

endmodule
