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
//This also does the sign extending during the id stage


`include "timescale.v"
`include "or1200_defines.v"

module or1200_ctrl_id_decode
  (
   clk, rst, id_insn, ex_freeze, id_freeze, ex_flushpipe, id_pc, du_hwbkpt, abort_mvspr, sel_imm, rf_addrw1, rf_addrw2, rfwb_op1, rfwb_op2, wb_rfaddrw1, wbforw_valid1, wb_rfaddrw2, wbforw_valid2, id_branch_op, id_simm, id_macrc_op, ex_macrc_op, sig_syscall, dc_no_writethrough, id_void, fpu_op, multicycle, wait_on, rf_addrw, except_illegal, alu_op, alu_op2, spr_read, spr_write, mac_op, rfwb_op, id_lsu_op, comp_op, sig_trap, sel_a, sel_b, ex_branch_op, ex_simm, ex_branch_addrtarget, id_illegal
   );

   input					clk;
   input					rst;
   input [31:0] 				id_insn;
   input 					ex_freeze;
   input 					id_freeze;		
   input 					ex_flushpipe;		
   input [31:0] 				id_pc;			
   input					du_hwbkpt;
   input 					abort_mvspr;		
   input 					sel_imm;
   input [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrw1;
   input [`OR1200_RFWBOP_WIDTH-1:0]		rfwb_op1;
   input [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrw2;
   input [`OR1200_RFWBOP_WIDTH-1:0]		rfwb_op2;
   input [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	wb_rfaddrw1;
   input 					wbforw_valid1;
   input [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	wb_rfaddrw2;
   input 					wbforw_valid2;
   input [`OR1200_BRANCHOP_WIDTH-1:0] 		id_branch_op;
   output [31:0] 				id_simm;
   output 					id_macrc_op;
   output 					ex_macrc_op;
   output 					sig_syscall;
   output 					dc_no_writethrough;
   output 					id_void;		
   output [`OR1200_FPUOP_WIDTH-1:0] 		fpu_op;
   output [`OR1200_MULTICYCLE_WIDTH-1:0] 	multicycle;
   output [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_on; 
   output [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrw;  
   output 					except_illegal;
   output [`OR1200_ALUOP_WIDTH-1:0] 		alu_op;
   output [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2;
   output 					spr_read;
   output 					spr_write;
   output [`OR1200_MACOP_WIDTH-1:0] 		mac_op;
   output [`OR1200_RFWBOP_WIDTH-1:0] 		rfwb_op;
   output [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_op;
   output [`OR1200_COMPOP_WIDTH-1:0] 		comp_op;
   output 					sig_trap;
   output [`OR1200_SEL_WIDTH:0] 		sel_a; //modified to add one bit to have more options
   output [`OR1200_SEL_WIDTH:0] 		sel_b;
   output [`OR1200_BRANCHOP_WIDTH-1:0] 		ex_branch_op;
   output [31:0] 				ex_simm;
   output [31:2] 				ex_branch_addrtarget;
   output 					id_illegal;
   wire [31:2] 					id_branch_addrtarget;
   reg [31:0] 					id_simm;
   reg 						sig_syscall;
`ifdef OR1200_DC_NOSTACKWRITETHROUGH
   reg 						dc_no_writethrough;
`endif   
   reg [`OR1200_MULTICYCLE_WIDTH-1:0] 		multicycle;
   reg [`OR1200_WAIT_ON_WIDTH-1:0] 		wait_on;   
   reg [`OR1200_REGFILE_ADDR_WIDTH-1:0] 	rf_addrw;
   reg 						except_illegal;
   reg [`OR1200_ALUOP_WIDTH-1:0] 		alu_op;
   reg [`OR1200_ALUOP2_WIDTH-1:0] 		alu_op2;
   reg 						spr_read;
   reg 						spr_write;
   reg [`OR1200_MACOP_WIDTH-1:0] 		id_mac_op;		
   reg [`OR1200_RFWBOP_WIDTH-1:0] 		rfwb_op;
   reg [`OR1200_LSUOP_WIDTH-1:0] 		id_lsu_op;
   reg [`OR1200_COMPOP_WIDTH-1:0] 		comp_op;
   reg 						sig_trap;
   reg [`OR1200_MACOP_WIDTH-1:0] 		ex_mac_op;
   reg 						ex_macrc_op;
   reg [`OR1200_SEL_WIDTH:0] 			sel_a;
   reg [`OR1200_SEL_WIDTH:0] 			sel_b;
   reg [`OR1200_BRANCHOP_WIDTH-1:0] 		ex_branch_op;
   reg [31:0] 					ex_simm;
   reg [31:2] 					ex_branch_addrtarget;
   reg 						id_illegal;
   
   
assign id_void = (id_insn[31:26] == `OR1200_OR32_NOP) & id_insn[16];
    
//
// ID Sign extension of branch offset
//
assign id_branch_addrtarget = {{4{id_insn[25]}}, id_insn[25:0]} + id_pc[31:2];

`ifdef OR1200_FPU_IMPLEMENTED
//
// Decode of FPU ops
//
   assign fpu_op = {(id_insn[31:26] == `OR1200_OR32_FLOAT), 
		    id_insn[`OR1200_FPUOP_WIDTH-2:0]};
`else
   assign fpu_op = {`OR1200_FPUOP_WIDTH{1'b0}};
`endif

   
//
// ID Sign/Zero extension of immediate
//
always @(id_insn) begin
	case (id_insn[31:26])     // synopsys parallel_case

	// l.addi
	`OR1200_OR32_ADDI:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

	// l.addic
	`OR1200_OR32_ADDIC:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

	// l.lxx (load instructions)
	`OR1200_OR32_LWZ, `OR1200_OR32_LWS,
   `OR1200_OR32_LBZ, `OR1200_OR32_LBS,
	`OR1200_OR32_LHZ, `OR1200_OR32_LHS:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

	// l.muli
	`ifdef OR1200_MULT_IMPLEMENTED
	`OR1200_OR32_MULI:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};
	`endif

	// l.maci
	`ifdef OR1200_MAC_IMPLEMENTED
	`OR1200_OR32_MACI:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};
	`endif

	// l.mtspr
	`OR1200_OR32_MTSPR:
		id_simm = {16'b0, id_insn[25:21], id_insn[10:0]};

	// l.sxx (store instructions)
	`OR1200_OR32_SW, `OR1200_OR32_SH, `OR1200_OR32_SB:
		id_simm = {{16{id_insn[25]}}, id_insn[25:21], id_insn[10:0]};

	// l.xori
	`OR1200_OR32_XORI:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

	// l.sfxxi (SFXX with immediate)
	`OR1200_OR32_SFXXI:
		id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

	// Instructions with no or zero extended immediate
	default:
		id_simm = {{16'b0}, id_insn[15:0]};

	endcase
end

//
// l.macrc in ID stage
//
`ifdef OR1200_MAC_IMPLEMENTED
assign id_macrc_op = (id_insn[31:26] == `OR1200_OR32_MACRC) & id_insn[16];
`else
assign id_macrc_op = 1'b0;
`endif

   
//
// Decode of l.sys
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		sig_syscall <=  1'b0;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		sig_syscall <=  1'b0;
	else if (!ex_freeze) begin
`ifdef OR1200_VERBOSE
// synopsys translate_off
		if (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b000})
			$display("Generating sig_syscall");
// synopsys translate_on
`endif
		sig_syscall <=  (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b000});
	end
end
// Decode destination register address for data cache to check if store ops
// are being done from the stack register (r1) or frame pointer register (r2)
`ifdef OR1200_DC_NOSTACKWRITETHROUGH   
always @(posedge clk or `OR1200_RST_EVENT rst) begin
   if (rst == `OR1200_RST_VALUE)
     dc_no_writethrough <= 0;
   else if (!ex_freeze)
     dc_no_writethrough <= (id_insn[20:16] == 5'd1) | (id_insn[20:16] == 5'd2);
end
`else
   
   assign dc_no_writethrough = 0;
  
`endif      


//
// Decode of multicycle
//
always @(id_insn) begin
  case (id_insn[31:26])		// synopsys parallel_case
    // l.rfe
    `OR1200_OR32_RFE,
    // l.mfspr
    `OR1200_OR32_MFSPR:
      multicycle = `OR1200_TWO_CYCLES;	// to read from ITLB/DTLB (sync RAMs)
    // Single cycle instructions
    default: begin
      multicycle = `OR1200_ONE_CYCLE;
    end    
  endcase
end // always @ (id_insn)

//
// Encode wait_on signal
//    
always @(id_insn) begin
   case (id_insn[31:26])		// synopsys parallel_case
     `OR1200_OR32_ALU: 
       wait_on =  ( 1'b0
`ifdef OR1200_DIV_IMPLEMENTED
                     | (id_insn[4:0] == `OR1200_ALUOP_DIV)
		     | (id_insn[4:0] == `OR1200_ALUOP_DIVU)
`endif
`ifdef OR1200_MULT_IMPLEMENTED
		     | (id_insn[4:0] == `OR1200_ALUOP_MUL)
		     | (id_insn[4:0] == `OR1200_ALUOP_MULU)
`endif
		    ) ? `OR1200_WAIT_ON_MULTMAC : `OR1200_WAIT_ON_NOTHING;
`ifdef OR1200_MULT_IMPLEMENTED
`ifdef OR1200_MAC_IMPLEMENTED
     `OR1200_OR32_MACMSB,
     `OR1200_OR32_MACI,
`endif
     `OR1200_OR32_MULI:       
	 wait_on = `OR1200_WAIT_ON_MULTMAC;
`endif
`ifdef OR1200_MAC_IMPLEMENTED
     `OR1200_OR32_MACRC:
         wait_on = id_insn[16] ? `OR1200_WAIT_ON_MULTMAC : 
		                 `OR1200_WAIT_ON_NOTHING;
`endif		   
`ifdef OR1200_FPU_IMPLEMENTED
       `OR1200_OR32_FLOAT: begin
	 wait_on = id_insn[`OR1200_FPUOP_DOUBLE_BIT] ? 2'b0 : `OR1200_WAIT_ON_FPU;
       end
`endif
`ifndef OR1200_DC_WRITEHROUGH
     // l.mtspr
     `OR1200_OR32_MTSPR: begin
	wait_on = `OR1200_WAIT_ON_MTSPR;
     end
`endif
     default: begin
	wait_on = `OR1200_WAIT_ON_NOTHING;
     end
   endcase // case (id_insn[31:26])
end // always @ (id_insn)

//
// Register file write address
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		rf_addrw <=  5'd0;
	else if (!ex_freeze & id_freeze)
		rf_addrw <=  5'd00;
	else if (!ex_freeze)
		case (id_insn[31:26])	// synopsys parallel_case
			`OR1200_OR32_JAL, `OR1200_OR32_JALR:
				rf_addrw <=  5'd09;	// link register r9
			default:
				rf_addrw <=  id_insn[25:21];
		endcase
end

   
//
// Decode of except_illegal
//
//id_illegal needed because of the possibility that the first instruction is legal and the second instruction is illegal
always @(id_insn) begin
   case (id_insn[31:26])		// synopsys parallel_case

		`OR1200_OR32_J,
		`OR1200_OR32_JAL,
		`OR1200_OR32_JALR,
		`OR1200_OR32_JR,
		`OR1200_OR32_BNF,
		`OR1200_OR32_BF,
		`OR1200_OR32_RFE,
		`OR1200_OR32_MOVHI,
		`OR1200_OR32_MFSPR,
		`OR1200_OR32_XSYNC,
`ifdef OR1200_MAC_IMPLEMENTED
		`OR1200_OR32_MACI,
`endif
		`OR1200_OR32_LWZ,
		`OR1200_OR32_LWS,
		`OR1200_OR32_LBZ,
		`OR1200_OR32_LBS,
		`OR1200_OR32_LHZ,
		`OR1200_OR32_LHS,
		`OR1200_OR32_ADDI,
		`OR1200_OR32_ADDIC,
		`OR1200_OR32_ANDI,
		`OR1200_OR32_ORI,
		`OR1200_OR32_XORI,
`ifdef OR1200_MULT_IMPLEMENTED
		`OR1200_OR32_MULI,
`endif
		`OR1200_OR32_SH_ROTI,
		`OR1200_OR32_SFXXI,
		`OR1200_OR32_MTSPR,
`ifdef OR1200_MAC_IMPLEMENTED
		`OR1200_OR32_MACMSB,
`endif
		`OR1200_OR32_SW,
		`OR1200_OR32_SB,
		`OR1200_OR32_SH,
		`OR1200_OR32_SFXX,
`ifdef OR1200_IMPL_ALU_CUST5
		`OR1200_OR32_CUST5,
`endif
	`OR1200_OR32_NOP:
		id_illegal <=  1'b0;
`ifdef OR1200_FPU_IMPLEMENTED
	    `OR1200_OR32_FLOAT:
                // Check it's not a double precision instruction
                id_illegal <=  id_insn[`OR1200_FPUOP_DOUBLE_BIT];
`endif	      

	`OR1200_OR32_ALU:
		id_illegal <=  1'b0 

`ifdef OR1200_MULT_IMPLEMENTED
`ifdef OR1200_DIV_IMPLEMENTED
`else 
		| (id_insn[4:0] == `OR1200_ALUOP_DIV)
		| (id_insn[4:0] == `OR1200_ALUOP_DIVU)
`endif
`else
		| (id_insn[4:0] == `OR1200_ALUOP_DIV)
		| (id_insn[4:0] == `OR1200_ALUOP_DIVU)
		| (id_insn[4:0] == `OR1200_ALUOP_MUL)
`endif

`ifdef OR1200_IMPL_ADDC
`else
		| (id_insn[4:0] == `OR1200_ALUOP_ADDC)
`endif

`ifdef OR1200_IMPL_ALU_FFL1
`else
		| (id_insn[4:0] == `OR1200_ALUOP_FFL1)
`endif

`ifdef OR1200_IMPL_ALU_ROTATE
`else
		| ((id_insn[4:0] == `OR1200_ALUOP_SHROT) &
		   (id_insn[9:6] == `OR1200_SHROTOP_ROR))
`endif

`ifdef OR1200_IMPL_SUB
`else
		| (id_insn[4:0] == `OR1200_ALUOP_SUB)
`endif
`ifdef OR1200_IMPL_ALU_EXT
`else
		| (id_insn[4:0] == `OR1200_ALUOP_EXTHB)
		| (id_insn[4:0] == `OR1200_ALUOP_EXTW)
`endif
		;

		// Illegal and OR1200 unsupported instructions
	default:
		id_illegal <=  1'b1;

   endcase
end
   

always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		except_illegal <=  1'b0;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		except_illegal <=  1'b0;
	else if (!ex_freeze) begin
	   except_illegal <= id_illegal;
	end // if (!ex_freeze)
end
   

//
// Decode of alu_op
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		alu_op <=  `OR1200_ALUOP_NOP;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		alu_op <=  `OR1200_ALUOP_NOP;
	else if (!ex_freeze) begin
	  case (id_insn[31:26])		// synopsys parallel_case
	    
	    // l.movhi
	    `OR1200_OR32_MOVHI:
	      alu_op <=  `OR1200_ALUOP_MOVHI;
	    
	    // l.addi
	    `OR1200_OR32_ADDI:
	      alu_op <=  `OR1200_ALUOP_ADD;
	    
	    // l.addic
	    `OR1200_OR32_ADDIC:
	      alu_op <=  `OR1200_ALUOP_ADDC;
	    
	    // l.andi
	    `OR1200_OR32_ANDI:
	      alu_op <=  `OR1200_ALUOP_AND;
	    
	    // l.ori
	    `OR1200_OR32_ORI:
	      alu_op <=  `OR1200_ALUOP_OR;
	    
	    // l.xori
	    `OR1200_OR32_XORI:
	      alu_op <=  `OR1200_ALUOP_XOR;
	    
	    // l.muli
`ifdef OR1200_MULT_IMPLEMENTED
	    `OR1200_OR32_MULI:
	      alu_op <=  `OR1200_ALUOP_MUL;
`endif
	    
	    // Shift and rotate insns with immediate
	    `OR1200_OR32_SH_ROTI:
	      alu_op <=  `OR1200_ALUOP_SHROT;
	    
	    // SFXX insns with immediate
	    `OR1200_OR32_SFXXI:
	      alu_op <=  `OR1200_ALUOP_COMP;
	    
	    // ALU instructions except the one with immediate
	    `OR1200_OR32_ALU:
	      alu_op <=  {1'b0,id_insn[3:0]};
	    
	    // SFXX instructions
	    `OR1200_OR32_SFXX:
	      alu_op <=  `OR1200_ALUOP_COMP;
`ifdef OR1200_IMPL_ALU_CUST5	    
	    // l.cust5
	    `OR1200_OR32_CUST5:
	      alu_op <=  `OR1200_ALUOP_CUST5;
`endif	    
	    // Default
	    default: begin
	      alu_op <=  `OR1200_ALUOP_NOP;
	    end
	      
	  endcase
	  
	end
end


//
// Decode of second ALU operation field [9:6]
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		alu_op2 <=  0;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
	        alu_op2 <= 0;
   	else if (!ex_freeze) begin
		alu_op2 <=  id_insn[`OR1200_ALUOP2_POS];
	end
end
   

//
// Decode of spr_read, spr_write
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE) begin
		spr_read <=  1'b0;
		spr_write <=  1'b0;
	end
	else if (!ex_freeze & id_freeze | ex_flushpipe) begin
		spr_read <=  1'b0;
		spr_write <=  1'b0;
	end
	else if (!ex_freeze) begin
		case (id_insn[31:26])     // synopsys parallel_case

		// l.mfspr
		`OR1200_OR32_MFSPR: begin
			spr_read <=  1'b1;
			spr_write <=  1'b0;
		end

		// l.mtspr
		`OR1200_OR32_MTSPR: begin
			spr_read <=  1'b0;
			spr_write <=  1'b1;
		end

		// Default
		default: begin
			spr_read <=  1'b0;
			spr_write <=  1'b0;
		end

		endcase
	end
end


//
// l.macrc in EX stage
//
`ifdef OR1200_MAC_IMPLEMENTED
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		ex_macrc_op <=  1'b0;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		ex_macrc_op <=  1'b0;
	else if (!ex_freeze)
		ex_macrc_op <=  id_macrc_op;
end
`else
assign ex_macrc_op = 1'b0;
`endif
   
//
// Decode of mac_op
//
`ifdef OR1200_MAC_IMPLEMENTED

always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		ex_mac_op <=  `OR1200_MACOP_NOP;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		ex_mac_op <=  `OR1200_MACOP_NOP;
	else if (!ex_freeze)
		ex_mac_op <=  id_mac_op;
end

assign mac_op = abort_mvspr ? `OR1200_MACOP_NOP : ex_mac_op;

always @(id_insn) begin
	case (id_insn[31:26])		// synopsys parallel_case

	// l.maci
	`OR1200_OR32_MACI:
		id_mac_op =  `OR1200_MACOP_MAC;

	// l.mac, l.msb
	`OR1200_OR32_MACMSB:
		id_mac_op =  id_insn[2:0];

	// Illegal and OR1200 unsupported instructions
	default:
		id_mac_op =  `OR1200_MACOP_NOP;

	endcase
end

`else
assign id_mac_op = `OR1200_MACOP_NOP;
assign mac_op = `OR1200_MACOP_NOP;
`endif

//
// Decode of rfwb_op
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		rfwb_op <=  `OR1200_RFWBOP_NOP;
	else  if (!ex_freeze & id_freeze | ex_flushpipe)
		rfwb_op <=  `OR1200_RFWBOP_NOP;
	else  if (!ex_freeze) begin
		case (id_insn[31:26])		// synopsys parallel_case

		// j.jal
		`OR1200_OR32_JAL:
			rfwb_op <=  {`OR1200_RFWBOP_LR, 1'b1};
		  
		// j.jalr
		`OR1200_OR32_JALR:
			rfwb_op <=  {`OR1200_RFWBOP_LR, 1'b1};
		  
		// l.movhi
		`OR1200_OR32_MOVHI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.mfspr
		`OR1200_OR32_MFSPR:
			rfwb_op <=  {`OR1200_RFWBOP_SPRS, 1'b1};
		  
		// l.lwz
		`OR1200_OR32_LWZ:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

		// l.lws
		`OR1200_OR32_LWS:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

		// l.lbz
		`OR1200_OR32_LBZ:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};
		  
		// l.lbs
		`OR1200_OR32_LBS:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};
		  
		// l.lhz
		`OR1200_OR32_LHZ:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};
		  
		// l.lhs
		`OR1200_OR32_LHS:
			rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};
		  
		// l.addi
		`OR1200_OR32_ADDI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.addic
		`OR1200_OR32_ADDIC:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.andi
		`OR1200_OR32_ANDI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.ori
		`OR1200_OR32_ORI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.xori
		`OR1200_OR32_XORI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// l.muli
`ifdef OR1200_MULT_IMPLEMENTED
		`OR1200_OR32_MULI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
`endif
		  
		// Shift and rotate insns with immediate
		`OR1200_OR32_SH_ROTI:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
		  
		// ALU instructions except the one with immediate
		`OR1200_OR32_ALU:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

`ifdef OR1200_ALU_IMPL_CUST5
		// l.cust5 instructions
		`OR1200_OR32_CUST5:
			rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
`endif
`ifdef OR1200_FPU_IMPLEMENTED
		  // FPU instructions, lf.XXX.s, except sfxx
		  `OR1200_OR32_FLOAT:
		    rfwb_op <=  {`OR1200_RFWBOP_FPU,!id_insn[3]};
`endif
		// Instructions w/o register-file write-back
		default: 
			rfwb_op <=  `OR1200_RFWBOP_NOP;

		endcase
	end
end


//
// Decode of id_lsu_op
//
always @(id_insn) begin
	case (id_insn[31:26])		// synopsys parallel_case

	// l.lwz
	`OR1200_OR32_LWZ:
		id_lsu_op =  `OR1200_LSUOP_LWZ;

	// l.lws
	`OR1200_OR32_LWS:
		id_lsu_op =  `OR1200_LSUOP_LWS;

	// l.lbz
	`OR1200_OR32_LBZ:
		id_lsu_op =  `OR1200_LSUOP_LBZ;

	// l.lbs
	`OR1200_OR32_LBS:
		id_lsu_op =  `OR1200_LSUOP_LBS;

	// l.lhz
	`OR1200_OR32_LHZ:
		id_lsu_op =  `OR1200_LSUOP_LHZ;

	// l.lhs
	`OR1200_OR32_LHS:
		id_lsu_op =  `OR1200_LSUOP_LHS;

	// l.sw
	`OR1200_OR32_SW:
		id_lsu_op =  `OR1200_LSUOP_SW;

	// l.sb
	`OR1200_OR32_SB:
		id_lsu_op =  `OR1200_LSUOP_SB;

	// l.sh
	`OR1200_OR32_SH:
		id_lsu_op =  `OR1200_LSUOP_SH;

	// Non load/store instructions
	default:
		id_lsu_op =  `OR1200_LSUOP_NOP;

	endcase
end

//
// Decode of comp_op
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE) begin
		comp_op <=  4'd0;
	end else if (!ex_freeze & id_freeze | ex_flushpipe)
		comp_op <=  4'd0;
	else if (!ex_freeze)
		comp_op <=  id_insn[24:21];
end


//
// Decode of l.trap
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		sig_trap <=  1'b0;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		sig_trap <=  1'b0;
	else if (!ex_freeze) begin
`ifdef OR1200_VERBOSE
// synopsys translate_off
		if (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b010})
			$display("Generating sig_trap");
// synopsys translate_on
`endif
		sig_trap <=  (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b010})
			| du_hwbkpt;
	end
end

//This is the forwarding logic used
   
//
// Generation of sel_a
//
always @(*)
  if ((id_insn[20:16] == rf_addrw2) & rfwb_op2[0])
    sel_a = {1'b1, `OR1200_SEL_EX_FORW};
  else if ((id_insn[20:16] == rf_addrw1) & rfwb_op1[0])
    sel_a = {1'b0, `OR1200_SEL_EX_FORW};
  else if ((id_insn[20:16] == wb_rfaddrw2) & wbforw_valid2)
    sel_a = {1'b1, `OR1200_SEL_WB_FORW};
  else if ((id_insn[20:16] == wb_rfaddrw1) & wbforw_valid1)
    sel_a = {1'b0, `OR1200_SEL_WB_FORW};
  else
    sel_a = {1'b0, `OR1200_SEL_RF};

//
// Generation of sel_b
//
always @(*)
	if (sel_imm)
	  sel_b = {1'b0, `OR1200_SEL_IMM};
	else if ((id_insn[15:11] == rf_addrw2) & rfwb_op2[0])
	  sel_b = {1'b1, `OR1200_SEL_EX_FORW};
	else if ((id_insn[15:11] == rf_addrw1) & rfwb_op1[0])
	  sel_b = {1'b0, `OR1200_SEL_EX_FORW};
	else if ((id_insn[15:11] == wb_rfaddrw2) & wbforw_valid2)
	  sel_b = {1'b1, `OR1200_SEL_WB_FORW};
	else if ((id_insn[15:11] == wb_rfaddrw1) & wbforw_valid1)
	  sel_b = {1'b0, `OR1200_SEL_WB_FORW};
	else
	  sel_b = {1'b0, `OR1200_SEL_RF};

//
// Generation of ex_branch_op
//
always @(posedge clk or `OR1200_RST_EVENT rst)
	if (rst == `OR1200_RST_VALUE)
		ex_branch_op <=  `OR1200_BRANCHOP_NOP;
	else if (!ex_freeze & id_freeze | ex_flushpipe)
		ex_branch_op <=  `OR1200_BRANCHOP_NOP;		
	else if (!ex_freeze)
		ex_branch_op <=  id_branch_op;

//
// EX Sign extension of branch offset
//

// pipeline ID and EX branch target address 
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		ex_branch_addrtarget <=  0;
	else if (!ex_freeze) 
		ex_branch_addrtarget <=  id_branch_addrtarget;
end

//
// EX Sign/Zero extension of immediates
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		ex_simm <=  32'h0000_0000;
	else if (!ex_freeze) begin
		ex_simm <=  id_simm;
	end
end



   
endmodule
