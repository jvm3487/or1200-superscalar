//This file was create to try and break the control down into better sizes that could be used for two insns

`include "timescale.v"
`include "or1200_defines.v"

module or1200_ctrl_if_decode
  (
   // Clock and reset
   clk, rst, if_insn, id_freeze, id_flushpipe, rf_addr1, rf_addr2, rf_rd1, rf_rd2, sel_imm, id_branch_op
   
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
   output 					sel_imm;
   output [`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
   reg 						sel_imm;
   reg	[`OR1200_BRANCHOP_WIDTH-1:0]		id_branch_op;
   

//
// Register file read addresses
//
   assign rf_addr1 = if_insn[20:16];
   assign rf_addr2 = if_insn[15:11];
   assign rf_rd1 = if_insn[31] || if_maci_op;
   assign rf_rd2 = if_insn[30];
   
//
// Decode of sel_imm
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		sel_imm <=  1'b0;
	else if (!id_freeze) begin
	  case (if_insn[31:26])		// synopsys parallel_case

	    // j.jalr
	    `OR1200_OR32_JALR:
	      sel_imm <=  1'b0;
	    
	    // l.jr
	    `OR1200_OR32_JR:
	      sel_imm <=  1'b0;
	    
	    // l.rfe
	    `OR1200_OR32_RFE:
	      sel_imm <=  1'b0;
	    
	    // l.mfspr
	    `OR1200_OR32_MFSPR:
	      sel_imm <=  1'b0;
	    
	    // l.mtspr
	    `OR1200_OR32_MTSPR:
	      sel_imm <=  1'b0;
	    
	    // l.sys, l.brk and all three sync insns
	    `OR1200_OR32_XSYNC:
	      sel_imm <=  1'b0;
	    
	    // l.mac/l.msb
`ifdef OR1200_MAC_IMPLEMENTED
	    `OR1200_OR32_MACMSB:
	      sel_imm <=  1'b0;
`endif

	    // l.sw
	    `OR1200_OR32_SW:
	      sel_imm <=  1'b0;
	    
	    // l.sb
	    `OR1200_OR32_SB:
	      sel_imm <=  1'b0;
	    
	    // l.sh
	    `OR1200_OR32_SH:
	      sel_imm <=  1'b0;
	    
	    // ALU instructions except the one with immediate
	    `OR1200_OR32_ALU:
	      sel_imm <=  1'b0;
	    
	    // SFXX instructions
	    `OR1200_OR32_SFXX:
	      sel_imm <=  1'b0;

`ifdef OR1200_IMPL_ALU_CUST5
	    // l.cust5 instructions
	    `OR1200_OR32_CUST5:
	      sel_imm <=  1'b0;
`endif
`ifdef OR1200_FPU_IMPLEMENTED
	    // FPU instructions
	    `OR1200_OR32_FLOAT:
	      sel_imm <=  1'b0;
`endif
	    // l.nop
	    `OR1200_OR32_NOP:
	      sel_imm <=  1'b0;

	    // All instructions with immediates
	    default: begin
	      sel_imm <=  1'b1;
	    end
	    
	  endcase
	  
	end
end

//
// Decode of id_branch_op
//
always @(posedge clk or `OR1200_RST_EVENT rst) begin
	if (rst == `OR1200_RST_VALUE)
		id_branch_op <=  `OR1200_BRANCHOP_NOP;
	else if (id_flushpipe)
		id_branch_op <=  `OR1200_BRANCHOP_NOP;
	else if (!id_freeze) begin
		case (if_insn[31:26])		// synopsys parallel_case

		// l.j
		`OR1200_OR32_J:
			id_branch_op <=  `OR1200_BRANCHOP_J;
		  
		// j.jal
		`OR1200_OR32_JAL:
			id_branch_op <=  `OR1200_BRANCHOP_J;
		  
		// j.jalr
		`OR1200_OR32_JALR:
			id_branch_op <=  `OR1200_BRANCHOP_JR;
		  
		// l.jr
		`OR1200_OR32_JR:
			id_branch_op <=  `OR1200_BRANCHOP_JR;
		  
		// l.bnf
		`OR1200_OR32_BNF:
			id_branch_op <=  `OR1200_BRANCHOP_BNF;
		  
		// l.bf
		`OR1200_OR32_BF:
			id_branch_op <=  `OR1200_BRANCHOP_BF;
		  
		// l.rfe
		`OR1200_OR32_RFE:
			id_branch_op <=  `OR1200_BRANCHOP_RFE;
		  
		// Non branch instructions
		default:
			id_branch_op <=  `OR1200_BRANCHOP_NOP;

		endcase
	end
end

   
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
