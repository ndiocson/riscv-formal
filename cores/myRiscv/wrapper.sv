module rvfi_wrapper (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);

	(* keep *) wire [31:0] pc;
	(* keep *) `rvformal_rand_reg [31:0] instr;

	(* keep *) wire [3:0] wr_en;
	(* keep *) wire [31:0] addr;
	(* keep *) wire [31:0] wr_data;
	(* keep *) `rvformal_rand_reg   [31:0] rd_data;

	myRiscv uut (
		.clk(clock),
		.reset(reset),
		.instr(instr),
		.rd_data(rd_data),
		.wr_en(wr_en),
		.pc(pc),
		.addr(addr),
		.wr_data(wr_data),
		`RVFI_CONN);
		// .rvfi_valid     (rvfi_valid    ), 
		// .rvfi_order     (rvfi_order    ),
		// .rvfi_insn      (rvfi_insn     ),
		// .rvfi_trap      (rvfi_trap     ),
		// .rvfi_halt      (rvfi_halt     ),
		// .rvfi_intr      (rvfi_intr     ),
		// .rvfi_mode      (rvfi_mode     ),
		// .rvfi_ixl       (rvfi_ixl      ),
		// .rvfi_rs1_addr  (rvfi_rs1_addr ),
		// .rvfi_rs2_addr  (rvfi_rs2_addr ),
		// .rvfi_rs1_rdata (rvfi_rs1_rdata),
		// .rvfi_rs2_rdata (rvfi_rs2_rdata),
		// .rvfi_rd_addr   (rvfi_rd_addr  ),
		// .rvfi_rd_wdata  (rvfi_rd_wdata ),
		// .rvfi_pc_rdata  (rvfi_pc_rdata ),
		// .rvfi_pc_wdata  (rvfi_pc_wdata ),
		// .rvfi_mem_addr  (rvfi_mem_addr ),
		// .rvfi_mem_rmask (rvfi_mem_rmask),
		// .rvfi_mem_wmask (rvfi_mem_wmask),
		// .rvfi_mem_rdata (rvfi_mem_rdata),
		// .rvfi_mem_wdata (rvfi_mem_wdata));

endmodule

