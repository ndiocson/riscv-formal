`define FORMAL_VERIFICATION
`include "src/CtrlToDataInterface.sv"
`include "src/MemoryWriteInterface.sv"
`include "src/MemoryReadInterface.sv"

`include "src/DFF.sv"
`include "src/MUX2x1.sv"
`include "src/MUX4x1.sv"
`include "src/MUX5x1.sv"
`include "src/SignExtend.sv"
`include "src/LoadExtend.sv"
`include "src/StoreExtend.sv"
`include "src/RegisterFile.sv"
`include "src/ALU.sv"
`include "src/Datapath.sv"
`include "src/ControlUnit.sv"


module rvfi_wrapper (
	input clock,
	input reset,

	`RVFI_OUTPUTS
);

	(* keep *) wire [31:0] pc;
	(* keep *) `rvformal_rand_reg [31:0] instr;

	(* keep *) wire [3:0] wr_en;
	(* keep *) wire [31:0] addr;
	(* keep *) wire [31:0] wr_data;
	(* keep *) `rvformal_rand_reg   [31:0] rd_data;

	// mem_write #(.DATA_WIDTH(32), .ADDR_WIDTH(5), .WR_EN_WIDTH(1)) wr_rf_if();
	// mem_read #(.DATA_WIDTH(32), .ADDR_WIDTH(5)) rd_rf_if();

	myRiscv uut (
		.clk(clock),
		.reset(reset),
		// .mem_in_sel(1'b0),
		// .mem_out_sel(1'b0),
		.instr(instr),
		.rd_data(rd_data),
		.wr_en(wr_en),
		.pc(pc),
		.addr(addr),
		.wr_data(wr_data),
		// .wr_rf_if(wr_rf_if.mem_in),
		// .rd_rf_if(rd_rf_if.mem_in),

		`RVFI_CONN
	);

endmodule

