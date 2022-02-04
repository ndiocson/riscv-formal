`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/14/2021 12:44:09 PM
// Design Name: 
// Module Name: Datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Datapath (
    input  logic            clk,            // System clock
    input  logic            reset,          // Reset signal
    input  logic            mem_in_sel,     // 
    input  logic            mem_out_sel,    // 
    input  logic [31:0]     instr,          // 32-bit instruction received from instruction memory
    input  logic [31:0]     rd_data,        // 32-bit data word received from data memory
    output logic [31:0]     pc,             // 32-bit program counter
    output logic [31:0]     addr,           // 32-bit output from ALU for addressing data memory
    output logic [31:0]     wr_data,        // 32-bit data word to write to data memory
    ctrl_to_data.data_path  dp_if,          // Interface connecting datapath to control unit
    mem_write.mem_in        wr_rf_if,       // Interface for register file memory writes
    mem_read.mem_in         rd_rf_if,       // Interface for register file memory reads

    `ifdef FORMAL_VERIFICATION
        output logic [63:0]     instr_index,    //
        output logic [4:0]      rf_src_addr_2,  // 
        output logic [31:0]     src_data_1,     // 
        output logic [31:0]     src_data_2,     // 
        output logic [4:0]      rf_dst_addr,    // 
        output logic [31:0]     rf_dst_data,    // 
        output logic [31:0]     pc_next,        // 
        output logic [3:0]      rmask           // 
    `endif
);
   
    // Declare internal signals
    logic rf_wr_en;                                             // 1-bit write-enable signal for register file
    logic [4:0] rf_dst_addr, rf_src_addr_1, rf_src_addr_2;      // 5-bit address signals for register file
    logic [31:0] dst_data, rf_dst_data;                         // 32-bit intermediate and final nets connecting to register file
    logic [31:0] imm_ext, rd_data_ext;                          // 32-bit internal signals for extend units
    logic [31:0] pc_next, pc_plus_4, target;                    // 32-bit internal signals for PC flip-flop
    logic [31:0] src_data_1, src_data_2;                        // 32-bit internal signals for register file
    logic [31:0] alu_in_1, alu_in_2;                            // 32-bit internal signals for ALU

    `ifdef FORMAL_VERIFICATION
        DFF #(.DATA_WIDTH(64)) instr_index_dff (
            .clk(clk), 
            .reset(reset), 
            .data(instr_index + 64'b1), 
            .out(instr_index)
        );
    `endif

    // 
    assign rd_rf_if.rd_data = src_data_2;
    
    // 
    assign rf_src_addr_1 = instr[19:15];

    // Adder for updating the PC for non-branch instructions
    assign pc_plus_4 = pc + 32'b0100;
    
    // Adder for updating the PC for B-, J-, or U-type instructions
    assign target = pc + imm_ext;
    
    // Register File instance for source and destination register access
    RegisterFile reg_file (
        .clk(clk),
        .wr_en(rf_wr_en),
        .dst_addr(rf_dst_addr),
        .src_addr_1(rf_src_addr_1),
        .src_addr_2(rf_src_addr_2),
        .dst_data(rf_dst_data),
        .src_data_1(src_data_1),
        .src_data_2(src_data_2)
    );
                      
    // ALU instance for instruction execution
    ALU #(.DATA_WIDTH(32)) alu (
        .alu_ctrl(dp_if.alu_ctrl),
        .in_1(alu_in_1),
        .in_2(alu_in_2),
        .b_flag(dp_if.b_flag),
        .result(dp_if.result)
    );
    
    // Sign-extend unit for sign extending the immediate of the instruction
    SignExtend sign_ext (
        .imm_sel(dp_if.imm_sel), 
        .instr(instr), 
        .imm_ext(imm_ext)
    );    
    
    // Load-extend unit for sign-extending or zero-extending the rd_data read from the data memory
    LoadExtend ld_ext (
        .ld_ctrl(dp_if.ld_ctrl), 
        .data_in(rd_data), 
        .data_out(rd_data_ext),

        `ifdef FORMAL_VERIFICATION
            .rmask(rmask)
        `endif
    );
    
    // Store-extend unit for rearranging src_data_2 before writing to the data memory
    StoreExtend s_ext (
        .s_ctrl(dp_if.s_ctrl), 
        .data_in(src_data_2), 
        .data_out(wr_data)
    );

    // Update PC at positive clock edge
    DFF #(.DATA_WIDTH(32)) pc_dff (
        .clk(clk), 
        .reset(reset), 
        .data(pc_next), 
        .out(pc)
    );  

    // 4-to-1 MUX instance for selecting pc_next source
    MUX4x1 #(.DATA_WIDTH(32)) pc_next_mux (
        .sel(pc_sel), 
        .in_1(pc_plus_4), 
        .in_2(target), 
        .in_3(dp_if.result), 
        .in_4({dp_if.result[31:1], 1'b0}), 
        .out(pc_next)
    );

    // 2-to-1 MUX instance for selecting addr output
    MUX2x1 #(.DATA_WIDTH(32)) addr_mux (
        .sel(dp_if.addr_align), 
        .in_1(dp_if.result), 
        .in_2({dp_if.result[31:2], 2'b0}), 
        .out(addr)
    );

    // 2-to-1 MUX instances for selecting ALU input sources
    MUX2x1 #(.DATA_WIDTH(32)) alu_in_mux_1 (.sel(dp_if.src_1_sel), .in_1(src_data_1), .in_2(pc), .out(alu_in_1));
    MUX2x1 #(.DATA_WIDTH(32)) alu_in_mux_2 (.sel(dp_if.src_2_sel), .in_1(src_data_2), .in_2(imm_ext), .out(alu_in_2));
    
    // 5-to-1 MUX instance for selecting data source for destination register writes
    MUX5x1 #(.DATA_WIDTH(32)) rf_data_mux (
        .sel(rd_data_sel), 
        .in_1(dp_if.result), 
        .in_2(rd_data_ext), 
        .in_3(pc_plus_4), 
        .in_4(imm_ext), 
        .in_5(32'b0), 
        .out(dst_data)
    );
    
    // 2-to-1 MUX instance for selecting register file inputs/outputs
    MUX2x1 #(.DATA_WIDTH(1)) rf_mux_1 (.sel(mem_in_sel), .in_1(dp_if.rf_wr_en), .in_2(wr_rf_if.wr_en), .out(rf_wr_en));
    MUX2x1 #(.DATA_WIDTH(5)) rf_mux_2 (.sel(dp_if.dst_addr_sel), .in_1(instr[11:7]), .in_2(32'b0), .out(dst_addr));
    MUX2x1 #(.DATA_WIDTH(5)) rf_mux_3 (.sel(mem_in_sel), .in_1(dst_addr), .in_2(wr_rf_if.wr_addr), .out(rf_dst_addr));
    MUX2x1 #(.DATA_WIDTH(32)) rf_mux_4 (.sel(mem_in_sel), .in_1(dst_data), .in_2(wr_rf_if.wr_data), .out(rf_dst_data));
    MUX2x1 #(.DATA_WIDTH(5)) rf_mux_5 (.sel(mem_out_sel), .in_1(instr[24:20]), .in_2(rd_rf_if.rd_addr), .out(rf_src_addr_2));
    
endmodule
