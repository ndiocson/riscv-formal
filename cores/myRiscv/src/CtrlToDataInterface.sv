`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/25/2021 12:11:49 PM
// Design Name: 
// Module Name: CtrlToDataInterface
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

interface ctrl_to_data();
    
    // Internal signals to connect between datapath and control unit
    logic b_flag;
    logic rf_wr_en;
    logic src_1_sel;
    logic src_2_sel;
    logic addr_align;
    logic dst_addr_sel;
    logic [1:0] pc_sel;
    logic [1:0] rd_data_sel;
    logic [1:0] wr_data_sel;
    logic [2:0] imm_sel;
    logic [2:0] ld_ctrl;
    logic [4:0] s_ctrl;
    logic [3:0] alu_ctrl;
    logic [31:0] result;
    
    // Modport for datapath
    modport data_path (
        input rf_wr_en, src_1_sel, src_2_sel, addr_align, dst_addr_sel, pc_sel, rd_data_sel, wr_data_sel, imm_sel, ld_ctrl, s_ctrl, alu_ctrl,
        output b_flag, result
    );
    
    // Modport for control unit
    modport ctrl_unit (
        input b_flag, result,
        output rf_wr_en, src_1_sel, src_2_sel, addr_align, dst_addr_sel, pc_sel, rd_data_sel, wr_data_sel, imm_sel, ld_ctrl, s_ctrl, alu_ctrl,
    );
    
endinterface
