`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/18/2021 11:30:09 AM
// Design Name: 
// Module Name: SignExtend
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

module SignExtend (
    input  logic [2:0]  imm_sel,    // 
    input  logic [31:0] instr,      // 
    output logic [31:0] imm_ext);   // 
    
    // Enumerate instruction types
    typedef enum logic [2:0] {
        I_TYPE   = 3'b000,
        I_TYPE_U = 3'b001,
        S_TYPE   = 3'b010,
        B_TYPE   = 3'b011,
        J_TYPE   = 3'b100,
        U_TYPE   = 3'b101
    } instr_types_t;
    
    // 
    always_comb begin
        case (imm_sel)
            I_TYPE:     imm_ext = {{20{instr[31]}}, instr[31:20]};
            I_TYPE_U:   imm_ext = {27'b0, instr[24:20]};
            S_TYPE:     imm_ext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            B_TYPE:     imm_ext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            J_TYPE:     imm_ext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            U_TYPE:     imm_ext = {instr[31:12], 12'b0};
            default:    imm_ext = 'X;
        endcase
    end

endmodule
