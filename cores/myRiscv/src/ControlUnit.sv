`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/17/2021 06:18:02 PM
// Design Name: 
// Module Name: ControlUnit
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

module ControlUnit (
    input  logic [6:0]      opcode,     // 7-bit opcode slice from 32-bit instruction
    input  logic [2:0]      funct3,     // 3-bit funct3 slice from 32-bit instruction
    input  logic [6:0]      funct7,     // 7-bit funct7 slice from 32-bit instruction
    output logic [3:0]      wr_en,      // Write-enable signal for controlling data memory writes
    
    `ifdef FORMAL_VERIFICATION
        output logic        trap,       // 
    `endif

    ctrl_to_data.ctrl_unit  cu_if       // Interface connecting datapath to control unit
);     
    
    // Enumerate opcodes
    typedef enum logic [6:0] {
        LOAD       = 7'b0000011,
        IMM        = 7'b0010011,
        STORE      = 7'b0100011,
        ARITH      = 7'b0110011,
        BRANCH     = 7'b1100011,
        JUMP       = 7'b1101111,
        JUMP_REG   = 7'b1100111,
        LOAD_UIMM  = 7'b0110111,
        ADD_UIMM   = 7'b0010111
    } ops_t;
    
    logic load_check;
    logic imm_check;
    logic store_check;
    logic arith_check;
    logic branch_check;
    logic jump_check;
    logic jump_reg_check;
    logic load_uimm_check;
    logic add_uimm_check;
    logic [6:0] opcode_mask;

    assign load_check = &{~opcode[6:2], opcode[1:0]} & (~funct3[2] & ~funct3[0]) | funct3[1];
    assign imm_check = &{~opcode[6:5], opcode[4], ~opcode[3:2], opcode[1:0]} & (funct3[1] | ~funct3[0]) | (~funct7) | (~{funct7[6], funct7[4:0]} & funct3[2]);
    assign store_check = &{~opcode[6], opcode[5], ~opcode[4:2], opcode[1:0]} & (~funct3[2] & ~funct3[1]) | (~funct3[2] & ~funct3[0]);
    assign arith_check = &{~opcode[6], opcode[5:4], ~opcode[3:2], opcode[1:0]} & &~{funct7[6], funct7[4:0]} & (~funct3[1] & ~(funct3[2] ^ funct3[0])) | ~funct7[5];
    assign branch_check = &{opcode[6:5], ~opcode[4:2], opcode[1:0]} & funct3[2] | ~funct3[1];
    assign jump_check = &{opcode[6:5], ~opcode[4], opcode[3:0]};
    assign jump_reg_check = &{opcode[6:5], ~opcode[4:3], opcode[2:0]} & &~funct3;
    assign load_uimm_check = &{~opcode[6], opcode[5:4], ~opcode[3], opcode[2:0]};
    assign add_uimm_check = &{~opcode[6:5], opcode[4], ~opcode[3], opcode[2:0]};
    assign opcode_mask = {7{|{load_check, imm_check, store_check, arith_check, branch_check, jump_check, jump_reg_check, load_uimm_check, add_uimm_check}}};

    // Combinational logic for determining outputs based on instruction slices
    always_comb begin
        cu_if.dst_addr_sel = 1'b0;
        cu_if.addr_align = 1'b0;
        cu_if.s_ctrl = 5'b010_00;

        `ifdef FORMAL_VERIFICATION
            trap = 1'b0;
        `endif

        case (opcode & opcode_mask)
            // Load instructions
            LOAD:       begin
                            wr_en = 4'b0000;

                            if (funct3 == 3'b010)
                                cu_if.rf_wr_en = (cu_if.result[1:0] != 2'b0) ? 1'b0 : 1'b1;
                            else if (funct3 == 3'b001 || funct3 == 3'b101)
                                cu_if.rf_wr_en = (cu_if.result[0] != 1'b0) ? 1'b0 : 1'b1;
                            else 
                                cu_if.rf_wr_en = 1'b1;

                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b1;
                            cu_if.addr_align = 1'b1;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'b001;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b000;
                            cu_if.ld_ctrl = {funct3, cu_if.result[1:0]};
                            cu_if.alu_ctrl = 4'b0000;
                            
                            `ifdef FORMAL_VERIFICATION
                                if (funct3 == 3'b010)
                                    trap = (cu_if.result[1:0] != 2'b0) ? 1'b1 : 1'b0;
                                else if (funct3 == 3'b001 || funct3 == 3'b101)
                                    trap = (cu_if.result[0] != 1'b0) ? 1'b1 : 1'b0;
                                else 
                                    trap = 1'b0;
                            `endif
                        end
            
            // Immediate instructions
            IMM:        begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b1;
                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b1;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'b000;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = {2'b0, (~funct3[1] & funct3[0])};
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = {(funct3[2] & ~funct3[1] & funct3[0] & funct7[5]), funct3};
                        end
            
            // Store instructions
            STORE:      begin
                            if (funct3 == 3'b010)
                                wr_en = (cu_if.result[1:0] != 2'b0) ? 4'b0000 : 4'b1111;
                            else if (funct3 == 3'b001)
                                wr_en = (cu_if.result[0] != 1'b0) ? 4'b0000 : ((cu_if.result[1] == 1'b0) ? 4'b0011 : 4'b1100);
                            else begin
                                case (cu_if.result[1:0])
                                    2'b00:      wr_en = 4'b0001;
                                    2'b01:      wr_en = 4'b0010;
                                    2'b10:      wr_en = 4'b0100;
                                    2'b11:      wr_en = 4'b1000;
                                    default:    wr_en = 4'bX;
                                endcase
                            end

                            cu_if.rf_wr_en = 1'b0;
                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b1;
                            cu_if.addr_align = 1'b1;
                            cu_if.pc_sel = 2'b00;
                            cu_if.dst_addr_sel = 1'b1;
                            cu_if.rd_data_sel = 3'bX;
                            cu_if.wr_data_sel = funct3[1:0];
                            cu_if.imm_sel = 3'b010;
                            cu_if.s_ctrl = {funct3, cu_if.result[1:0]};
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'b0000;

                            `ifdef FORMAL_VERIFICATION
                                if (funct3 == 3'b010)
                                    trap = (cu_if.result[1:0] != 2'b0) ? 1'b1 : 1'b0;
                                else if (funct3 == 3'b001)
                                    trap = (cu_if.result[0] != 1'b0) ? 1'b1 : 1'b0;
                                else 
                                    trap = 1'b0;
                            `endif
                        end
            
            //  Arithmetic instructions 
            ARITH:      begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b1;
                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b0;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'b000;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'bX;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = {funct7[5], funct3};
                        end            
            
            // Branch instructions
            BRANCH:     begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b0;
                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b0;
                            cu_if.pc_sel = {1'b0, (funct3[0] ^ b_flag)};
                            cu_if.dst_addr_sel = 1'b1;
                            cu_if.rd_data_sel = 3'b100;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b011;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = {~funct3[2], 1'b0, funct3[2:1]};

                            `ifdef FORMAL_VERIFICATION
                                trap = |pc_next[1:0];
                            `endif
                        end
            
            // Jump and link
            JUMP:       begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = &~target[1:0];
                            cu_if.src_1_sel = 1'bX;
                            cu_if.src_2_sel = 1'bX;
                            cu_if.pc_sel = {1'b0, &~target[1:0]};
                            cu_if.rd_data_sel = 3'b010;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b100;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'bX;

                            `ifdef FORMAL_VERIFICATION
                                trap = |target[1:0];
                            `endif
                        end
            
            // Jump and link register
            JUMP_REG:   begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = ~cu_if.result[1];
                            cu_if.src_1_sel = 1'b0;
                            cu_if.src_2_sel = 1'b1;
                            cu_if.pc_sel = {2{~cu_if.result[1]}};
                            cu_if.rd_data_sel = 3'b010;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b000;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'b0000;

                            `ifdef FORMAL_VERIFICATION
                                trap = cu_if.result[1];
                            `endif
                        end
            
            // Load upper-immediate instructions
            LOAD_UIMM:  begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b1;
                            cu_if.src_1_sel = 1'bX;
                            cu_if.src_2_sel = 1'bX;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'b011;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b101;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'bX;
                        end
            
            // Add upper-immediate to PC instruction
            ADD_UIMM:   begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b1;
                            cu_if.src_1_sel = 1'b1;
                            cu_if.src_2_sel = 1'b1;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'b000;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'b101;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'b0000;
                        end            
            
            // Unknown opcode (treated as NOPs - only PC is incremented)
            default:    begin
                            wr_en = 4'b0000;
                            cu_if.rf_wr_en = 1'b0;
                            cu_if.src_1_sel = 1'bX;
                            cu_if.src_2_sel = 1'bX;
                            cu_if.pc_sel = 2'b00;
                            cu_if.rd_data_sel = 3'bX;
                            cu_if.wr_data_sel = 2'bX;
                            cu_if.imm_sel = 3'bX;
                            cu_if.ld_ctrl = 5'bX;
                            cu_if.alu_ctrl = 4'bX;

                            `ifdef FORMAL_VERIFICATION
                                trap = 1'b1;
                            `endif
                        end
        endcase
    end

endmodule
