`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/14/2021 12:42:16 PM
// Design Name: 
// Module Name: ALU
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

module ALU #(
    parameter                       DATA_WIDTH=32) (    // 
    input  logic [3:0]              alu_ctrl,           // 
    input  logic [DATA_WIDTH-1:0]   in_1,               // 
    input  logic [DATA_WIDTH-1:0]   in_2,               // 
    output logic                    b_flag,             // 
    output logic [DATA_WIDTH-1:0]   result);            // 
    
    // Enumerate possible alu_ctrl codes
    typedef enum logic [3:0] {
        ALU_ADD  = 4'b0000,
        ALU_SUB  = 4'b1000,
        ALU_SLL  = 4'b0001,
        ALU_SLT  = 4'b0010,
        ALU_SLTU = 4'b0011,
        ALU_XOR  = 4'b0100,
        ALU_SRL  = 4'b0101,
        ALU_SRA  = 4'b1101,
        ALU_OR   = 4'b0110,
        ALU_AND  = 4'b0111
    } alu_ctrls_t;
    
    // 
    always_comb begin
        case (alu_ctrl)
            // Add
            ALU_ADD:    begin
                            result = signed'(in_1) + signed'(in_2);
                            b_flag = 1'bX;
                        end
                        
            // Subtract
            ALU_SUB:    begin
                            result = signed'(in_1) - signed'(in_2);
                            b_flag = (result == '0);
                        end
            
            // Shift left logical
            ALU_SLL:    begin
                            result = signed'(in_1) << signed'(in_2[4:0]);
                            b_flag = 1'bX;
                        end
                        
            // Set less than
            ALU_SLT:    begin
                            result = (signed'(in_1) < signed'(in_2));
                            b_flag = result[0];
                        end

            // Set less than unsigned
            ALU_SLTU:   begin
                            result = (unsigned'(in_1) < unsigned'(in_2));
                            b_flag = result[0];
                        end
            
            // Exclusive-OR
            ALU_XOR:    begin
                            result = in_1 ^ in_2;
                            b_flag = 1'bX;
                        end
            
            // Shift right logical
            ALU_SRL:    begin
                            result = signed'(in_1) >> signed'(in_2[4:0]);
                            b_flag = 1'bX;
                        end
            
            // Shift right arithmetic
            ALU_SRA:    begin
                            result = signed'(in_1) >>> signed'(in_2[4:0]);
                            b_flag = 1'bX;
                        end
            
            // OR 
            ALU_OR:     begin
                            result = in_1 | in_2;
                            b_flag = 1'bX;
                        end
                        
            // AND 
            ALU_AND:    begin
                            result = in_1 & in_2;
                            b_flag = 1'bX;
                        end
                        
            // Unknwon alu_ctrl
            default:    begin
                            result = 'X;
                            b_flag = 1'bX;
                        end
        endcase
    end
    
endmodule
