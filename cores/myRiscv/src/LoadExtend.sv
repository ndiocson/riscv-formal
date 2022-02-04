`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/29/2021 12:20:56 PM
// Design Name: 
// Module Name: LoadExtend
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

module LoadExtend (
    input  logic [4:0]  ld_ctrl,    // 
    input  logic [31:0] data_in,    // 
    output logic [31:0] data_out,   // 

    `ifdef FORMAL_VERIFICATION
        output logic [3:0]  rmask       //
    `endif
);
    
    // Enumerate load instruction control
    typedef enum logic [4:0] {
        LD_BYTE_0   = 5'b000_00,
        LD_BYTE_1   = 5'b000_01,
        LD_BYTE_2   = 5'b000_10,
        LD_BYTE_3   = 5'b000_11,
        LD_HALF_0   = 5'b001_00,
        LD_HALF_1   = 5'b001_10,
        LD_WORD     = 5'b010_00,
        LD_BYTE_U_0 = 5'b100_00,
        LD_BYTE_U_1 = 5'b100_01,
        LD_BYTE_U_2 = 5'b100_10,
        LD_BYTE_U_3 = 5'b100_11,
        LD_HALF_U_0 = 5'b101_00,
        LD_HALF_U_1 = 5'b101_10
    } ld_ctrls_t;
    
    // 
    always_comb begin
        case (ld_ctrl)
            LD_BYTE_0:      data_out = {{24{data_in[7]}}, data_in[7:0]};
            LD_BYTE_1:      data_out = {{24{data_in[15]}}, data_in[15:8]};
            LD_BYTE_2:      data_out = {{24{data_in[23]}}, data_in[23:16]};
            LD_BYTE_3:      data_out = {{24{data_in[31]}}, data_in[31:24]};
            LD_HALF_0:      data_out = {{16{data_in[15]}}, data_in[15:0]};
            LD_HALF_1:      data_out = {{16{data_in[31]}}, data_in[31:16]};
            LD_WORD:        data_out = data_in;
            LD_BYTE_U_0:    data_out = {24'b0, data_in[7:0]};
            LD_BYTE_U_1:    data_out = {24'b0, data_in[15:8]};
            LD_BYTE_U_2:    data_out = {24'b0, data_in[23:16]};
            LD_BYTE_U_3:    data_out = {24'b0, data_in[31:24]};
            LD_HALF_U_0:    data_out = {16'b0, data_in[15:0]};
            LD_HALF_U_1:    data_out = {16'b0, data_in[31:16]};
            default:        data_out = 'X;
        endcase
    end
    
    `ifdef FORMAL_VERIFICATION
        // 
        always_comb begin
            case (ld_ctrl)
                LD_BYTE_0:      rmask = 4'b0001;
                LD_BYTE_1:      rmask = 4'b0010;
                LD_BYTE_2:      rmask = 4'b0100;
                LD_BYTE_3:      rmask = 4'b1000;
                LD_HALF_0:      rmask = 4'b0011;
                LD_HALF_1:      rmask = 4'b1100;
                LD_WORD:        rmask = 4'b1111;
                LD_BYTE_U_0:    rmask = 4'b0001;
                LD_BYTE_U_1:    rmask = 4'b0010;
                LD_BYTE_U_2:    rmask = 4'b0100;
                LD_BYTE_U_3:    rmask = 4'b1000;
                LD_HALF_U_0:    rmask = 4'b0011;
                LD_HALF_U_1:    rmask = 4'b1100;
                default:        rmask = 4'b0000;
            endcase
        end
    `endif

endmodule
