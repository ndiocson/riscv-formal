`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2021 10:35:07 AM
// Design Name: 
// Module Name: MUX4x1
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


module MUX4x1#(
    parameter                       DATA_WIDTH=32) (    // 
    input  logic [1:0]              sel,                // 
    input  logic [DATA_WIDTH-1:0]   in_1,               // 
    input  logic [DATA_WIDTH-1:0]   in_2,               // 
    input  logic [DATA_WIDTH-1:0]   in_3,               // 
    input  logic [DATA_WIDTH-1:0]   in_4,               // 
    output logic [DATA_WIDTH-1:0]   out);               // 
    
    // 
    always_comb begin
        case (sel)
            2'b00:      out = in_1;
            2'b01:      out = in_2;
            2'b10:      out = in_3;
            2'b11:      out = in_4;
            default:    out = 'X;
        endcase
    end
    
endmodule
