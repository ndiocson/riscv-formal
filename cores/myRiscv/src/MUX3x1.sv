`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2021 10:20:48 AM
// Design Name: 
// Module Name: MUX3x1
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

module MUX3x1 #(
    parameter                       DATA_WIDTH=32) (    // 
    input  logic [1:0]              sel,                // 
    input  logic [DATA_WIDTH-1:0]   in_1,               // 
    input  logic [DATA_WIDTH-1:0]   in_2,               // 
    input  logic [DATA_WIDTH-1:0]   in_3,               // 
    output logic [DATA_WIDTH-1:0]   out);               // 
    
    // 
    always_comb begin
        case (sel)
            2'b00:      out = in_1;
            2'b01:      out = in_2;
            2'b10:      out = in_3;
            default:    out = 'X;
        endcase
    end
    
endmodule
