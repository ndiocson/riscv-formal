`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/18/2021 11:04:27 AM
// Design Name: 
// Module Name: MUX2x1
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

module MUX2x1 #(
    parameter                       DATA_WIDTH=32) (    // 
    input  logic                    sel,                // 
    input  logic [DATA_WIDTH-1:0]   in_1,               // 
    input  logic [DATA_WIDTH-1:0]   in_2,               // 
    output logic [DATA_WIDTH-1:0]   out);               // 
    
    // 
    always_comb begin
        case (sel)
            1'b0:       out = in_1;
            1'b1:       out = in_2;
            default:    out = 'X;
        endcase
    end
    
endmodule
