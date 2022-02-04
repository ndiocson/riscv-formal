`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/18/2021 11:04:27 AM
// Design Name: 
// Module Name: DFF
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

module DFF #(
    parameter                       DATA_WIDTH=32) (    // 
    input  logic                    clk,                // System clock
    input  logic                    reset,              // Reset signal
    input  logic [DATA_WIDTH-1:0]   data,               // 
    output logic [DATA_WIDTH-1:0]   out);               // 
    
    always_ff @(posedge clk) begin
        if (reset == 1'b1)
            out <= '0;
        else
            out <= data;
    end
    
endmodule
