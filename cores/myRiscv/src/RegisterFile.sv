`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/14/2021 12:42:16 PM
// Design Name: 
// Module Name: RegisterFile
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

module RegisterFile (
    input  logic        clk,            // System clock
    input  logic        wr_en,          // Enable signal for allowing dst_data to be written to wr_addr
    input  logic [4:0]  dst_addr,       // 5-bit address of destination register to write dst_data to
    input  logic [4:0]  src_addr_1,     // 5-bit address of first source register to read from
    input  logic [4:0]  src_addr_2,     // 5-bit address of second source register to read from
    input  logic [31:0] dst_data,       // 32-bit data to write to destination register
    output logic [31:0] src_data_1,     // 32-bit data read from first source register
    output logic [31:0] src_data_2);    // 32-bit data read from the second source register
    
    // 32, 32-bit registers
    logic [31:0] mem [0:31];
    
    // Data write
    always_ff @(posedge clk) begin
        if (wr_en == 1'b1)
            mem[dst_addr] <= (dst_addr == 32'b0) ? 32'b0 : dst_data;
        else
            mem[dst_addr] <= mem[dst_addr];
    end

    // Data read
    always_comb begin
        src_data_1 = (src_addr_1 == 32'b0) ? 32'b0 : mem[src_addr_1];
        src_data_2 = (src_addr_2 == 32'b0) ? 32'b0 : mem[src_addr_2];
    end
    
endmodule
