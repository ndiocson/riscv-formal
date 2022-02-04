`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2021 12:04:09 PM
// Design Name: 
// Module Name: MemoryWriteInterface
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

interface mem_write #(parameter DATA_WIDTH=32, parameter ADDR_WIDTH=32, parameter WR_EN_WIDTH=4) ();
    
    // Internal signals to connect to memory module
//    logic wr_en;
    logic [WR_EN_WIDTH-1:0] wr_en;
    logic [ADDR_WIDTH-1:0] wr_addr;
    logic [DATA_WIDTH-1:0] wr_data;
    
    // Modport for memory module
    modport mem_in (
        input wr_en, wr_addr, wr_data
    );
    
    // Modport for memory write controller
    modport mem_out (
        output wr_en, wr_addr, wr_data
    );
    
endinterface
