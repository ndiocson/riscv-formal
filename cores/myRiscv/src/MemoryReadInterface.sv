`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/16/2021 12:10:27 PM
// Design Name: 
// Module Name: MemoryReadInterface
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

interface mem_read #(parameter DATA_WIDTH=32, parameter ADDR_WIDTH=32) ();

    // Declare signals to connect to memory module
    logic [ADDR_WIDTH-1:0] rd_addr;
    logic [DATA_WIDTH-1:0] rd_data;
    
    // Modport for memory module
    modport mem_in (
        input rd_addr,
        output rd_data
    );
    
    // Modport for memory read controller
    modport mem_out (
        input rd_data,
        output rd_addr
    );

endinterface
