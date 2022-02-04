
module MUX5x1 #(
    parameter                       DATA_WIDTH=32) (
    input  logic [2:0]              sel,
    input  logic [DATA_WIDTH-1:0]   in_1,
    input  logic [DATA_WIDTH-1:0]   in_2,
    input  logic [DATA_WIDTH-1:0]   in_3,
    input  logic [DATA_WIDTH-1:0]   in_4,
    input  logic [DATA_WIDTH-1:0]   in_5,
    output logic [DATA_WIDTH-1:0]   out);

    // 
    always_comb begin
        case (sel)
            3'b000:     out = in_1;
            3'b001:     out = in_2;
            3'b010:     out = in_3;
            3'b011:     out = in_4;
            3'b100:     out = in_5;
            default:    out = 'X;
        endcase
    end

endmodule