
module StoreExtend (
    input  logic [4:0]  s_ctrl,
    input  logic [31:0] data_in,
    output logic [31:0] data_out);

    // Enumerate store instruction control
    typedef enum logic [4:0] {
        S_BYTE_0   = 5'b000_00,
        S_BYTE_1   = 5'b000_01,
        S_BYTE_2   = 5'b000_10,
        S_BYTE_3   = 5'b000_11,
        S_HALF_0   = 5'b001_00,
        S_HALF_1   = 5'b001_10,
        S_WORD     = 5'b010_00
    } s_ctrls_t;

    // 
    always_comb begin
        data_out = 32'b0;
        case (s_ctrl)
            S_BYTE_0:       data_out[7:0] = data_in[7:0];
            S_BYTE_1:       data_out[15:8] = data_in[7:0];
            S_BYTE_2:       data_out[23:16] = data_in[7:0];
            S_BYTE_3:       data_out[31:24] = data_in[7:0];
            S_HALF_0:       data_out[15:0] = data_in[15:0];
            S_HALF_1:       data_out[31:16] = data_in[15:0];
            S_WORD:         data_out = data_in;
            default:        data_out = 'X;
        endcase
    end

endmodule