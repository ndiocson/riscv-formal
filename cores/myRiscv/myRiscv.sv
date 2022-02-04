
module myRiscv (
    input  logic        clk,            // System clock
    input  logic        reset,          // Reset signal
    input  logic [31:0] instr,          // 32-bit instruction received from instruction memory
    input  logic [31:0] rd_data,        // 32-bit data word received from data memor
    output logic [3:0]  wr_en,          // Write-enable signal for controlling data memory writes
    output logic [31:0] pc,             // 32-bit program counter
    output logic [31:0] addr,           // 32-bit output from ALU for addressing data memory
    output logic [31:0] wr_data,        // 32-bit data word to write to data memory
    
    `RVFI_OUTPUTS
);
    
    // Internal signals to connect between datapath and control unit
    logic b_flag;
    logic rf_wr_en;
    logic src_1_sel;
    logic src_2_sel;
    logic [1:0] pc_sel;
    logic [2:0] rd_data_sel;
    logic [1:0] wr_data_sel;
    logic [2:0] imm_sel;
    logic [4:0] ld_ctrl;
    logic [3:0] alu_ctrl;

    logic [6:0] opcode;
    assign opcode = (reset == 1'b1) ? '0 : instr[6:0];


    // -------------------------------------------------------- RVFI CONTROL --------------------------------------------------------

    // Instruction Metadata
    assign rvfi_valid = ~reset;
    assign rvfi_order = instr_index;
    assign rvfi_insn = instr;
    assign rvfi_trap = ctrl_instr_trap;
    assign rvfi_halt = 1'b0;
    assign rvfi_intr = 1'b0;
    assign rvfi_mode = 2'b11;
    assign rvfi_ixl = 2'b01;

    // Integer Register Read/Write
    assign rvfi_rs1_addr = instr[19:15];
    assign rvfi_rs2_addr = rf_src_addr_2;
    assign rvfi_rs1_rdata = src_data_1;
    assign rvfi_rs2_rdata = src_data_2;
    assign rvfi_rd_addr = rf_dst_addr;
    assign rvfi_rd_wdata = (rf_dst_addr == 32'b0) ? 32'b0 : rf_dst_data;
    
    // Program Counter
    assign rvfi_pc_rdata = pc;
    assign rvfi_pc_wdata = pc_next;

    // Memory Access
    assign rvfi_mem_addr = addr;
    assign rvfi_mem_rmask = rmask;
    assign rvfi_mem_wmask = wr_en;
    assign rvfi_mem_rdata = rd_data;
    assign rvfi_mem_wdata = wr_data;

    // -------------------------------------------------------- RVFI CONTROL --------------------------------------------------------


    // -------------------------------------------------------- DATAPATH --------------------------------------------------------

    logic [63:0] instr_index;                                   // 
    logic [4:0] rf_dst_addr, rf_src_addr_1, rf_src_addr_2;      // 
    logic [31:0] dst_data, rf_dst_data;                         // 32-bit intermediate and final nets connecting to register file
    logic [31:0] imm_ext, rd_data_ext;                          // 32-bit extended signals
    logic [31:0] pc_next, pc_plus_4, target;                    // 32-bit internal signals for PC flip-flop
    logic [31:0] src_data_1, src_data_2;                        // 32-bit internal signals for register file
    logic [31:0] alu_in_1, alu_in_2;                            // 32-bit internal signals for ALU

    // assign addr = result;
    logic addr_align;
    always_comb begin
        case (addr_align)
            1'b0:       addr = result;
            1'b1:       addr = {result[31:2], 2'b0};
            default:    addr = result;
        endcase
    end

    // Adder for updating the PC for non-branch instructions
    assign pc_plus_4 = pc + 32'b0100;
    
    // Adder for updating the PC for B-, J-, or U-type instructions
    assign target = pc + imm_ext;

    // assign rf_dst_addr = instr[11:7];
    logic dst_addr_sel;
    always_comb begin
        case (dst_addr_sel)
            1'b0:       rf_dst_addr = instr[11:7];
            1'b1:       rf_dst_addr = 32'b0;
            default:    rf_dst_addr = 'X;
        endcase
    end

    assign rf_dst_data = dst_data;
    assign rf_src_addr_1 = instr[19:15];
    assign rf_src_addr_2 = instr[24:20];

    // 
    always_ff @(posedge clk) begin
        if (reset == 1'b1)
            instr_index <= '0;
        else
            instr_index <= instr_index + 64'b1;
    end

    // Update PC at positive clock edge
    DFF #(.DATA_WIDTH(32)) pc_dff (.clk(clk), .reset(reset), .data(pc_next), .out(pc));

    // 4-to-1 MUX instance for selecting pc_next source
    MUX4x1 #(.DATA_WIDTH(32)) pc_next_mux (.sel(pc_sel), .in_1(pc_plus_4), .in_2(target), .in_3(result), .in_4({result[31:1], 1'b0}), .out(pc_next));

    // 2-to-1 MUX instances for selecting ALU input sources
    MUX2x1 #(.DATA_WIDTH(32)) alu_in_mux_1 (.sel(src_1_sel), .in_1(src_data_1), .in_2(pc), .out(alu_in_1));
    MUX2x1 #(.DATA_WIDTH(32)) alu_in_mux_2 (.sel(src_2_sel), .in_1(src_data_2), .in_2(imm_ext), .out(alu_in_2));

    // 5-to-1 MUX instance for selecting data source for destination register writes
    MUX5x1 #(.DATA_WIDTH(32)) rf_data_mux (.sel(rd_data_sel), .in_1(result), .in_2(rd_data_ext), .in_3(pc_plus_4), .in_4(imm_ext), .in_5(32'b0), .out(dst_data));


    // ------------------------- Register File -------------------------

    RegisterFile reg_file (
        .clk(clk),
        .wr_en(rf_wr_en),
        .dst_addr(rf_dst_addr),
        .src_addr_1(rf_src_addr_1),
        .src_addr_2(rf_src_addr_2),
        .dst_data(rf_dst_data),
        .src_data_1(src_data_1),
        .src_data_2(src_data_2));

    // ------------------------- Register File -------------------------


    // ------------------------- ALU -------------------------

    logic [31:0] result;

    // ALU instance for instruction execution
    ALU #(.DATA_WIDTH(32)) alu (
        .alu_ctrl(alu_ctrl),
        .in_1(alu_in_1),
        .in_2(alu_in_2),
        .b_flag(b_flag),
        .result(result));

    // ------------------------- ALU -------------------------


    // ------------------------- SignExtend Unit -------------------------

    // Sign-extend unit for sign extending the immediate of the instruction
    SignExtend sign_ext (.imm_sel(imm_sel), .instr(instr), .imm_ext(imm_ext));

    // ------------------------- SignExtend Unit -------------------------


    // ------------------------- LoadExtend Unit -------------------------

    // Load-extend unit for sign-extending or zero-extending rd_data from the data memory
    LoadExtend ld_ext (.ld_ctrl(ld_ctrl), .data_in(rd_data), .data_out(rd_data_ext));

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

    logic [3:0] rmask;
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

    // ------------------------- LoadExtend Unit -------------------------


    // ------------------------- StoreExtend Unit -------------------------

    logic [4:0] s_ctrl;

    // Store-extend unit for rearranging src_data_2 before writing to the data memory
    StoreExtend s_ext (.s_ctrl(s_ctrl), .data_in(src_data_2), .data_out(wr_data));

    // ------------------------- StoreExtend Unit -------------------------

    // -------------------------------------------------------- DATAPATH --------------------------------------------------------


    // -------------------------------------------------------- CONTROL UNIT --------------------------------------------------------

    // Enumerate opcodes
    typedef enum logic [6:0] {
        LOAD       = 7'b0000011,
        IMM        = 7'b0010011,
        STORE      = 7'b0100011,
        ARITH      = 7'b0110011,
        BRANCH     = 7'b1100011,
        JUMP       = 7'b1101111,
        JUMP_REG   = 7'b1100111,
        LOAD_UIMM  = 7'b0110111,
        ADD_UIMM   = 7'b0010111
    } ops_t;
    
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic instr_valid, ctrl_instr_trap;

    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    logic load_check;
    logic imm_check;
    logic store_check;
    logic arith_check;
    logic branch_check;
    logic jump_check;
    logic jump_reg_check;
    logic load_uimm_check;
    logic add_uimm_check;

    assign load_check = &{~opcode[6:2], opcode[1:0]} & ((~funct3[2] & ~funct3[0]) | funct3[1]);
    assign imm_check = &{~opcode[6:5], opcode[4], ~opcode[3:2], opcode[1:0]} & ((funct3[1] | ~funct3[0]) | (~funct7) | (~{funct7[6], funct7[4:0]} & funct3[2]));
    assign store_check = &{~opcode[6], opcode[5], ~opcode[4:2], opcode[1:0]} & ((~funct3[2] & ~funct3[1]) | (~funct3[2] & ~funct3[0]));
    assign arith_check = &{~opcode[6], opcode[5:4], ~opcode[3:2], opcode[1:0]} & (&~{funct7[6], funct7[4:0]} & (~funct3[1] & ~(funct3[2] ^ funct3[0])) | ~funct7[5]);
    assign branch_check = &{opcode[6:5], ~opcode[4:2], opcode[1:0]} & (funct3[2] | ~funct3[1]);
    assign jump_check = &{opcode[6:5], ~opcode[4], opcode[3:0]};
    assign jump_reg_check = &{opcode[6:5], ~opcode[4:3], opcode[2:0]} & (&~funct3);
    assign load_uimm_check = &{~opcode[6], opcode[5:4], ~opcode[3], opcode[2:0]};
    assign add_uimm_check = &{~opcode[6:5], opcode[4], ~opcode[3], opcode[2:0]};

    logic [6:0] opcode_mask;
    assign opcode_mask = {7{|{load_check, imm_check, store_check, arith_check, branch_check, jump_check, jump_reg_check, load_uimm_check, add_uimm_check}}};

    // Combinational logic for determining outputs based on instruction slices
    always_comb begin
        instr_valid = 1'b1;
        ctrl_instr_trap = 1'b0;
        dst_addr_sel = 1'b0;
        addr_align = 1'b0;
        s_ctrl = 5'b010_00;

        case (opcode & opcode_mask)
            // Load instructions
            LOAD:       begin
                            wr_en = 4'b0000;

                            if (funct3 == 3'b010)
                                rf_wr_en = (result[1:0] != 2'b0) ? 1'b0 : 1'b1;
                            else if (funct3 == 3'b001 || funct3 == 3'b101)
                                rf_wr_en = (result[0] != 1'b0) ? 1'b0 : 1'b1;
                            else 
                                rf_wr_en = 1'b1;

                            src_1_sel = 1'b0;
                            src_2_sel = 1'b1;
                            addr_align = 1'b1;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'b001;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b000;
                            ld_ctrl = {funct3, result[1:0]};
                            alu_ctrl = 4'b0000;

                            if (funct3 == 3'b010)
                                ctrl_instr_trap = (result[1:0] != 2'b0) ? 1'b1 : 1'b0;
                            else if (funct3 == 3'b001 || funct3 == 3'b101)
                                ctrl_instr_trap = (result[0] != 1'b0) ? 1'b1 : 1'b0;
                            else 
                                ctrl_instr_trap = 1'b0;
                        end
            
            // Immediate instructions
            IMM:        begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b1;
                            src_1_sel = 1'b0;
                            src_2_sel = 1'b1;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'b000;
                            wr_data_sel = 2'bX;
                            imm_sel = {2'b0, (~funct3[1] & funct3[0])};
                            ld_ctrl = 5'bX;
                            alu_ctrl = {(funct3[2] & ~funct3[1] & funct3[0] & funct7[5]), funct3};
                        end
            
            // Store instructions
            STORE:      begin
                            addr_align = 1'b1;

                            if (funct3 == 3'b010)
                                wr_en = (result[1:0] != 2'b0) ? 4'b0000 : 4'b1111;
                            else if (funct3 == 3'b001)
                                wr_en = (result[0] != 1'b0) ? 4'b0000 : ((result[1] == 1'b0) ? 4'b0011 : 4'b1100);
                            else begin
                                case (result[1:0])
                                    2'b00:      wr_en = 4'b0001;
                                    2'b01:      wr_en = 4'b0010;
                                    2'b10:      wr_en = 4'b0100;
                                    2'b11:      wr_en = 4'b1000;
                                    default:    wr_en = 4'bX;
                                endcase
                            end

                            rf_wr_en = 1'b0;
                            src_1_sel = 1'b0;
                            src_2_sel = 1'b1;
                            pc_sel = 2'b00;
                            dst_addr_sel = 1'b1;
                            rd_data_sel = 3'bX;
                            wr_data_sel = funct3[1:0];
                            imm_sel = 3'b010;
                            s_ctrl = {funct3, result[1:0]};
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'b0000;

                            if (funct3 == 3'b010)
                                ctrl_instr_trap = (result[1:0] != 2'b0) ? 1'b1 : 1'b0;
                            else if (funct3 == 3'b001)
                                ctrl_instr_trap = (result[0] != 1'b0) ? 1'b1 : 1'b0;
                            else 
                                ctrl_instr_trap = 1'b0;
                        end
            
            //  Arithmetic instructions 
            ARITH:      begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b1;
                            src_1_sel = 1'b0;
                            src_2_sel = 1'b0;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'b000;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'bX;
                            ld_ctrl = 5'bX;
                            alu_ctrl = {funct7[5], funct3};
                        end            
            
            // Branch instructions
            BRANCH:     begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b0;
                            src_1_sel = 1'b0;
                            src_2_sel = 1'b0;
                            pc_sel = {1'b0, (funct3[0] ^ b_flag)};
                            dst_addr_sel = 1'b1;
                            rd_data_sel = 3'b100;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b011;
                            ld_ctrl = 5'bX;
                            alu_ctrl = {~funct3[2], 1'b0, funct3[2:1]};
                            ctrl_instr_trap = |pc_next[1:0];
                        end
            
            // Jump and link
            JUMP:       begin
                            wr_en = 4'b0000;
                            rf_wr_en = &~target[1:0];
                            src_1_sel = 1'bX;
                            src_2_sel = 1'bX;
                            pc_sel = {1'b0, &~target[1:0]};
                            rd_data_sel = 3'b010;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b100;
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'bX;
                            ctrl_instr_trap = |target[1:0];
                        end
            
            // Jump and link register
            JUMP_REG:   begin
                            wr_en = 4'b0000;
                            rf_wr_en = ~result[1];
                            src_1_sel = 1'b0;
                            src_2_sel = 1'b1;
                            pc_sel = {2{~result[1]}};
                            rd_data_sel = 3'b010;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b000;
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'b0000;
                            ctrl_instr_trap = result[1];
                        end
            
            // Load upper-immediate instructions
            LOAD_UIMM:  begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b1;
                            src_1_sel = 1'bX;
                            src_2_sel = 1'bX;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'b011;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b101;
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'bX;
                        end
            
            // Add upper-immediate to PC instruction
            ADD_UIMM:   begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b1;
                            src_1_sel = 1'b1;
                            src_2_sel = 1'b1;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'b000;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b101;
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'b0000;
                        end            
            
            // Unknown opcode (treated as NOPs - only PC is incremented)
            default:    begin
                            wr_en = 4'b0000;
                            rf_wr_en = 1'b0;
                            src_1_sel = 1'bX;
                            src_2_sel = 1'bX;
                            pc_sel = 2'b00;
                            rd_data_sel = 3'bX;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'bX;
                            ld_ctrl = 5'bX;
                            alu_ctrl = 4'bX;
                            instr_valid = 1'b0;
                            ctrl_instr_trap = 1'b1;
                        end
        endcase
    end

    // -------------------------------------------------------- CONTROL UNIT --------------------------------------------------------

endmodule