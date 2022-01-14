
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
    logic [2:0] ld_ctrl;
    logic [3:0] alu_ctrl;

    logic [6:0] opcode;
    assign opcode = (reset == 1'b1) ? '0 : instr[6:0];


    // -------------------------------------------------------- RVFI CONTROL --------------------------------------------------------

    // Instruction Metadata
    // assign rvfi_valid = ~reset & instr_valid;
    // assign rvfi_valid = ~reset & ~rvfi_trap;
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
    // assign rvfi_pc_rdata = pc_last;
    assign rvfi_pc_rdata = pc;
    assign rvfi_pc_wdata = pc_next;
    // assign rvfi_pc_wdata = pc;

    // Memory Access
    assign rvfi_mem_addr = addr;
    assign rvfi_mem_wmask = wr_en;
    assign rvfi_mem_rdata = rd_data;
    assign rvfi_mem_wdata = wr_data;    

    // -------------------------------------------------------- RVFI CONTROL --------------------------------------------------------


    // -------------------------------------------------------- DATAPATH --------------------------------------------------------
    
    logic [31:0] pc_last;                                       // 
    logic [63:0] instr_index;                                   // 
    logic [4:0] rf_dst_addr, rf_src_addr_1, rf_src_addr_2;      // 
    logic [31:0] dst_data, rf_dst_data;                         // 32-bit intermediate and final nets connecting to register file
    logic [31:0] imm_ext, rd_data_ext;                          // 32-bit extended signals
    logic [31:0] pc_next, pc_plus_4, target;                    // 32-bit internal signals for PC flip-flop
    logic [31:0] src_data_1, src_data_2;                        // 32-bit internal signals for register file
    logic [31:0] alu_in_1, alu_in_2;                            // 32-bit internal signals for ALU

    // Write-Data decorder for selecting wr_data to write to the data memory
    assign wr_data = src_data_2;

    // Adder for updating the PC for non-branch instructions
    assign pc_plus_4 = signed'(pc) + signed'(32'b0100);
    
    // Adder for updating the PC for B-, J-, or U-type instructions
    logic target_carry;
    // assign {target_carry, target} = signed'(pc) + signed'(imm_ext);
    assign {target_carry, target} = pc + imm_ext;

    // 
    // assign rf_dst_addr = instr[11:7];
    logic rd_addr_sel;
    always_comb begin
        case (rd_addr_sel)
            1'b0:       rf_dst_addr = instr[11:7];
            1'b1:       rf_dst_addr = 32'b0;
            default:    rf_dst_addr = 'X;
        endcase
    end

    assign rf_dst_data = dst_data;
    assign rf_src_addr_1 = instr[19:15];
    assign rf_src_addr_2 = instr[24:20];

    // Update PC at positive clock edge
    always_ff @(posedge clk) begin
        if (reset == 1'b1) begin
            pc <= '0;
            pc_last <= '0;
            instr_index <= '0;
        end
        else begin
            pc <= pc_next;
            pc_last <= pc;
            // instr_index <= (instr_valid == 1'b1) ? instr_index + 64'b1 : instr_index;
            instr_index <= (rvfi_valid == 1'b1) ? instr_index + 64'b1 : instr_index;
            // instr_index <= (rvfi_trap == 1'b0) ? instr_index + 64'b1 : instr_index;
            // instr_index <= instr_index + 64'b1;
        end
    end

    // 4-to-1 MUX instance for selecting pc_next source
    always_comb begin
        case (pc_sel)
            2'b00:      pc_next = pc_plus_4;
            2'b01:      pc_next = target;
            2'b10:      pc_next = addr;
            2'b11:      pc_next = {addr[31:1], 1'b0};
            default:    pc_next = 'X;
        endcase
    end

    // 2-to-1 MUX instances for selecting ALU input sources
    always_comb begin
        case (src_1_sel)
            1'b0:      alu_in_1 = src_data_1;
            1'b1:      alu_in_1 = pc;
            default:   alu_in_1 = 'X;
        endcase
    end

    always_comb begin
        case (src_2_sel)
            1'b0:      alu_in_2 = src_data_2;
            1'b1:      alu_in_2 = imm_ext;
            default:   alu_in_2 = 'X;
        endcase
    end

    // 4-to-1 MUX instance for selecting data source for destination register writes
    always_comb begin
        case (rd_data_sel)
            3'b000:     dst_data = addr;
            3'b001:     dst_data = rd_data_ext;
            3'b010:     dst_data = pc_plus_4;
            3'b011:     dst_data = imm_ext;
            3'b100:     dst_data = 32'b0;
            default:    dst_data = 'X;
        endcase
    end


    // ------------------------- Register File -------------------------
    
    // 32, 32-bit registers
    logic [31:0] mem [0:31];

    // Data write
    always_ff @(posedge clk) begin
        if (rf_wr_en == 1'b1)
            mem[rf_dst_addr] <= (rf_dst_addr == 32'b0) ? 32'b0 : rf_dst_data;
        else
            mem[rf_dst_addr] <= mem[rf_dst_addr];
    end

    // Data read
    always_comb begin
        src_data_1 = (rf_src_addr_1 == 32'b0) ? 32'b0 : mem[rf_src_addr_1];
        src_data_2 = (rf_src_addr_2 == 32'b0) ? 32'b0 : mem[rf_src_addr_2];
    end

    // ------------------------- Register File -------------------------


    // ------------------------- ALU -------------------------

    // Enumerate possible alu_ctrl codes
    typedef enum logic [3:0] {
        ALU_ADD  = 4'b0000,
        ALU_SUB  = 4'b1000,
        ALU_SLL  = 4'b0001,
        ALU_SLT  = 4'b0010,
        ALU_SLTU = 4'b0011,
        ALU_XOR  = 4'b0100,
        ALU_SRL  = 4'b0101,
        ALU_SRA  = 4'b1101,
        ALU_OR   = 4'b0110,
        ALU_AND  = 4'b0111
    } alu_ctrls_t;

    logic carry;

    // 
    always_comb begin
        carry = 1'b0;
        case (alu_ctrl)
            // Add
            ALU_ADD:    begin
                            {carry, addr} = signed'(alu_in_1) + signed'(alu_in_2);
                            b_flag = 1'bX;
                        end
                        
            // Subtract
            ALU_SUB:    begin
                            addr = signed'(alu_in_1) - signed'(alu_in_2);
                            // addr = alu_in_1 - alu_in_2;
                            b_flag = (addr == '0);
                        end
            
            // Shift left logical
            ALU_SLL:    begin
                            addr = signed'(alu_in_1) << signed'(alu_in_2[4:0]);
                            // addr = alu_in_1 << alu_in_2;
                            b_flag = 1'bX;
                        end
                        
            // Set less than
            ALU_SLT:    begin
                            addr = (signed'(alu_in_1) < signed'(alu_in_2));
                            // addr = alu_in_1 < alu_in_2;
                            b_flag = addr[0];
                        end

            // Set less than unsigned
            ALU_SLTU:   begin
                            addr = (unsigned'(alu_in_1) < unsigned'(alu_in_2));
                            b_flag = addr[0];
                        end
            
            // Exclusive-OR
            ALU_XOR:    begin
                            // addr = signed'(alu_in_1) ^ signed'(alu_in_2);
                            addr = alu_in_1 ^ alu_in_2;
                            b_flag = 1'bX;
                        end
            
            // Shift right logical
            ALU_SRL:    begin
                            addr = signed'(alu_in_1) >> signed'(alu_in_2[4:0]);
                            // addr = alu_in_1 >> alu_in_2;
                            b_flag = 1'bX;
                        end
            
            // Shift right arithmetic
            ALU_SRA:    begin
                            addr = signed'(alu_in_1) >>> signed'(alu_in_2[4:0]);
                            // addr = alu_in_1 >>> alu_in_2;
                            b_flag = 1'bX;
                        end
            
            // OR 
            ALU_OR:     begin
                            // addr = signed'(alu_in_1) | signed'(alu_in_2);
                            addr = alu_in_1 | alu_in_2;
                            b_flag = 1'bX;
                        end
                        
            // AND 
            ALU_AND:    begin
                            // addr = signed'(alu_in_1) & signed'(alu_in_2);
                            addr = alu_in_1 & alu_in_2;
                            b_flag = 1'bX;
                        end
                        
            // Unknwon alu_ctrl
            default:    begin
                            addr = 'X;
                            b_flag = 1'bX;
                        end
        endcase
    end

    // ------------------------- ALU -------------------------


    // ------------------------- SignExtend Unit -------------------------

    // Enumerate instruction types
    typedef enum logic [2:0] {
        I_TYPE   = 3'b000,
        I_TYPE_U = 3'b001,
        S_TYPE   = 3'b010,
        B_TYPE   = 3'b011,
        J_TYPE   = 3'b100,
        U_TYPE   = 3'b101
    } instr_types_t;
    
    // 
    always_comb begin
        case (imm_sel)
            I_TYPE:     imm_ext = {{20{instr[31]}}, instr[31:20]};
            I_TYPE_U:   imm_ext = {27'b0, instr[24:20]};
            S_TYPE:     imm_ext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            B_TYPE:     imm_ext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            J_TYPE:     imm_ext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            U_TYPE:     imm_ext = {instr[31:12], 12'b0};
            default:    imm_ext = 'X;
        endcase
    end

    // ------------------------- SignExtend Unit -------------------------


    // ------------------------- LoadExtend Unit -------------------------

    // Enumerate load instruction funct3's
    typedef enum logic [2:0] {
        LD_BYTE   = 3'b000,
        LD_HALF   = 3'b001,
        LD_WORD   = 3'b010,
        LD_BYTE_U = 3'b100,
        LD_HALF_U = 3'b101
    } ld_funct3s_t;
    
    // 
    always_comb begin
        case (ld_ctrl)
            LD_BYTE:    rd_data_ext = {{24{rd_data[7]}}, rd_data[7:0]};
            LD_HALF:    rd_data_ext = {{16{rd_data[7]}}, rd_data[15:0]};
            LD_WORD:    rd_data_ext = rd_data;
            LD_BYTE_U:  rd_data_ext = {24'b0, rd_data[7:0]};
            LD_HALF_U:  rd_data_ext = {16'b0, rd_data[15:0]};
            default:    rd_data_ext = 'X;
        endcase
    end

    always_comb begin
        case (ld_ctrl)
            LD_BYTE:    rvfi_mem_rmask = 4'b0001;
            LD_HALF:    rvfi_mem_rmask = 4'b0011;
            LD_WORD:    rvfi_mem_rmask = 4'b1111;
            LD_BYTE_U:  rvfi_mem_rmask = 4'b0001;
            LD_BYTE_U:  rvfi_mem_rmask = 4'b0011;
            default:    rvfi_mem_rmask = 4'b0000;
        endcase
    end

    // ------------------------- LoadExtend Unit -------------------------

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

    // Combinational logic for determining outputs based on instruction slices
    always_comb begin
        instr_valid = 1'b1;
        ctrl_instr_trap = 1'b0;
        rd_addr_sel = 1'b0;

        case (opcode)
            // Load instructions
            LOAD:       begin
                            if (funct3 == 3'b011 || funct3 == 3'b110 || funct3 == 3'b111) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = 4'b0000;
                                // rf_wr_en = 1'b1;
                                rf_wr_en = (addr[1:0] != 2'b0) ? 1'b0 : 1'b1;
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b1;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'b01;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'b000;
                                ld_ctrl = funct3;
                                alu_ctrl = 4'b0000;
                                ctrl_instr_trap = (addr[1:0] != 2'b0) ? 1'b1 : 1'b0;
                            end
                        end
            
            // Immediate instructions
            IMM:        begin
                            if (funct3 == 3'b001 && funct7 != 7'b0) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else if (funct3 == 3'b101 && (funct7 != 7'b0 && funct7 != 7'b0100000)) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b1;
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b1;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'b000;
                                wr_data_sel = 2'bX;
                                imm_sel = {2'b0, (~funct3[1] & funct3[0])};
                                ld_ctrl = 3'bX;
                                alu_ctrl = {(funct3[2] & ~funct3[1] & funct3[0] & funct7[5]), funct3};
                            end
                        end
            
            // Store instructions
            STORE:      begin
                            if (funct3 != 3'b000 && funct3 != 3'b001 && funct3 != 3'b010) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = (addr[1:0] != 2'b0) ? 1'b0 : {funct3[1], funct3[1], (funct3[1] | funct3[0]), 1'b1};
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b1;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = funct3[1:0];
                                imm_sel = 3'b010;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'b0000;
                                ctrl_instr_trap = (addr[1:0] != 2'b0) ? 1'b1 : 1'b0;
                            end
                        end
            
            //  Arithmetic instructions 
            ARITH:      begin
                            if (funct3 == 3'b000 && (funct7 != 7'b0 && funct7 != 7'b0100000)) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else if (funct3 == 3'b101 && (funct7 != 7'b0 && funct7 != 7'b0100000)) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else if ((funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011 || funct3 == 3'b100 || funct3 == 3'b110 || funct3 == 3'b111) && funct7 != 7'b0) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b1;
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b0;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'b000;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = {funct7[5], funct3};
                            end
                            
                        end            
            
            // Branch instructions
            BRANCH:     begin
                            if (funct3 == 3'b010 || funct3 == 3'b011) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b0;
                                pc_sel = {1'b0, (funct3[0] ^ b_flag)};
                                rd_addr_sel = 1'b1;
                                rd_data_sel = 3'b100;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'b011;
                                ld_ctrl = 3'bX;
                                alu_ctrl = {~funct3[2], 1'b0, funct3[2:1]};
                                ctrl_instr_trap = |pc_next[1:0];
                            end
                        end
            
            // Jump and link
            JUMP:       begin
                            wr_en = 4'b0000;
                            rf_wr_en = |target[1:0];
                            src_1_sel = 1'bX;
                            src_2_sel = 1'bX;
                            pc_sel = {1'b0, &~target[1:0]};
                            rd_data_sel = 3'b010;
                            wr_data_sel = 2'bX;
                            imm_sel = 3'b100;
                            ld_ctrl = 3'bX;
                            alu_ctrl = 4'bX;
                            ctrl_instr_trap = |target[1:0];
                        end
            
            // Jump and link register
            JUMP_REG:   begin
                            if (funct3 != 3'b000) begin
                                wr_en = 4'b0000;
                                rf_wr_en = 1'b0;
                                src_1_sel = 1'bX;
                                src_2_sel = 1'bX;
                                pc_sel = 2'b00;
                                rd_data_sel = 3'bX;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'bX;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'bX;
                                instr_valid = 1'b0;
                                ctrl_instr_trap = 1'b1;
                            end
                            else begin
                                wr_en = 4'b0000;
                                rf_wr_en = ~addr[1];
                                src_1_sel = 1'b0;
                                src_2_sel = 1'b1;
                                pc_sel = {2{~addr[1]}};
                                rd_data_sel = 3'b010;
                                wr_data_sel = 2'bX;
                                imm_sel = 3'b000;
                                ld_ctrl = 3'bX;
                                alu_ctrl = 4'b0000;
                                ctrl_instr_trap = addr[1];
                            end

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
                            ld_ctrl = 3'bX;
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
                            ld_ctrl = 3'bX;
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
                            ld_ctrl = 3'bX;
                            alu_ctrl = 4'bX;
                            instr_valid = 1'b0;
                            ctrl_instr_trap = 1'b1;
                        end
        endcase
    end

    // -------------------------------------------------------- CONTROL UNIT --------------------------------------------------------

endmodule