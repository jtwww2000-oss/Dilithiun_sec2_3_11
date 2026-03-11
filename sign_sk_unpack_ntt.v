`timescale 1ns / 1ps

module sign_sk_unpack_ntt (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,

    // ==========================================
    // 现有接口：SK 读取与 NTT 输出
    // ==========================================
    output reg  [9:0]    o_sk_raddr,
    input  wire [31:0]   i_sk_rdata,

    output reg  [255:0]  o_rho,
    output reg  [255:0]  o_K,
    output reg  [511:0]  o_tr,

    output reg           o_s1_we,
    output reg  [9:0]    o_s1_addr,
    output reg  [23:0]   o_s1_wdata,

    output reg           o_s2_we,
    output reg  [9:0]    o_s2_addr,
    output reg  [23:0]   o_s2_wdata,

    output reg           o_t0_we,
    output reg  [9:0]    o_t0_addr,
    output reg  [23:0]   o_t0_wdata,

    // ==========================================
    // 新增接口：A 矩阵与 SHAKE256 (u, rho') 结果
    // ==========================================
    input  wire [255:0]  i_M,            // 待签名的消息 M (设定为 256 bits，可依需修改)
    
    // A 矩阵输出 RAM 接口 (4x4个多项式 = 16 * 256 = 4096 深度)
    output reg           o_A_we,
    output reg  [11:0]   o_A_addr,
    output reg  [23:0]   o_A_wdata,

    // u 与 rho_prime 寄存器输出
    output reg  [511:0]  o_u,
    output reg  [511:0]  o_rho_prime
);

    // ==========================================
    // 参数与常量定义
    // ==========================================
    localparam [23:0] Q = 24'd8380417;

    // FSM 状态定义 (增加新状态)
    localparam S_IDLE           = 5'd0;
    localparam S_HEADER_REQ     = 5'd1;
    localparam S_HEADER_WAIT1   = 5'd2;
    localparam S_HEADER_WAIT2   = 5'd3;
    localparam S_UNPACK_CHECK   = 5'd4;
    localparam S_UNPACK_COEFF   = 5'd5;
    localparam S_UNPACK_WRITE   = 5'd6;
    localparam S_UNPACK_NEXT    = 5'd7;
    localparam S_FETCH_REQ      = 5'd8;
    localparam S_FETCH_WAIT1    = 5'd9;
    localparam S_FETCH_WAIT2    = 5'd10;
    localparam S_NTT_START      = 5'd11;
    localparam S_NTT_WAIT       = 5'd12;
    localparam S_DUMP_REQ       = 5'd13;
    localparam S_DUMP_WAIT      = 5'd14;
    localparam S_DUMP_STORE     = 5'd15;
    localparam S_DUMP_NEXT      = 5'd16;
    
    // 新增状态
    localparam S_GEN_A_START    = 5'd17;
    localparam S_GEN_A_WAIT     = 5'd18;
    localparam S_GEN_U_START    = 5'd19;
    localparam S_GEN_U_WAIT     = 5'd20;
    localparam S_GEN_RHO_START  = 5'd21;
    localparam S_GEN_RHO_WAIT   = 5'd22;
    localparam S_DONE           = 5'd23;

    reg [4:0]  state;
    reg [4:0]  ret_state;

    // 内部控制变量
    reg [5:0]  header_idx;
    reg [1:0]  mode;         
    reg [2:0]  poly_idx;     
    reg [8:0]  coeff_idx;    
    reg [8:0]  dump_idx;     
    
    reg [9:0]  sk_ptr;
    reg [63:0] bit_buf;
    reg [6:0]  bit_cnt;

    // ==========================================
    // 实例化：NTT 处理通道
    // ==========================================
    reg         ntt_start;
    wire        ntt_done;
    
    reg         ntt_ram_we_user;
    reg  [7:0]  ntt_ram_addr_user;
    reg  [23:0] ntt_ram_wdata_user;
    wire [23:0] ntt_ram_rdata_a;

    wire [7:0]  ntt_core_addr_a, ntt_core_addr_b;
    wire        ntt_core_we_a, ntt_core_we_b;
    wire [23:0] ntt_core_wdata_a, ntt_core_wdata_b;
    wire [23:0] ntt_core_rdata_b;

    wire        use_user = (state != S_NTT_START && state != S_NTT_WAIT);
    wire [7:0]  ram_addr_a_mux  = use_user ? ntt_ram_addr_user  : ntt_core_addr_a;
    wire        ram_we_a_mux    = use_user ? ntt_ram_we_user    : ntt_core_we_a;
    wire [23:0] ram_wdata_a_mux = use_user ? ntt_ram_wdata_user : ntt_core_wdata_a;

    tdpram_24x256 u_ntt_ram (
        .clk(clk), .we_a(ram_we_a_mux), .addr_a(ram_addr_a_mux), .din_a(ram_wdata_a_mux), .dout_a(ntt_ram_rdata_a),
        .we_b(ntt_core_we_b), .addr_b(ntt_core_addr_b), .din_b(ntt_core_wdata_b), .dout_b(ntt_core_rdata_b)
    );

    ntt_core #( .WIDTH(24) ) u_ntt (
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done),
        .ram_addr_a(ntt_core_addr_a), .ram_we_a(ntt_core_we_a), .ram_wdata_a(ntt_core_wdata_a), .ram_rdata_a(ntt_ram_rdata_a), 
        .ram_addr_b(ntt_core_addr_b), .ram_we_b(ntt_core_we_b), .ram_wdata_b(ntt_core_wdata_b), .ram_rdata_b(ntt_core_rdata_b) 
    );

    // ==========================================
    // 实例化：Rejsam_a 模块 (Line 5)
    // ==========================================
    reg         rejsam_start;
    wire        rejsam_valid;
    wire [22:0] rejsam_data;
    wire        rejsam_done;
    reg  [2:0]  rejsam_i;
    reg  [2:0]  rejsam_j;

    Rejsam_a u_rejsam_a (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_start       (rejsam_start),
        .i_rho         (o_rho),
        .i_row         ({5'd0, rejsam_i}),
        .i_column      ({5'd0, rejsam_j}),
        .o_coeff_valid (rejsam_valid),
        .o_coeff_data  (rejsam_data),
        .o_done        (rejsam_done)
    );

    // ==========================================
    // 实例化：SHAKE256 for u (Line 6)
    // ==========================================
    reg          shake_u_start;
    wire         shake_u_valid;
    wire [511:0] shake_u_data;

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(64),
        .ABSORB_LEN(512 + 256) // tr (512) + M (256)
    ) u_shake_u (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_u_start),
        .i_seed          ({i_M, o_tr}), // 连接 tr 和 M
        .o_busy          (),
        .i_squeeze_req   (1'b0),        // 只获取第一块 64 bytes 即可
        .o_squeeze_valid (shake_u_valid),
        .o_squeeze_data  (shake_u_data)
    );

    // ==========================================
    // 实例化：SHAKE256 for rho_prime (Line 8)
    // ==========================================
    reg          shake_rho_start;
    wire         shake_rho_valid;
    wire [511:0] shake_rho_data;

    SHAKE256 #(
        .OUTPUT_LEN_BYTES(64),
        .ABSORB_LEN(256 + 256 + 512) // K (256) + zeros (256) + u (512)
    ) u_shake_rho_prime (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_rho_start),
        .i_seed          ({o_u, 256'd0, o_K}), // [K, zeros, u]
        .o_busy          (),
        .i_squeeze_req   (1'b0),
        .o_squeeze_valid (shake_rho_valid),
        .o_squeeze_data  (shake_rho_data)
    );

    // 预处理映射逻辑
    wire [3:0]  target_bits = (mode == 2) ? 4'd13 : 4'd3;
    wire [12:0] raw_val     = bit_buf[12:0] & ((1 << target_bits) - 1);
    wire [23:0] pre_ntt_val = (mode == 2) ? 
                         ((raw_val <= 13'd4096) ? (24'd4096 - raw_val) : (Q + 24'd4096 - raw_val)) :
                         ((raw_val <= 13'd2)    ? (24'd2 - raw_val)    : (Q + 24'd2 - raw_val));

    // ==========================================
    // 主状态机
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0;
            o_s1_we <= 0; o_s2_we <= 0; o_t0_we <= 0;
            ntt_start <= 0; ntt_ram_we_user <= 0;
            bit_buf <= 0; bit_cnt <= 0; sk_ptr <= 0;
            o_rho <= 0; o_K <= 0; o_tr <= 0;
            o_A_we <= 0; o_A_addr <= 0; o_u <= 0; o_rho_prime <= 0;
            rejsam_start <= 0; shake_u_start <= 0; shake_rho_start <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; o_A_we <= 0; o_A_addr <= 0;
                    if (start) begin
                        header_idx <= 0;
                        state <= S_HEADER_REQ;
                    end
                end

                // --- 1. 读取头部的 rho, K, tr ---
                S_HEADER_REQ: begin
                    o_sk_raddr <= header_idx;
                    state <= S_HEADER_WAIT1;
                end
                S_HEADER_WAIT1: state <= S_HEADER_WAIT2; 
                S_HEADER_WAIT2: begin
                    if (header_idx < 8)       o_rho[header_idx*32 +: 32] <= i_sk_rdata;
                    else if (header_idx < 16) o_K[(header_idx-8)*32 +: 32] <= i_sk_rdata;
                    else                      o_tr[(header_idx-16)*32 +: 32] <= i_sk_rdata;

                    if (header_idx == 31) begin
                        state <= S_UNPACK_CHECK;
                        mode <= 0; poly_idx <= 0; sk_ptr <= 10'd32;
                        bit_buf <= 0; bit_cnt <= 0;
                    end else begin
                        header_idx <= header_idx + 1;
                        state <= S_HEADER_REQ;
                    end
                end

                // --- 2. 判断当前要解包哪个多项式 ---
                S_UNPACK_CHECK: begin
                    if (poly_idx == 4) begin
                        if (mode == 0) begin mode <= 1; poly_idx <= 0; end      
                        else if (mode == 1) begin mode <= 2; poly_idx <= 0; end 
                        else state <= S_GEN_A_START; // t0 完毕后，跳转到生成矩阵 A
                    end else begin
                        coeff_idx <= 0;
                        state <= S_UNPACK_COEFF;
                    end
                end

                // --- 3. 提取系数预处理 & 写入 NTT RAM ---
                S_UNPACK_COEFF: begin
                    if (bit_cnt < target_bits) begin
                        o_sk_raddr <= sk_ptr;
                        state <= S_FETCH_WAIT1;
                        ret_state <= S_UNPACK_COEFF;
                    end else begin
                        state <= S_UNPACK_WRITE;
                    end
                end
                S_UNPACK_WRITE: begin
                    ntt_ram_we_user <= 1;
                    ntt_ram_addr_user <= coeff_idx[7:0];
                    ntt_ram_wdata_user <= pre_ntt_val;
                    bit_buf <= bit_buf >> target_bits;
                    bit_cnt <= bit_cnt - target_bits;
                    state <= S_UNPACK_NEXT;
                end
                S_UNPACK_NEXT: begin
                    ntt_ram_we_user <= 0; 
                    if (coeff_idx == 255) state <= S_NTT_START;
                    else begin coeff_idx <= coeff_idx + 1; state <= S_UNPACK_COEFF; end
                end

                S_FETCH_WAIT1: state <= S_FETCH_WAIT2;
                S_FETCH_WAIT2: begin
                    bit_buf <= bit_buf | ({32'd0, i_sk_rdata} << bit_cnt);
                    bit_cnt <= bit_cnt + 32;
                    sk_ptr <= sk_ptr + 1;
                    state <= ret_state;
                end

                // --- 4. 触发 NTT ---
                S_NTT_START: begin ntt_start <= 1; state <= S_NTT_WAIT; end
                S_NTT_WAIT: begin
                    ntt_start <= 0;
                    if (ntt_done) begin dump_idx <= 0; state <= S_DUMP_REQ; end
                end

                // --- 5. 从 NTT RAM 读取结果 ---
                S_DUMP_REQ: begin
                    ntt_ram_we_user <= 0; ntt_ram_addr_user <= dump_idx[7:0]; state <= S_DUMP_WAIT;
                end
                S_DUMP_WAIT: state <= S_DUMP_STORE;
                S_DUMP_STORE: begin
                    if (mode == 0) begin o_s1_we <= 1; o_s1_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_s1_wdata <= ntt_ram_rdata_a; end
                    if (mode == 1) begin o_s2_we <= 1; o_s2_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_s2_wdata <= ntt_ram_rdata_a; end
                    if (mode == 2) begin o_t0_we <= 1; o_t0_addr <= {poly_idx[1:0], dump_idx[7:0]}; o_t0_wdata <= ntt_ram_rdata_a; end
                    state <= S_DUMP_NEXT;
                end
                S_DUMP_NEXT: begin
                    o_s1_we <= 0; o_s2_we <= 0; o_t0_we <= 0;
                    if (dump_idx == 255) begin
                        poly_idx <= poly_idx + 1;
                        state <= S_UNPACK_CHECK;
                    end else begin
                        dump_idx <= dump_idx + 1;
                        state <= S_DUMP_REQ;
                    end
                end

// ===============================================
                // --- 6. 生成矩阵 A ---
                // ===============================================
                S_GEN_A_START: begin
                    rejsam_i <= 0;
                    rejsam_j <= 0;
                    rejsam_start <= 1;
                    o_A_addr <= 0;
                    o_A_we <= 0;
                    state <= S_GEN_A_WAIT;
                end

                S_GEN_A_WAIT: begin
                    // 接收有效系数并写入 RAM (单周期流水线)
                    o_A_we <= rejsam_valid;
                    if (rejsam_valid) o_A_wdata <= {1'b0, rejsam_data};
                    if (o_A_we) o_A_addr <= o_A_addr + 1;

                    // 修复：加入 !rejsam_start 防止 double-trigger
                    if (rejsam_done && !rejsam_start) begin
                        if (rejsam_j == 3) begin
                            if (rejsam_i == 3) begin
                                state <= S_GEN_U_START; // 全矩阵完毕，下一步
                                rejsam_start <= 0;      // 确保清零
                            end else begin
                                rejsam_i <= rejsam_i + 1;
                                rejsam_j <= 0;          // 修复：列索引重置
                                rejsam_start <= 1;      // 启动下一个
                            end
                        end else begin
                            rejsam_j <= rejsam_j + 1;
                            rejsam_start <= 1;
                        end
                    end else begin
                        rejsam_start <= 0; // 产生 1-cycle 的启动脉冲后自动拉低
                    end
                end

                // ===============================================
                // --- 7. 生成 mu (u) ---
                // ===============================================
                S_GEN_U_START: begin
                    o_A_we <= 0; // 确保 RAM 写使能拉低
                    shake_u_start <= 1;
                    state <= S_GEN_U_WAIT;
                end

                S_GEN_U_WAIT: begin
                    shake_u_start <= 0;
                    if (shake_u_valid) begin
                        o_u <= shake_u_data;
                        state <= S_GEN_RHO_START;
                    end
                end

                // ===============================================
                // --- 8. 生成 rho_prime ---
                // ===============================================
                S_GEN_RHO_START: begin
                    shake_rho_start <= 1;
                    state <= S_GEN_RHO_WAIT;
                end

                S_GEN_RHO_WAIT: begin
                    shake_rho_start <= 0;
                    if (shake_rho_valid) begin
                        o_rho_prime <= shake_rho_data;
                        state <= S_DONE;
                    end
                end

                // 结束态
                S_DONE: begin
                    done <= 1;
                    if (!start) state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule