`timescale 1ns / 1ps

module sign_sk_unpacker (
    input  wire           clk,
    input  wire           rst_n,
    input  wire           i_start,
    input  wire [2:0]     security_level,
    
    // 输入的私钥长总线
    input  wire [39167:0] i_sk,

    // --- 1. 参数输出 ---
    output reg  [3:0]     o_k,
    output reg  [3:0]     o_l,
    output reg  [3:0]     o_eta,
    output reg  [19:0]    o_y1,
    output reg  [18:0]    o_y2,
    output reg  [6:0]     o_omega,
    output reg  [5:0]     o_tau,
    output reg  [8:0]     o_lambda,
    output reg  [7:0]     o_beta,

    // --- 2. 头部信息输出 ---
    output reg  [255:0]   o_rho,
    output reg  [255:0]   o_K_key,
    output reg  [511:0]   o_tr,

    // --- 3. RAM 写入与分发接口 ---
    // 通过这三个 WE 将数据路由到不同的 RAM 中
    output reg            o_we_s1,
    output reg            o_we_s2,
    output reg            o_we_t0,
    output wire [7:0]     o_row_idx,        // 多项式索引 (对应 MATLAB 矩阵的行-1)
    output wire [7:0]     o_col_idx,        // 系数索引 (对应 MATLAB 矩阵的列-1)
    
    // 【核心新增】直接来自 bin2dec_matrix 的输出，等效于 _pre_ntt 矩阵！
    output wire [23:0]    o_pre_ntt_data,   
    
    // 经过 mod(eta - s) 或 mod(2^12 - t0) 运算后的结果，送给 NTT 模块使用
    output reg  [23:0]    o_post_mod_data,  

    // --- 控制信号 ---
    output reg            o_done
);

    // Dilithium 模数 Q
    localparam [23:0] Q = 24'd8380417;

    // 顶层 FSM 状态定义 (使用 4-bit)
    localparam S_IDLE      = 4'd0;
    localparam S_HEADER    = 4'd1;
    localparam S_START_S1  = 4'd2;
    localparam S_WAIT_S1   = 4'd3;
    localparam S_START_S2  = 4'd4;
    localparam S_WAIT_S2   = 4'd5;
    localparam S_START_T0  = 4'd6;
    localparam S_WAIT_T0   = 4'd7;
    localparam S_DONE      = 4'd8;

    reg [3:0]  state;

    // 动态参数寄存器
    reg [4:0]  s_width;
    reg [15:0] s1_elements;
    reg [15:0] s2_elements;
    reg [15:0] t0_elements;
    
    reg [31:0] s1_offset;
    reg [31:0] s2_offset;
    reg [31:0] t0_offset;

    // =========================================================================
    // 实例化底层的 bin2dec_matrix 模块
    // =========================================================================
    reg            bin2dec_start;
    reg  [4:0]     bin2dec_width;
    reg  [15:0]    bin2dec_num_elements;
    wire [39167:0] bin2dec_array;
    
    wire           bin2dec_we;
    wire           bin2dec_done;

    // 巧妙利用偏移截取 i_sk (等效于 MATLAB 里切片 sk(1025:end))
    assign bin2dec_array = (state == S_START_S1 || state == S_WAIT_S1) ? (i_sk >> s1_offset) :
                           (state == S_START_S2 || state == S_WAIT_S2) ? (i_sk >> s2_offset) :
                           (state == S_START_T0 || state == S_WAIT_T0) ? (i_sk >> t0_offset) : i_sk;

    bin2dec_matrix #(
        .MAX_ARRAY_BITS(39168), 
        .MAX_DATA_WIDTH(24)
    ) u_bin2dec_matrix (
        .clk(clk),
        .rst_n(rst_n),
        .i_start(bin2dec_start),
        .i_width(bin2dec_width),
        .i_num_elements(bin2dec_num_elements),
        .i_bit_array(bin2dec_array),
        
        .o_we(bin2dec_we),
        .o_row_idx(o_row_idx),
        .o_col_idx(o_col_idx),
        
        // ★ 直接连出，这个就是纯解包数据 (s1_pre_ntt, s2_pre_ntt, t0_pre_ntt)
        .o_data(o_pre_ntt_data), 
        
        .o_done(bin2dec_done)
    );

    // =========================================================================
    // 组合逻辑：基于解包数据的 Mod 预处理
    // =========================================================================
    // S 取模逻辑: mod(eta - s, Q)
    wire [23:0] s_mod_q = (o_pre_ntt_data <= {20'd0, o_eta}) ? 
                          ({20'd0, o_eta} - o_pre_ntt_data) : 
                          ({20'd0, o_eta} + Q - o_pre_ntt_data);

    // T0 取模逻辑: mod(2^12 - t0, Q)
    wire [23:0] t0_mod_q = (24'd4096 >= o_pre_ntt_data) ? 
                           (24'd4096 - o_pre_ntt_data) : 
                           (24'd4096 + Q - o_pre_ntt_data);

    // 动态分发写使能与取模数据
    always @(*) begin
        o_we_s1 = 1'b0;
        o_we_s2 = 1'b0;
        o_we_t0 = 1'b0;
        o_post_mod_data = 24'd0;
        
        if (bin2dec_we) begin
            if (state == S_WAIT_S1) begin
                o_we_s1 = 1'b1;
                o_post_mod_data = s_mod_q;
            end else if (state == S_WAIT_S2) begin
                o_we_s2 = 1'b1;
                o_post_mod_data = s_mod_q;
            end else if (state == S_WAIT_T0) begin
                o_we_t0 = 1'b1;
                o_post_mod_data = t0_mod_q;
            end
        end
    end

    // =========================================================================
    // 主控制状态机
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_done <= 0;
            bin2dec_start <= 0;
            o_rho <= 0; o_K_key <= 0; o_tr <= 0;
        end else begin
            bin2dec_start <= 0; // 默认形成单时钟脉冲

            case (state)
                S_IDLE: begin
                    o_done <= 0;
                    if (i_start) begin
                        // 1. 初始化参数与各段数据在 i_sk 中的偏移位置
                        s1_offset <= 32'd1024; // rho(256)+K(256)+tr(512) = 1024
                        
                        case (security_level)
                            3'd2: begin 
                                o_k <= 4; o_l <= 4; o_eta <= 2; s_width <= 3;
                                o_y1 <= 20'd131072; o_y2 <= 19'd95232; 
                                o_omega <= 80; o_tau <= 39; o_lambda <= 128; o_beta <= 78;
                                s1_elements <= 16'd1024; s2_elements <= 16'd1024; t0_elements <= 16'd1024;
                                s2_offset <= 32'd1024 + (32'd4 * 32'd256 * 32'd3);
                                t0_offset <= 32'd1024 + (32'd4 * 32'd256 * 32'd3) + (32'd4 * 32'd256 * 32'd3);
                            end
                            3'd3: begin 
                                o_k <= 6; o_l <= 5; o_eta <= 4; s_width <= 4;
                                o_y1 <= 20'd524288; o_y2 <= 19'd261888; 
                                o_omega <= 55; o_tau <= 49; o_lambda <= 192; o_beta <= 196;
                                s1_elements <= 16'd1280; s2_elements <= 16'd1536; t0_elements <= 16'd1536;
                                s2_offset <= 32'd1024 + (32'd5 * 32'd256 * 32'd4);
                                t0_offset <= 32'd1024 + (32'd5 * 32'd256 * 32'd4) + (32'd6 * 32'd256 * 32'd4);
                            end
                            3'd5: begin 
                                o_k <= 8; o_l <= 7; o_eta <= 2; s_width <= 3;
                                o_y1 <= 20'd524288; o_y2 <= 19'd261888; 
                                o_omega <= 75; o_tau <= 60; o_lambda <= 256; o_beta <= 120;
                                s1_elements <= 16'd1792; s2_elements <= 16'd2048; t0_elements <= 16'd2048;
                                s2_offset <= 32'd1024 + (32'd7 * 32'd256 * 32'd3);
                                t0_offset <= 32'd1024 + (32'd7 * 32'd256 * 32'd3) + (32'd8 * 32'd256 * 32'd3);
                            end
                            default: begin 
                                o_k <= 4; o_l <= 4; o_eta <= 2; s_width <= 3;
                                s1_elements <= 1024; s2_elements <= 1024; t0_elements <= 1024;
                            end
                        endcase
                        state <= S_HEADER;
                    end
                end

                S_HEADER: begin
                    // 2. 提取头部
                    o_rho   <= i_sk[255:0];
                    o_K_key <= i_sk[511:256];
                    o_tr    <= i_sk[1023:512];
                    state   <= S_START_S1;
                end

                // ------------------------------------
                // 调度 bin2dec_matrix: 解包 s1
                // ------------------------------------
                S_START_S1: begin
                    bin2dec_width <= s_width;
                    bin2dec_num_elements <= s1_elements;
                    bin2dec_start <= 1'b1; 
                    state <= S_WAIT_S1;
                end
                S_WAIT_S1: begin
                    if (bin2dec_done) state <= S_START_S2;
                end

                // ------------------------------------
                // 调度 bin2dec_matrix: 解包 s2
                // ------------------------------------
                S_START_S2: begin
                    bin2dec_width <= s_width;
                    bin2dec_num_elements <= s2_elements;
                    bin2dec_start <= 1'b1; 
                    state <= S_WAIT_S2;
                end
                S_WAIT_S2: begin
                    if (bin2dec_done) state <= S_START_T0;
                end

                // ------------------------------------
                // 调度 bin2dec_matrix: 解包 t0
                // ------------------------------------
                S_START_T0: begin
                    bin2dec_width <= 5'd13; // t0 的位宽固定为 13
                    bin2dec_num_elements <= t0_elements;
                    bin2dec_start <= 1'b1; 
                    state <= S_WAIT_T0;
                end
                S_WAIT_T0: begin
                    if (bin2dec_done) state <= S_DONE;
                end

                // ------------------------------------
                // 结束阶段
                // ------------------------------------
                S_DONE: begin
                    o_done <= 1'b1;
                    if (!i_start) state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule