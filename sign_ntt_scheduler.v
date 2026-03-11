`timescale 1ns / 1ps

module sign_ntt_scheduler (
    input  wire           clk,
    input  wire           rst_n,
    input  wire           i_start,
    input  wire [2:0]     security_level,
    input  wire [39167:0] i_sk,

    output reg            o_done
);

    // =========================================================================
    // 0. 前置连线声明 (修复隐式声明与多维数组报错的核心)
    // =========================================================================
    wire [23:0] work_dout_a;
    wire [23:0] work_dout_b;
    
    wire [23:0] s1_dout_b [0:7];
    wire [23:0] s2_dout_b [0:7];
    wire [23:0] t0_dout_b [0:7];
    
    wire [255:0] rho, K_key;
    wire [511:0] tr;

    wire [7:0] ntt_addr_a, ntt_addr_b;
    wire       ntt_we_a, ntt_we_b;
    wire [23:0] ntt_wdata_a, ntt_wdata_b;

    // =========================================================================
    // 1. 实例化解包模块 (Unpacker)
    // =========================================================================
    wire         unpack_done;
    wire [3:0]   k, l, eta;
    wire [19:0]  y1; wire [18:0] y2; wire [6:0] omega; wire [5:0] tau; wire [8:0] lambda; wire [7:0] beta;

    wire         unpack_we_s1, unpack_we_s2, unpack_we_t0;
    wire [7:0]   unpack_row_idx; 
    wire [7:0]   unpack_col_idx; 
    wire [23:0]  unpack_pre_ntt_data, unpack_post_mod_data;

    reg          unpack_start;

    sign_sk_unpacker u_unpacker (
        .clk(clk), .rst_n(rst_n), .i_start(unpack_start), .security_level(security_level), .i_sk(i_sk),
        .o_k(k), .o_l(l), .o_eta(eta), .o_y1(y1), .o_y2(y2), .o_omega(omega), .o_tau(tau), .o_lambda(lambda), .o_beta(beta),
        // [修复] 补全被遗漏的端口
        .o_rho(rho), .o_K_key(K_key), .o_tr(tr),
        
        .o_we_s1(unpack_we_s1), .o_we_s2(unpack_we_s2), .o_we_t0(unpack_we_t0),
        .o_row_idx(unpack_row_idx), .o_col_idx(unpack_col_idx),
        .o_pre_ntt_data(unpack_pre_ntt_data), .o_post_mod_data(unpack_post_mod_data),
        .o_done(unpack_done)
    );

    // =========================================================================
    // 2. 核心状态机与控制寄存器
    // =========================================================================
    localparam S_IDLE          = 4'd0;
    localparam S_UNPACK        = 4'd1;
    localparam S_COPY_IN_INIT  = 4'd2; // 从 Bank 搬运到 NTT 工作 RAM
    localparam S_COPY_IN_RUN   = 4'd3;
    localparam S_COPY_IN_WAIT  = 4'd4;
    localparam S_NTT_START     = 4'd5; // 启动 NTT
    localparam S_NTT_RUN       = 4'd6;
    localparam S_COPY_OUT_INIT = 4'd7; // 从 NTT 工作 RAM 导回 (Dump) Bank
    localparam S_COPY_OUT_RUN  = 4'd8;
    localparam S_COPY_OUT_WAIT = 4'd9;
    localparam S_NEXT_POLY     = 4'd10;
    localparam S_DONE          = 4'd11;

    reg [3:0] state;
    reg [4:0] p_cnt;            // 当前正在处理的全局多项式编号
    reg [8:0] coeff_cnt;        // 用于搬运数据的系数计数器 (0~255)
    reg [7:0] write_addr_reg;   // 写回地址
    reg       copy_read_req;    // 搬运读请求
    reg       copy_valid;       // 读数据有效 (延迟 1 拍)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) copy_valid <= 0;
        else copy_valid <= copy_read_req;
    end

    // =========================================================================
    // 3. Bank RAM 阵列 (对应 KeyGen.v 中的架构)
    // =========================================================================
    genvar i;
    generate
        // S1 Bank (最大 8 个)
        for (i = 0; i < 8; i = i + 1) begin : gen_s1_bank
            wire we_a = (state == S_UNPACK) ? (unpack_we_s1 && unpack_row_idx == i) :
                        (state == S_COPY_OUT_RUN || state == S_COPY_OUT_WAIT) ? (copy_valid && p_cnt < l && p_cnt == i) : 1'b0;
            wire [7:0] addr_a = (state == S_UNPACK) ? unpack_col_idx : write_addr_reg;
            
            tdpram_24x256 u_s1_ram (
                .clk(clk), 
                .we_a(we_a), .addr_a(addr_a), .din_a(state == S_UNPACK ? unpack_post_mod_data : work_dout_b), .dout_a(),
                .we_b(1'b0), .addr_b(coeff_cnt[7:0]), .din_b(24'd0), .dout_b(s1_dout_b[i])
            );
        end

        // S2 Bank (最大 8 个)
        for (i = 0; i < 8; i = i + 1) begin : gen_s2_bank
            wire we_a = (state == S_UNPACK) ? (unpack_we_s2 && unpack_row_idx == i) :
                        (state == S_COPY_OUT_RUN || state == S_COPY_OUT_WAIT) ? (copy_valid && p_cnt >= l && p_cnt < l+k && (p_cnt - l) == i) : 1'b0;
            wire [7:0] addr_a = (state == S_UNPACK) ? unpack_col_idx : write_addr_reg;
            
            tdpram_24x256 u_s2_ram (
                .clk(clk), 
                .we_a(we_a), .addr_a(addr_a), .din_a(state == S_UNPACK ? unpack_post_mod_data : work_dout_b), .dout_a(),
                .we_b(1'b0), .addr_b(coeff_cnt[7:0]), .din_b(24'd0), .dout_b(s2_dout_b[i])
            );
        end

        // T0 Bank (最大 8 个)
        for (i = 0; i < 8; i = i + 1) begin : gen_t0_bank
            wire we_a = (state == S_UNPACK) ? (unpack_we_t0 && unpack_row_idx == i) :
                        (state == S_COPY_OUT_RUN || state == S_COPY_OUT_WAIT) ? (copy_valid && p_cnt >= l+k && (p_cnt - l - k) == i) : 1'b0;
            wire [7:0] addr_a = (state == S_UNPACK) ? unpack_col_idx : write_addr_reg;
            
            tdpram_24x256 u_t0_ram (
                .clk(clk), 
                .we_a(we_a), .addr_a(addr_a), .din_a(state == S_UNPACK ? unpack_post_mod_data : work_dout_b), .dout_a(),
                .we_b(1'b0), .addr_b(coeff_cnt[7:0]), .din_b(24'd0), .dout_b(t0_dout_b[i])
            );
        end
    endgenerate

    // [修复] 安全的数组越界保护索引 (保证位宽严格匹配 3-bit)
    wire [2:0] idx_s1 = p_cnt[2:0];
    wire [2:0] idx_s2 = p_cnt[2:0] - l[2:0];
    wire [2:0] idx_t0 = p_cnt[2:0] - l[2:0] - k[2:0];

    reg [23:0] current_bank_dout_b;
    always @(*) begin
        current_bank_dout_b = 24'd0;
        if (p_cnt < l) current_bank_dout_b = s1_dout_b[idx_s1];
        else if (p_cnt < l+k) current_bank_dout_b = s2_dout_b[idx_s2];
        else current_bank_dout_b = t0_dout_b[idx_t0];
    end

    // =========================================================================
    // 4. NTT 工作 RAM (对应 KeyGen 中的 u_ram) 及 NTT Core
    // =========================================================================
    wire ntt_active = (state == S_NTT_START || state == S_NTT_RUN);
    
    // Work RAM 端口 A：搬运写入 / NTT 运算 A 端口
    wire work_we_a = ntt_active ? ntt_we_a : ((state == S_COPY_IN_RUN || state == S_COPY_IN_WAIT) ? copy_valid : 1'b0);
    wire [7:0] work_addr_a = ntt_active ? ntt_addr_a : write_addr_reg;
    wire [23:0] work_din_a = ntt_active ? ntt_wdata_a : current_bank_dout_b;

    // Work RAM 端口 B：NTT 运算 B 端口 / 搬运读出 (Dump 导回给 Bank)
    wire work_we_b = ntt_active ? ntt_we_b : 1'b0;
    wire [7:0] work_addr_b = ntt_active ? ntt_addr_b : coeff_cnt[7:0];
    wire [23:0] work_din_b = ntt_active ? ntt_wdata_b : 24'd0;

    tdpram_24x256 u_work_ram (
        .clk(clk),
        .we_a(work_we_a), .addr_a(work_addr_a), .din_a(work_din_a), .dout_a(work_dout_a),
        .we_b(work_we_b), .addr_b(work_addr_b), .din_b(work_din_b), .dout_b(work_dout_b)
    );

    reg ntt_start; wire ntt_done;
    ntt_core #(.WIDTH(24)) u_ntt_core (
        .clk(clk), .rst_n(rst_n), .start(ntt_start), .done(ntt_done),
        .ram_addr_a(ntt_addr_a), .ram_we_a(ntt_we_a), .ram_wdata_a(ntt_wdata_a), .ram_rdata_a(work_dout_a),
        .ram_addr_b(ntt_addr_b), .ram_we_b(ntt_we_b), .ram_wdata_b(ntt_wdata_b), .ram_rdata_b(work_dout_b),
        // [修复] 空连以避免 Warning
        .dbg_gk(), .dbg_g1(), .dbg_val_u(), .dbg_val_v(), .dbg_prod_y(), .dbg_butterfly_done()
    );

    // =========================================================================
    // 5. 主状态机调度逻辑 (严格复刻 DUMP 管道流)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; o_done <= 0; unpack_start <= 0; ntt_start <= 0;
            p_cnt <= 0; coeff_cnt <= 0; write_addr_reg <= 0; copy_read_req <= 0;
        end else begin
            unpack_start <= 0; ntt_start <= 0; copy_read_req <= 0;

            case (state)
                S_IDLE: begin
                    o_done <= 0;
                    if (i_start) begin unpack_start <= 1; state <= S_UNPACK; end
                end

                S_UNPACK: begin
                    if (unpack_done) begin p_cnt <= 0; state <= S_COPY_IN_INIT; end
                end

                // --- 1. 将 Bank 里的数据搬运到 工作 RAM ---
                S_COPY_IN_INIT: begin
                    coeff_cnt <= 0; write_addr_reg <= 0; copy_read_req <= 1;
                    state <= S_COPY_IN_RUN;
                end
                S_COPY_IN_RUN: begin
                    if (coeff_cnt < 255) begin coeff_cnt <= coeff_cnt + 1; copy_read_req <= 1; end
                    else begin copy_read_req <= 0; state <= S_COPY_IN_WAIT; end
                    if (copy_valid) write_addr_reg <= write_addr_reg + 1;
                end
                S_COPY_IN_WAIT: begin
                    if (copy_valid) write_addr_reg <= write_addr_reg + 1;
                    else state <= S_NTT_START;
                end

                // --- 2. 启动并等待 NTT ---
                S_NTT_START: begin ntt_start <= 1; state <= S_NTT_RUN; end
                S_NTT_RUN: if (ntt_done) state <= S_COPY_OUT_INIT;

                // --- 3. 将工作 RAM 里做完 NTT 的数据导回 Bank ---
                S_COPY_OUT_INIT: begin
                    coeff_cnt <= 0; write_addr_reg <= 0; copy_read_req <= 1;
                    state <= S_COPY_OUT_RUN;
                end
                S_COPY_OUT_RUN: begin
                    if (coeff_cnt < 255) begin coeff_cnt <= coeff_cnt + 1; copy_read_req <= 1; end
                    else begin copy_read_req <= 0; state <= S_COPY_OUT_WAIT; end
                    if (copy_valid) write_addr_reg <= write_addr_reg + 1;
                end
                S_COPY_OUT_WAIT: begin
                    if (copy_valid) write_addr_reg <= write_addr_reg + 1;
                    else state <= S_NEXT_POLY;
                end

                // --- 4. 循环下一个多项式 ---
                S_NEXT_POLY: begin
                    if (p_cnt < ({1'b0, l} + {1'b0, k} + {1'b0, k} - 1)) begin
                        p_cnt <= p_cnt + 1;
                        state <= S_COPY_IN_INIT;
                    end else state <= S_DONE;
                end

                S_DONE: begin
                    o_done <= 1;
                    if (!i_start) state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule