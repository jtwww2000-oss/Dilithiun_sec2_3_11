`timescale 1ns / 1ps

module dilithium_sign_core #(
    parameter WIDTH = 24,
    parameter Q     = 24'd8380417,
    parameter L_PARAM = 4'd4,
    parameter K_PARAM = 4'd4,
    parameter OMEGA   = 80
)(
    input  wire          clk,
    input  wire          rst_n,
    
    input  wire          i_start,  
    input  wire [255:0]  i_M,            
    
    output wire [9:0]    o_sk_raddr,
    input  wire [31:0]   i_sk_rdata,
    
    output reg           o_done,         
    output wire [15:0]   o_final_round,
    
    // ===============================================
    // ★ 外部测试台读取签名的 BRAM 接口
    // ===============================================
    input  wire [9:0]    i_sig_raddr,
    output reg  [31:0]   o_sig_rdata,
    output wire          o_sig_valid
);

    // ==========================================
    // 1. 内部 RAM
    // ==========================================
    reg [23:0] A_ram  [0:4095];
    reg [23:0] s1_ram [0:1023];
    reg [23:0] s2_ram [0:1023];
    reg [23:0] t0_ram [0:1023];

    reg  sk_start;
    wire sk_done;
    wire sk_s1_we, sk_s2_we, sk_t0_we, sk_A_we;
    wire [9:0]  sk_s1_addr, sk_s2_addr, sk_t0_addr;
    wire [23:0] sk_s1_wdata, sk_s2_wdata, sk_t0_wdata;
    wire [11:0] sk_A_addr;
    wire [23:0] sk_A_wdata;
    wire [511:0] sk_u, sk_rho_prime;

    sign_sk_unpack_ntt u_sk_unpack (
        .clk(clk), .rst_n(rst_n), .start(sk_start), .done(sk_done),
        .o_sk_raddr(o_sk_raddr), .i_sk_rdata(i_sk_rdata),
        .o_rho(), .o_K(), .o_tr(), 
        .o_s1_we(sk_s1_we), .o_s1_addr(sk_s1_addr), .o_s1_wdata(sk_s1_wdata),
        .o_s2_we(sk_s2_we), .o_s2_addr(sk_s2_addr), .o_s2_wdata(sk_s2_wdata),
        .o_t0_we(sk_t0_we), .o_t0_addr(sk_t0_addr), .o_t0_wdata(sk_t0_wdata),
        .i_M(i_M),
        .o_A_we(sk_A_we), .o_A_addr(sk_A_addr), .o_A_wdata(sk_A_wdata),
        .o_u(sk_u), .o_rho_prime(sk_rho_prime)
    );

    always @(posedge clk) begin
        if (sk_A_we)  A_ram[sk_A_addr]   <= sk_A_wdata;
        if (sk_s1_we) s1_ram[sk_s1_addr] <= sk_s1_wdata;
        if (sk_s2_we) s2_ram[sk_s2_addr] <= sk_s2_wdata;
        if (sk_t0_we) t0_ram[sk_t0_addr] <= sk_t0_wdata;
    end

    // ==========================================
    // 2. 签名核心流水线
    // ==========================================
    reg         sig_start;
    reg  [15:0] sig_rej_round;
    reg         sig_A_valid;
    reg  [23:0] sig_A_data;
    reg  [7:0]  sig_A_m_idx;
    reg  [3:0]  sig_A_j_idx;
    wire        sig_ready_for_A;
    wire        sig_w_valid;     
    wire [5:0]  sig_w1_data;
    wire [3:0]  sig_w_poly_idx;
    wire [7:0]  sig_w_coeff_idx;
    wire        sig_w_done;
    reg         sig_c_start;
    reg  [255:0] c_tilde_reg;
    
    wire [9:0]  sig_s1_rd_addr, sig_s2_rd_addr, sig_t0_rd_addr;
    reg [23:0] s1_rd_data_reg, s2_rd_data_reg, t0_rd_data_reg;

    always @(posedge clk) begin
        s1_rd_data_reg <= s1_ram[sig_s1_rd_addr];
        s2_rd_data_reg <= s2_ram[sig_s2_rd_addr];
        t0_rd_data_reg <= t0_ram[sig_t0_rd_addr];
    end

    wire        sig_rej_flag, sig_all_done;
    wire        sig_z_valid;     wire [23:0] sig_z_data;
    wire [3:0]  sig_z_poly_idx;  wire [7:0]  sig_z_coeff_idx;
    wire        sig_hint_pre_valid; wire       sig_hint_pre_data;
    wire [3:0]  sig_hint_pre_poly_idx; wire [7:0] sig_hint_pre_coeff_idx;

    sign_y_ntt_mac_intt #( .WIDTH(WIDTH), .Q(Q), .L_PARAM(L_PARAM), .K_PARAM(K_PARAM) ) u_sig_loop (
        .clk(clk), .rst_n(rst_n),
        .i_start(sig_start), .i_rho_prime(sk_rho_prime), .i_rej_round(sig_rej_round),
        .i_A_valid(sig_A_valid), .i_A_data(sig_A_data), .i_A_m_idx(sig_A_m_idx), .i_A_j_idx(sig_A_j_idx),
        .o_ready_for_A(sig_ready_for_A),
        
        .o_w_valid(sig_w_valid), .o_w_data(), .o_w1_data(sig_w1_data),
        .o_w_poly_idx(sig_w_poly_idx), .o_w_coeff_idx(sig_w_coeff_idx), .o_w_done(sig_w_done),
        
        .i_c_start(sig_c_start), .i_c_tilde(c_tilde_reg),
        .i_s1_rd_data(s1_rd_data_reg), .o_s1_rd_addr(sig_s1_rd_addr),
        .i_s2_rd_data(s2_rd_data_reg), .o_s2_rd_addr(sig_s2_rd_addr),
        .i_t0_rd_data(t0_rd_data_reg), .o_t0_rd_addr(sig_t0_rd_addr),
        
        .o_z_valid(sig_z_valid), .o_z_data(sig_z_data), .o_z_poly_idx(sig_z_poly_idx), .o_z_coeff_idx(sig_z_coeff_idx),
        .o_cs2_valid(), .o_cs2_data(), .o_cs2_poly_idx(), .o_cs2_coeff_idx(),
        .o_r0_valid(), .o_r0_data(), .o_r0_poly_idx(), .o_r0_coeff_idx(),
        .o_ct0_valid(), .o_ct0_data(), .o_ct0_poly_idx(), .o_ct0_coeff_idx(),
        
        .o_hint_pre_valid(sig_hint_pre_valid), .o_hint_pre_data(sig_hint_pre_data), 
        .o_hint_pre_poly_idx(sig_hint_pre_poly_idx), .o_hint_pre_coeff_idx(sig_hint_pre_coeff_idx),
        
        .o_rej_flag(sig_rej_flag), .o_all_done(sig_all_done)
    );
    assign o_final_round = sig_rej_round;

    // ==========================================
    // 3. SHAKE256 (c_tilde)
    // ==========================================
    reg [6143:0] w1_buf;
    always @(posedge clk) begin
        if (sig_w_valid) w1_buf[(sig_w_poly_idx * 256 + sig_w_coeff_idx)*6 +: 6] <= sig_w1_data;
    end

    reg          shake_c_start;
    wire         shake_c_valid;
    wire [255:0] shake_c_data_wire;
    SHAKE256 #( .OUTPUT_LEN_BYTES(32), .ABSORB_LEN(512 + 6144) ) u_shake_c (
        .clk(clk), .rst_n(rst_n), .i_start(shake_c_start),
        .i_seed({w1_buf, sk_u}),      
        .o_busy(), .i_squeeze_req(1'b0), 
        .o_squeeze_valid(shake_c_valid), .o_squeeze_data(shake_c_data_wire)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) c_tilde_reg <= 256'd0;
        else if (shake_c_valid) c_tilde_reg <= shake_c_data_wire;
    end

    // ========================================================
    // ★ 4. Z_RAM 缓冲与 Hint 缓冲
    // ========================================================
    // 4.1 Z 缓冲计算与写入 (将原本的巨型寄存器换成了 BRAM)
    wire [24:0] z_y1_sub = 25'd131072 - sig_z_data;
    wire [23:0] z_proc   = (24'd131072 >= sig_z_data) ? z_y1_sub[23:0] : (Q + 24'd131072 - sig_z_data);

    (* ram_style = "block" *) reg [17:0] z_ram [0:1023];
    always @(posedge clk) begin
        if (sig_z_valid) begin
            z_ram[{sig_z_poly_idx[1:0], sig_z_coeff_idx}] <= z_proc[17:0];
        end
    end

   // ========================================================
    // 4.2 Hint 缓冲计算与写入 (适配最新的单端口 Makehint)
    // ========================================================
    wire       hint_we;
    wire [7:0] hint_addr;
    wire [7:0] hint_wdata;

    Makehint #(
        .OMEGA(OMEGA), .K_PARAM(K_PARAM)
    ) u_makehint (
        .clk(clk), .rst_n(rst_n),
        .i_start(sig_c_start),
        .i_valid(sig_hint_pre_valid),
        .i_hint_pre(sig_hint_pre_data),
        .i_poly_idx(sig_hint_pre_poly_idx),
        .i_coeff_idx(sig_hint_pre_coeff_idx),
        
        // ★ 统一使用单端口输出，删除了之前的 hint_k 端口连线 ★
        .o_hint_we(hint_we), 
        .o_hint_addr(hint_addr), 
        .o_hint_wdata(hint_wdata),
        
        .o_fail(), .o_done()
    );

    reg [671:0] hint_buf;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hint_buf <= 0;
        end else begin
            // 每次重新采样开始时清空 buffer
            if (sig_c_start) begin
                hint_buf <= 0;
            end else begin
                // ★ 恢复最简单的单端口写入逻辑 ★
                if (hint_we) begin
                    hint_buf[hint_addr * 8 +: 8] <= hint_wdata;
                end
            end
        end
    end
    // ========================================================
    // ★ 5. 实例化 sig_packer 并挂载签名 BRAM
    // ========================================================
    reg         sig_pack_start;
    wire [3:0]  packer_z_poly;
    wire [7:0]  packer_z_coeff;
    wire [7:0]  packer_hint_addr;
    wire        sig_bram_we;
    wire [9:0]  sig_bram_waddr;
    wire [31:0] sig_bram_wdata;

    // 读取 z_ram 的一层寄存器缓冲
    reg [17:0] z_ram_rdata;
    always @(posedge clk) begin
        z_ram_rdata <= z_ram[{packer_z_poly[1:0], packer_z_coeff}];
    end

    sig_packer u_sig_packer (
        .clk(clk), .rst_n(rst_n), .i_start(sig_pack_start),
        .c_tilde(c_tilde_reg),
        .o_z_poly_idx(packer_z_poly), .o_z_coeff_idx(packer_z_coeff), .i_z_data(z_ram_rdata),
        .o_hint_addr(packer_hint_addr), .i_hint_data(hint_buf[packer_hint_addr * 8 +: 8]),
        .o_sig_we(sig_bram_we), .o_sig_addr(sig_bram_waddr), .o_sig_wdata(sig_bram_wdata), .o_sig_valid(o_sig_valid)
    );

    // 最终供外部读出的 32-bit Signature BRAM
    (* ram_style = "block" *) reg [31:0] signature_bram [0:1023];
    always @(posedge clk) begin
        if (sig_bram_we) signature_bram[sig_bram_waddr] <= sig_bram_wdata;
        o_sig_rdata <= signature_bram[i_sig_raddr];
    end

    // ==========================================
    // 6. 顶层自动重试 FSM
    // ==========================================
    localparam ST_IDLE       = 4'd0;
    localparam ST_UNPACK     = 4'd1;
    localparam ST_SIGN_INIT  = 4'd2;
    localparam ST_SIGN_PH1   = 4'd3;
    localparam ST_HASH_C     = 4'd4;
    localparam ST_SIGN_PH2   = 4'd5;
    localparam ST_PACK       = 4'd6; // 新增打包状态
    localparam ST_DONE       = 4'd7;

    reg [3:0] state;
    reg [3:0] stream_i, stream_j;
    reg [8:0] stream_m;
    reg       stream_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            o_done <= 0; sk_start <= 0;
            sig_start <= 0; sig_c_start <= 0; shake_c_start <= 0;
            sig_rej_round <= 0;
            stream_active <= 0; sig_A_valid <= 0;
            sig_pack_start <= 0;
        end else begin
            sk_start <= 0;
            sig_start <= 0; sig_c_start <= 0; shake_c_start <= 0;
            sig_pack_start <= 0;

            case (state)
                ST_IDLE: begin
                    o_done <= 0;
                    if (i_start) begin sk_start <= 1; state <= ST_UNPACK; end
                end
                
                ST_UNPACK: begin 
                    if (sk_done) begin sig_rej_round <= 0;
                        state <= ST_SIGN_INIT; end
                end
                
                ST_SIGN_INIT: begin 
                    sig_start <= 1;
                    stream_i <= 0; stream_j <= 0;
                    stream_active <= 0; state <= ST_SIGN_PH1;
                end
                
                ST_SIGN_PH1: begin 
                    if (sig_ready_for_A && !stream_active) begin
                        stream_active <= 1;
                        stream_m <= 0;
                    end
                    if (stream_active) begin
                        sig_A_valid <= 1;
                        sig_A_j_idx <= stream_j; sig_A_m_idx <= stream_m[7:0];
                        sig_A_data  <= A_ram[{stream_i[1:0], stream_j[1:0], stream_m[7:0]}];
                        if (stream_m == 255) begin
                            stream_active <= 0;
                            if (stream_j == L_PARAM - 1) begin
                                stream_j <= 0;
                                if (stream_i == K_PARAM - 1) stream_i <= 0; else stream_i <= stream_i + 1;
                            end else stream_j <= stream_j + 1;
                        end else stream_m <= stream_m + 1;
                    end else sig_A_valid <= 0;
                    
                    if (sig_w_done) begin shake_c_start <= 1; state <= ST_HASH_C; end
                end
                
                ST_HASH_C: begin 
                    if (shake_c_valid) begin sig_c_start <= 1;
                        state <= ST_SIGN_PH2; end
                end
                
                ST_SIGN_PH2: begin 
                    if (sig_all_done) begin
                        if (sig_rej_flag) begin
                            sig_rej_round <= sig_rej_round + L_PARAM;
                            state <= ST_SIGN_INIT;
                        end else begin
                            // ★ 成功通过所有采样检验！触发 Packer 打包 BRAM ★
                            sig_pack_start <= 1;
                            state <= ST_PACK;
                        end
                    end
                end
                
                ST_PACK: begin
                    // 等待 sig_packer 将结果全部压入 signature_bram 完成
                    if (o_sig_valid) begin
                        state <= ST_DONE;
                    end
                end
                
                ST_DONE: begin
                    o_done <= 1;
                    if (!i_start) state <= ST_IDLE; 
                end
            endcase
        end
    end
endmodule