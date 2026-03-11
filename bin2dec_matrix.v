`timescale 1ns / 1ps

module bin2dec_matrix #(
    parameter MAX_ARRAY_BITS = 8192,  
    parameter MAX_DATA_WIDTH = 24     
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        i_start,
    
    input  wire [4:0]                  i_width,          
    input  wire [15:0]                 i_num_elements,   
    input  wire [MAX_ARRAY_BITS-1:0]   i_bit_array,
    
    output reg                         o_we,
    output reg  [7:0]                  o_row_idx,        
    output reg  [7:0]                  o_col_idx,        
    output reg  [MAX_DATA_WIDTH-1:0]   o_data,           
    
    output reg                         o_done
);

    localparam S_IDLE   = 2'd0;
    localparam S_UNPACK = 2'd1;
    localparam S_PAD    = 2'd2;
    localparam S_DONE   = 2'd3;
    
    reg [1:0]  state;
    reg [15:0] count;          
    reg [31:0] read_ptr;       

    // 比特动态截取逻辑
    wire [MAX_ARRAY_BITS-1:0] shifted_array = i_bit_array >> read_ptr;
    wire [31:0]               mask          = (32'h1 << i_width) - 1'b1;
    wire [31:0]               current_val   = shifted_array[31:0] & mask;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            o_we      <= 1'b0;
            o_row_idx <= 8'd0;
            o_col_idx <= 8'd0;
            o_data    <= 0;
            o_done    <= 1'b0;
            count     <= 16'd0;
            read_ptr  <= 32'd0;
        end else begin
            o_we <= 1'b0; 
            
            case (state)
                S_IDLE: begin
                    o_done <= 1'b0;
                    if (i_start && i_num_elements > 0) begin
                        state     <= S_UNPACK;
                        count     <= 16'd0;
                        read_ptr  <= 32'd0;
                    end
                end
                
                S_UNPACK: begin
                    o_we      <= 1'b1;
                    o_data    <= current_val[MAX_DATA_WIDTH-1:0];
                    
                    // 妙招：利用 count 直接分配行列地址，完美解决延迟差 1 拍的问题
                    o_col_idx <= count[7:0];
                    o_row_idx <= count[15:8];
                    
                    read_ptr  <= read_ptr + i_width;
                    count     <= count + 1'b1;
                    
                    if (count == i_num_elements - 1'b1) begin
                        if (i_num_elements <= 16'd256) begin
                            state <= S_DONE;
                        end else if (count[7:0] == 8'd255) begin
                            // 刚好是 256 的倍数，不需要补零
                            state <= S_DONE;
                        end else begin
                            state <= S_PAD;
                        end
                    end
                end
                
                S_PAD: begin
                    o_we      <= 1'b1;
                    o_data    <= 0; // 强行写入 0 补齐矩阵
                    
                    // 继续利用 count 分配补零阶段的地址
                    o_col_idx <= count[7:0];
                    o_row_idx <= count[15:8];

                    count     <= count + 1'b1;

                    if (count[7:0] == 8'd255) begin
                        state <= S_DONE;
                    end
                end
                
                S_DONE: begin
                    o_done <= 1'b1;
                    if (!i_start) begin
                        state <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule