// ============================================================
// SYNTHESIZABLE TINY CNN - Fits in 2x2 TinyTapeout Tiles
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

module cnn_inference (
    input wire clk,
    input wire rst_n,
    
    input wire [7:0] pixel_in,
    input wire pixel_valid,
    input wire frame_start,
    
    output reg classification,
    output reg [7:0] confidence,
    output reg ready,
    output reg busy
);

localparam IMG_SIZE = 64;

// Minimal storage: 3 rows of 8 pixels = 192 flip-flops
reg [7:0] row_buf0 [0:7];
reg [7:0] row_buf1 [0:7];  
reg [7:0] row_buf2 [0:7];

reg signed [19:0] acc0, acc1;
reg [6:0] pixel_count;
reg [2:0] row_idx, col_idx, state;

localparam IDLE = 3'd0, LOADING = 3'd1, COMPUTE = 3'd2, DECIDE = 3'd3;

// Hardcoded weights (NO arrays, just wires!)
wire signed [7:0] w0_00 = 8'sd20, w0_01 = 8'sd30, w0_02 = -8'sd10;
wire signed [7:0] w0_10 = 8'sd15, w0_11 = -8'sd25, w0_12 = 8'sd20;
wire signed [7:0] w0_20 = -8'sd15, w0_21 = 8'sd35, w0_22 = -8'sd20;

wire signed [7:0] w1_00 = 8'sd10, w1_01 = -8'sd20, w1_02 = 8'sd25;
wire signed [7:0] w1_10 = 8'sd20, w1_11 = -8'sd30, w1_12 = -8'sd10;
wire signed [7:0] w1_20 = 8'sd30, w1_21 = 8'sd15, w1_22 = -8'sd25;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        pixel_count <= 0;
        row_idx <= 0;
        col_idx <= 0;
        ready <= 0;
        busy <= 0;
        acc0 <= 0;
        acc1 <= 0;
        classification <= 0;
        confidence <= 0;
    end else begin
        case (state)
            IDLE: begin
                ready <= 0;
                busy <= 0;
                if (frame_start) begin
                    state <= LOADING;
                    pixel_count <= 0;
                    row_idx <= 0;
                    col_idx <= 0;
                    acc0 <= 0;
                    acc1 <= 0;
                    busy <= 1;
                end
            end
            
            LOADING: begin
                if (pixel_valid) begin
                    case (row_idx)
                        3'd0: row_buf0[col_idx] <= pixel_in;
                        3'd1: row_buf1[col_idx] <= pixel_in;
                        3'd2: row_buf2[col_idx] <= pixel_in;
                        3'd3: row_buf0[col_idx] <= pixel_in;
                        3'd4: row_buf1[col_idx] <= pixel_in;
                        3'd5: row_buf2[col_idx] <= pixel_in;
                        3'd6: row_buf0[col_idx] <= pixel_in;
                        3'd7: row_buf1[col_idx] <= pixel_in;
                    endcase
                    
                    if (col_idx == 7) begin
                        col_idx <= 0;
                        row_idx <= row_idx + 1;
                    end else begin
                        col_idx <= col_idx + 1;
                    end
                    
                    pixel_count <= pixel_count + 1;
                    
                    if (pixel_count == IMG_SIZE - 1) begin
                        state <= COMPUTE;
                        pixel_count <= 0;
                    end
                end
            end
            
            COMPUTE: begin
                if (pixel_count < IMG_SIZE) begin
                    case (pixel_count)
                        7'd0: begin
                            acc0 <= acc0 + (row_buf0[0] * w0_00);
                            acc1 <= acc1 + (row_buf0[0] * w1_00);
                        end
                        7'd1: begin
                            acc0 <= acc0 + (row_buf0[1] * w0_01);
                            acc1 <= acc1 + (row_buf0[1] * w1_01);
                        end
                        7'd2: begin
                            acc0 <= acc0 + (row_buf0[2] * w0_02);
                            acc1 <= acc1 + (row_buf0[2] * w1_02);
                        end
                        default: begin
                            acc0 <= acc0 + (row_buf0[pixel_count[2:0]] * w0_00);
                            acc1 <= acc1 + (row_buf0[pixel_count[2:0]] * w1_00);
                        end
                    endcase
                    pixel_count <= pixel_count + 1;
                end else begin
                    state <= DECIDE;
                end
            end
            
            DECIDE: begin
                if (acc0 > acc1) begin
                    classification <= 1;
                    confidence <= 8'd85;
                end else begin
                    classification <= 0;
                    confidence <= 8'd80;
                end
                ready <= 1;
                busy <= 0;
                state <= IDLE;
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule

`default_nettype wire
