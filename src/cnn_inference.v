// ============================================================
// ULTRA-TINY CNN - Fits in 2x2 TinyTapeout Tiles (~10K cells)
// ============================================================
// Absolute minimal CNN for proof-of-concept
// Input: 8x8 grayscale (4-bit per pixel to save space)
// Architecture: 1 conv layer, global pool, 1 dense layer
// Total memory: ~500 bytes = ~4000 cells
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

module cnn_inference (
    input wire clk,
    input wire rst_n,
    
    // Streaming pixel input (4-bit to save space)
    input wire [3:0] pixel_in,
    input wire pixel_valid,
    input wire frame_start,
    
    // Output
    output reg classification,  // 0=growth, 1=harvest
    output reg [7:0] confidence,
    output reg ready
);

// ============================================================
// PARAMETERS - Absolute minimum for 2x2 tiles
// ============================================================
localparam IMG_SIZE = 64;      // 8x8 image
localparam NUM_FILTERS = 2;    // Just 2 filters
localparam KERNEL_SIZE = 9;    // 3x3 kernel

// ============================================================
// WEIGHTS - Hardcoded as wires (no memory arrays!)
// ============================================================
// Filter 0: 3x3 weights (4-bit signed)
wire signed [3:0] f0_w[0:8];
assign f0_w[0] = 4'sd2;
assign f0_w[1] = 4'sd3;
assign f0_w[2] = -4'sd1;
assign f0_w[3] = 4'sd1;
assign f0_w[4] = -4'sd3;
assign f0_w[5] = 4'sd2;
assign f0_w[6] = -4'sd2;
assign f0_w[7] = 4'sd4;
assign f0_w[8] = -4'sd3;

// Filter 1: 3x3 weights
wire signed [3:0] f1_w[0:8];
assign f1_w[0] = 4'sd1;
assign f1_w[1] = -4'sd2;
assign f1_w[2] = 4'sd3;
assign f1_w[3] = 4'sd2;
assign f1_w[4] = -4'sd4;
assign f1_w[5] = -4'sd1;
assign f1_w[6] = 4'sd4;
assign f1_w[7] = 4'sd1;
assign f1_w[8] = -4'sd3;

// Dense layer weights (2 inputs -> 1 output)
wire signed [7:0] dense_w[0:1];
assign dense_w[0] = 8'sd45;   // Weight for filter 0
assign dense_w[1] = -8'sd38;  // Weight for filter 1

// ============================================================
// MINIMAL STORAGE - Only what's absolutely necessary
// ============================================================
// Input: Only store 3 rows at a time for convolution (sliding window)
reg [3:0] row_buffer[0:23];  // 3 rows × 8 cols = 24 pixels (96 bits)

// Feature maps: Just store the accumulated sums
reg signed [15:0] feature_sum[0:1];  // 2 filters

// ============================================================
// CONTROL SIGNALS
// ============================================================
reg [6:0] pixel_count;
reg [2:0] row_idx;
reg [2:0] col_idx;

// State machine
reg [1:0] state;
localparam IDLE = 0, LOADING = 1, CONV = 2, DENSE = 3;

// Convolution counter
reg [5:0] conv_position;  // Position in 6x6 valid conv area

// ============================================================
// PIXEL LOADING with Sliding Window
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pixel_count <= 0;
        row_idx <= 0;
        col_idx <= 0;
        state <= IDLE;
        ready <= 0;
        for (integer i = 0; i < 24; i = i + 1)
            row_buffer[i] <= 0;
    end else begin
        case (state)
            IDLE: begin
                ready <= 0;
                if (frame_start) begin
                    pixel_count <= 0;
                    row_idx <= 0;
                    col_idx <= 0;
                    state <= LOADING;
                    feature_sum[0] <= 0;
                    feature_sum[1] <= 0;
                    conv_position <= 0;
                end
            end
            
            LOADING: begin
                if (pixel_valid) begin
                    // Store in sliding window buffer
                    // We only keep 3 rows in memory
                    if (row_idx < 3) begin
                        row_buffer[row_idx * 8 + col_idx] <= pixel_in;
                    end else begin
                        // Shift rows: row0 <- row1, row1 <- row2, row2 <- new
                        // This is expensive, so we use indexing trick instead
                        row_buffer[((row_idx % 3) * 8) + col_idx] <= pixel_in;
                    end
                    
                    // Process convolution when we have 3x3 window ready
                    if (row_idx >= 2 && col_idx >= 2) begin
                        // We can compute one convolution output here
                        // This is done in parallel with loading
                        state <= CONV;
                    end
                    
                    // Update position
                    if (col_idx == 7) begin
                        col_idx <= 0;
                        row_idx <= row_idx + 1;
                    end else begin
                        col_idx <= col_idx + 1;
                    end
                    
                    pixel_count <= pixel_count + 1;
                    
                    if (pixel_count == IMG_SIZE - 1) begin
                        state <= DENSE;  // Move to final decision
                    end
                end
            end
            
            CONV: begin
                // Perform 3x3 convolution at current position
                // This is simplified - in reality you'd pipeline this
                
                // Extract 3x3 window (simplified indexing)
                reg signed [15:0] sum0, sum1;
                integer r, c, idx;
                
                sum0 = 0;
                sum1 = 0;
                
                for (r = 0; r < 3; r = r + 1) begin
                    for (c = 0; c < 3; c = c + 1) begin
                        idx = ((row_idx - 2 + r) % 3) * 8 + (col_idx - 2 + c);
                        sum0 = sum0 + ($signed({1'b0, row_buffer[idx]}) * f0_w[r*3 + c]);
                        sum1 = sum1 + ($signed({1'b0, row_buffer[idx]}) * f1_w[r*3 + c]);
                    end
                end
                
                // ReLU and accumulate (Global Average Pooling)
                if (sum0 > 0) feature_sum[0] <= feature_sum[0] + sum0;
                if (sum1 > 0) feature_sum[1] <= feature_sum[1] + sum1;
                
                state <= LOADING;  // Go back to loading
            end
            
            DENSE: begin
                // Final classification layer
                reg signed [23:0] output_logit;
                
                // Dense layer: 2 inputs -> 1 output
                output_logit = (feature_sum[0] * dense_w[0]) + 
                               (feature_sum[1] * dense_w[1]);
                
                // Sigmoid approximation: if positive -> harvest (1), else growth (0)
                classification <= (output_logit > 0);
                
                // Confidence based on magnitude
                if (output_logit > 1000 || output_logit < -1000)
                    confidence <= 8'd95;
                else if (output_logit > 500 || output_logit < -500)
                    confidence <= 8'd80;
                else
                    confidence <= 8'd60;
                
                ready <= 1;
                state <= IDLE;
            end
        endcase
    end
end

endmodule

`default_nettype wire
