// ============================================================
// ULTRA-TINY CNN INFERENCE ENGINE (PIPELINED V2)
// ============================================================
// Model: Nano-CNN Optimized for 100MHz+ ASIC/FPGA
// Changes: 3-Stage Pipeline, Modulo Removal, BRAM Inference
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

module cnn_inference (
    input wire clk,
    input wire rst_n,
    
    // Image input interface
    input wire [7:0] pixel_in,
    input wire pixel_valid,
    input wire frame_start,
    
    // Output interface
    output reg classification,
    output reg [7:0] confidence,
    output reg ready,
    output reg busy
);

  // ============================================================
  // PARAMETERS & WEIGHTS
  // ============================================================
  localparam IMG_SIZE = 1024;
  
  // Weights (Vivado will try to infer BRAM if possible)
  `include "cnn_weights.v"

  // ============================================================
  // PIPELINE REGISTERS
  // ============================================================
  reg [15:0] stage1_step;
  reg [6:0]  stage1_wptr;
  
  reg signed [15:0] stage2_data;   
  reg signed [7:0]  stage2_weight;
  reg stage2_valid;
  
  reg signed [23:0] stage3_prod;   
  reg stage3_valid;

  // ============================================================
  // FSM & CONTROL
  // ============================================================
  localparam IDLE       = 0;
  localparam CONV1      = 1; 
  localparam CONV1_DRAIN = 2;
  localparam DENSE      = 3;
  localparam DENSE_DRAIN = 4;
  localparam DONE       = 5;

  reg [2:0] state;
  reg [10:0] pix_cnt;
  reg signed [23:0] acc;
  reg inference_done;

  // Memory Resources
  (* ram_style = "block" *) reg [7:0] input_buffer [0:1023];
  (* ram_style = "block" *) reg signed [15:0] feature_buffer [0:127]; 

  // Initialize Memory
  integer l;
  initial begin
      for(l=0; l<1024; l=l+1) input_buffer[l] = 0;
      for(l=0; l<128; l=l+1) feature_buffer[l] = 0;
  end 

  // ============================================================
  // STORAGE LOGIC
  // ============================================================
  always @(posedge clk) begin
    if (pixel_valid && pix_cnt < IMG_SIZE) begin
      input_buffer[pix_cnt] <= pixel_in;
    end
  end

  // ============================================================
  // MAIN ENGINE FSM (PIPELINED)
  // ============================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= IDLE;
      busy <= 0;
      ready <= 0;
      acc <= 0;
      classification <= 0;
      confidence <= 0;
      pix_cnt <= 0;
      inference_done <= 0;
      stage1_step <= 0;
      stage1_wptr <= 0;
      stage2_valid <= 0;
      stage3_valid <= 0;
    end else begin
      if (frame_start) begin
        pix_cnt <= 0;
        inference_done <= 0;
      end else if (pixel_valid && pix_cnt < IMG_SIZE) begin
        pix_cnt <= pix_cnt + 1;
      end

      case (state)
        IDLE: begin
          ready <= 0;
          stage2_valid <= 0;
          stage3_valid <= 0;
          if (pix_cnt == IMG_SIZE && !inference_done) begin
            state <= CONV1;
            busy <= 1;
            stage1_step <= 0;
            stage1_wptr <= 0;
            // Initialize with signs-extended Conv Bias (add once, not per pixel)
            acc <= {{16{conv2d_b[0][7]}}, conv2d_b[0]};
          end
        end

        // CONV1 Stage: Cycles every 72 weights
        CONV1: begin
          if (stage1_step < IMG_SIZE) begin
            stage1_step <= stage1_step + 1;
            if (stage1_wptr == 71) stage1_wptr <= 0;
            else stage1_wptr <= stage1_wptr + 1;
            stage2_valid <= 1;
          end else begin
            state <= CONV1_DRAIN;
            stage2_valid <= 0;
          end
          
          // Correctly center pixels: (pixel - 128)
          stage2_data <= $signed({1'b0, input_buffer[stage1_step[9:0]]}) - 16'd128;
          stage2_weight <= conv2d_w[stage1_wptr];
          
          stage3_valid <= stage2_valid;
          stage3_prod <= stage2_data * stage2_weight;
          if (stage3_valid) acc <= acc + stage3_prod;
        end

        CONV1_DRAIN: begin
          stage3_valid <= stage2_valid;
          stage3_prod <= stage2_data * stage2_weight;
          if (stage3_valid) acc <= acc + stage3_prod;
          
          if (!stage3_valid && !stage2_valid) begin
            state <= DENSE;
            stage1_step <= 0;
            feature_buffer[0] <= acc[23:8];
            // Initialize with Dense Bias
            acc <= {{16{dense_b[0][7]}}, dense_b[0]};
          end
        end

        DENSE: begin
          if (stage1_step < 128) begin
            stage1_step <= stage1_step + 1;
            stage2_valid <= 1;
          end else begin
            state <= DENSE_DRAIN;
            stage2_valid <= 0;
          end
          
          stage2_data <= feature_buffer[stage1_step[6:0]];
          stage2_weight <= dense_w[stage1_step[6:0]];
          
          stage3_valid <= stage2_valid;
          stage3_prod <= stage2_data * stage2_weight;
          if (stage3_valid) acc <= acc + stage3_prod;
        end

        DENSE_DRAIN: begin
          stage3_valid <= stage2_valid;
          stage3_prod <= stage2_data * stage2_weight;
          if (stage3_valid) acc <= acc + stage3_prod;
          
          if (!stage3_valid && !stage2_valid) state <= DONE;
        end

        DONE: begin
          $display("[%0t] CNN FINAL: acc=%d, conf=%d", $time, acc, (acc[23]) ? 8'd0 : (acc[22:17] > 0 ? 8'd255 : acc[16:9]));
          // Rescale confidence: Use acc[16:9] to get good range for ~100k peak
          classification <= (acc > 0);
          confidence <= (acc[23]) ? 8'd0 : (acc[22:17] > 0 ? 8'd255 : acc[16:9]); 
          ready <= 1;
          busy <= 0;
          inference_done <= 1;
          state <= IDLE;
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule



