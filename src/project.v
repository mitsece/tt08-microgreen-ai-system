// MONOLITHIC PROJECT FILE - CLEANED FOR ASIC SYNTHESIS
`default_nettype none

module tt_um_precision_farming (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
  wire [6:0] status_leds;
  wire [2:0] alert_level;
  wire harvest_alert;
  wire uart_tx;
  wire fault_detected;

  main_processor_asic main_proc (
    .clk(clk), .rst_n(rst_n & ena), .cam_pclk(clk), .cam_href(uio_in[5]),
    .cam_vsync(uio_in[6]), .cam_data(ui_in), .uart_rx(uio_in[0]), .uart_tx(uart_tx),
    .harvest_alert(harvest_alert), .alert_level(alert_level), .status_leds(status_leds[6:0]), .fault_detected(fault_detected)
  );
  assign uo_out[6:0] = status_leds[6:0];
  assign uo_out[7]   = harvest_alert;
  assign uio_out[0] = uart_tx;
  assign uio_out[7:1] = 7'b0;
  assign uio_oe = 8'b00000001; 
  wire _unused = &{alert_level, fault_detected, uio_in[4:1], uio_in[7], ena, 1'b0};
endmodule

// --- From src/main_processor_top.v ---
// ============================================================
// MAIN PROCESSOR ASIC - TOP LEVEL
// ============================================================
// Competition Project: Dual-ASIC Precision Farming System
// Main Processor: CNN-based plant growth classification
// Co-Processor: Sensor monitoring and fault detection
// ============================================================




module main_processor_asic (
    input wire clk,              // System clock (50MHz)
    input wire rst_n,            // Active-low reset
    
    // Camera interface (OV7670)
    input wire cam_pclk,         // Pixel clock from camera
    input wire cam_href,         // Horizontal reference
    input wire cam_vsync,        // Vertical sync
    input wire [7:0] cam_data,   // 8-bit pixel data
    
    // UART interface (from co-processor)
    input wire uart_rx,          // UART receive from co-processor
    output wire uart_tx,         // UART transmit to co-processor
    
    // Output interface
    output wire harvest_alert,   // Harvest ready signal
    output wire [2:0] alert_level, // Alert severity (0-7)
    output wire [6:0] status_leds, // Status indicators
    output wire fault_detected   // System fault flag
);

// ============================================================
// INTERNAL SIGNALS
// ============================================================

// CNN signals
wire [7:0] cnn_pixel;
wire cnn_pixel_valid;
wire cnn_frame_start;
wire cnn_classification;
wire [7:0] cnn_confidence;
wire cnn_ready;
wire cnn_busy;

// UART signals
wire [7:0] uart_rx_data;
wire uart_rx_valid;
wire [7:0] uart_tx_data;
wire uart_tx_valid;

// Sensor data from co-processor
reg [1:0] sensor_temp;
reg [1:0] sensor_humidity;
reg [1:0] sensor_light;
reg [1:0] sensor_soil;
reg [7:0] fault_flags;

// Decision signals
reg harvest_ready_cnn;
reg harvest_ready_sensors;
reg harvest_ready_final;

// ============================================================
// CAMERA INTERFACE
// ============================================================

  wire _unused_frame_done;
  camera_interface cam_if (
    .clk(clk),
    .rst_n(rst_n),
    .cam_pclk(cam_pclk),
    .cam_href(cam_href),
    .cam_vsync(cam_vsync),
    .cam_data(cam_data),
    .pixel_out(cnn_pixel),
    .pixel_valid(cnn_pixel_valid),
    .frame_start(cnn_frame_start),
    .frame_done(_unused_frame_done)
);

// ============================================================
// CNN INFERENCE ENGINE
// ============================================================

cnn_inference cnn (
    .clk(clk),
    .rst_n(rst_n),
    .pixel_in(cnn_pixel),
    .pixel_valid(cnn_pixel_valid),
    .frame_start(cnn_frame_start),
    .classification(cnn_classification),
    .confidence(cnn_confidence),
    .ready(cnn_ready),
    .busy(cnn_busy)
);

// ============================================================
// UART RECEIVER (from co-processor)
// ============================================================

uart_rx #(
    .CLKS_PER_BIT(868)  // 100MHz / 115200 = 868
) uart_rx_inst (
    .clk(clk),
    .rst_n(rst_n),
    .rx(uart_rx),
    .rx_data(uart_rx_data),
    .rx_valid(uart_rx_valid)
);

// ============================================================
// UART TRANSMITTER (to co-processor)
// ============================================ ================

uart_tx #(
    .CLKS_PER_BIT(868)
) uart_tx_inst (
    .clk(clk),
    .rst_n(rst_n),
    .tx_data(uart_tx_data),
    .tx_valid(uart_tx_valid),
    .tx(uart_tx),
    .tx_ready(_unused_tx_ready)
);

wire _unused_tx_ready;

// ============================================================
// UART PACKET PARSER
// ============================================================

reg [2:0] rx_byte_count;
reg [7:0] rx_checksum;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_byte_count <= 0;
        sensor_temp <= 0;
        sensor_humidity <= 0;
        sensor_light <= 0;
        sensor_soil <= 0;
        fault_flags <= 0;
        // actuator_status <= 0;
    end else if (uart_rx_valid) begin
        case (rx_byte_count)
            3'd0: begin  // Header
                if (uart_rx_data == 8'hAA) begin
                    rx_byte_count <= rx_byte_count + 1;
                    rx_checksum <= 0;
                end
            end
            3'd1: begin  // Temperature
                sensor_temp <= uart_rx_data[1:0];
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd2: begin  // Humidity
                sensor_humidity <= uart_rx_data[1:0];
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd3: begin  // Light
                sensor_light <= uart_rx_data[1:0];
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd4: begin  // Soil moisture
                sensor_soil <= uart_rx_data[1:0];
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd5: begin  // Fault flags
                fault_flags <= uart_rx_data;
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd6: begin  // Actuator status
                // actuator_status <= uart_rx_data; // Unused
                rx_checksum <= rx_checksum + uart_rx_data;
                rx_byte_count <= rx_byte_count + 1;
            end
            3'd7: begin  // Checksum
                // Verify checksum
                rx_byte_count <= 0;
            end
        endcase
    end
end

// ============================================================
// DECISION FUSION LOGIC
// ============================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        harvest_ready_cnn <= 0;
        harvest_ready_sensors <= 0;
        harvest_ready_final <= 0;
    end else begin
        // CNN decision
        if (cnn_ready) begin
            harvest_ready_cnn <= cnn_classification && (cnn_confidence > 8'd40);
        end
        
        // Sensor-based decision
        // Optimal conditions: temp=2, humidity=2, light=2, soil=2
        harvest_ready_sensors <= (sensor_temp == 2'd2) && 
                                 (sensor_humidity == 2'd2) &&
                                 (sensor_light == 2'd2) &&
                                 (sensor_soil == 2'd2);
        
        // Final decision: Both CNN and sensors agree
        harvest_ready_final <= harvest_ready_cnn && harvest_ready_sensors;
    end
end

// ============================================================
// ALERT LEVEL GENERATION
// ============================================================

reg [2:0] alert_level_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        alert_level_reg <= 0;
    end else begin
        if (fault_flags != 0) begin
            alert_level_reg <= 3'd7;  // Critical fault
        end else if (harvest_ready_final) begin
            alert_level_reg <= 3'd6;  // Harvest ready
        end else if (harvest_ready_cnn) begin
            alert_level_reg <= 3'd4;  // CNN suggests harvest
        end else if (!harvest_ready_sensors) begin
            alert_level_reg <= 3'd2;  // Suboptimal conditions
        end else begin
            alert_level_reg <= 3'd0;  // Normal
        end
    end
end

// ============================================================
// STATUS LED GENERATION
// ============================================================

(* IOB = "true" *) reg [6:0] status_leds_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        status_leds_reg <= 0;
    end else begin
        status_leds_reg[0] <= cnn_busy;              // CNN processing
        status_leds_reg[1] <= cnn_ready;             // CNN result ready
        status_leds_reg[2] <= harvest_ready_cnn;     // CNN: harvest
        status_leds_reg[3] <= harvest_ready_sensors; // Sensors: optimal
        status_leds_reg[4] <= harvest_ready_final;   // Final: harvest!
        status_leds_reg[5] <= |fault_flags;          // Any fault
        status_leds_reg[6] <= uart_rx_valid;         // UART activity
    end
end

// ============================================================
// OUTPUT ASSIGNMENTS
// ============================================================

assign harvest_alert = harvest_ready_final;
assign alert_level = alert_level_reg;
assign status_leds = status_leds_reg;
assign fault_detected = |fault_flags;

// ============================================================
// UART TX (Send commands to co-processor)
// ============================================================

// For now, just send acknowledgment
assign uart_tx_data = 8'h55;  // ACK
assign uart_tx_valid = uart_rx_valid;  // Echo back

endmodule

`default_nettype wire


// --- From src/cnn_inference.v ---
// ============================================================
// ULTRA-TINY CNN INFERENCE ENGINE (PIPELINED V2)
// ============================================================
// Model: Nano-CNN Optimized for 100MHz+ ASIC/FPGA
// Changes: 3-Stage Pipeline, Modulo Removal, BRAM Inference
// ============================================================




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
  // Ultra-Tiny CNN Weights for 2x2 ASIC
// Generated automatically
// Model: ultra_tiny_cnn
// 8-bit fixed point

// Layer: conv2d
(* ram_style = "block" *) reg signed [7:0] conv2d_w [0:71];
initial begin
    conv2d_w[0] = 8'd14;
    conv2d_w[1] = 8'd53;
    conv2d_w[2] = -8'd27;
    conv2d_w[3] = 8'd16;
    conv2d_w[4] = -8'd78;
    conv2d_w[5] = 8'd70;
    conv2d_w[6] = -8'd41;
    conv2d_w[7] = 8'd127;
    conv2d_w[8] = -8'd84;
    conv2d_w[9] = 8'd22;
    conv2d_w[10] = -8'd47;
    conv2d_w[11] = 8'd68;
    conv2d_w[12] = 8'd27;
    conv2d_w[13] = -8'd102;
    conv2d_w[14] = -8'd29;
    conv2d_w[15] = 8'd111;
    conv2d_w[16] = 8'd14;
    conv2d_w[17] = -8'd93;
    conv2d_w[18] = 8'd92;
    conv2d_w[19] = 8'd29;
    conv2d_w[20] = -8'd40;
    conv2d_w[21] = -8'd10;
    conv2d_w[22] = -8'd42;
    conv2d_w[23] = -8'd47;
    conv2d_w[24] = 8'd48;
    conv2d_w[25] = -8'd5;
    conv2d_w[26] = -8'd72;
    conv2d_w[27] = -8'd49;
    conv2d_w[28] = 8'd80;
    conv2d_w[29] = -8'd15;
    conv2d_w[30] = -8'd37;
    conv2d_w[31] = 8'd59;
    conv2d_w[32] = -8'd29;
    conv2d_w[33] = 8'd66;
    conv2d_w[34] = -8'd61;
    conv2d_w[35] = 8'd78;
    conv2d_w[36] = -8'd54;
    conv2d_w[37] = 8'd20;
    conv2d_w[38] = 8'd18;
    conv2d_w[39] = -8'd87;
    conv2d_w[40] = -8'd28;
    conv2d_w[41] = 8'd59;
    conv2d_w[42] = 8'd39;
    conv2d_w[43] = -8'd51;
    conv2d_w[44] = 8'd41;
    conv2d_w[45] = -8'd70;
    conv2d_w[46] = 8'd111;
    conv2d_w[47] = -8'd2;
    conv2d_w[48] = -8'd51;
    conv2d_w[49] = 8'd2;
    conv2d_w[50] = 8'd38;
    conv2d_w[51] = -8'd112;
    conv2d_w[52] = 8'd70;
    conv2d_w[53] = 8'd90;
    conv2d_w[54] = 8'd83;
    conv2d_w[55] = -8'd66;
    conv2d_w[56] = 8'd14;
    conv2d_w[57] = 8'd94;
    conv2d_w[58] = 8'd36;
    conv2d_w[59] = -8'd94;
    conv2d_w[60] = -8'd43;
    conv2d_w[61] = -8'd28;
    conv2d_w[62] = -8'd45;
    conv2d_w[63] = 8'd93;
    conv2d_w[64] = -8'd22;
    conv2d_w[65] = -8'd4;
    conv2d_w[66] = 8'd98;
    conv2d_w[67] = 8'd75;
    conv2d_w[68] = 8'd37;
    conv2d_w[69] = 8'd50;
    conv2d_w[70] = 8'd107;
    conv2d_w[71] = -8'd75;
end

reg signed [7:0] conv2d_b [0:7];
initial begin
    conv2d_b[0] = -8'd81;
    conv2d_b[1] = -8'd39;
    conv2d_b[2] = -8'd47;
    conv2d_b[3] = -8'd127;
    conv2d_b[4] = -8'd25;
    conv2d_b[5] = -8'd111;
    conv2d_b[6] = -8'd54;
    conv2d_b[7] = -8'd81;
end
// Layer: dense
(* ram_style = "block" *) reg signed [7:0] dense_w [0:127];
initial begin
    dense_w[0] = 8'd26;
    dense_w[1] = 8'd61;
    dense_w[2] = -8'd35;
    dense_w[3] = -8'd76;
    dense_w[4] = -8'd34;
    dense_w[5] = -8'd83;
    dense_w[6] = 8'd26;
    dense_w[7] = -8'd21;
    dense_w[8] = -8'd63;
    dense_w[9] = 8'd112;
    dense_w[10] = 8'd25;
    dense_w[11] = 8'd28;
    dense_w[12] = 8'd117;
    dense_w[13] = 8'd58;
    dense_w[14] = -8'd17;
    dense_w[15] = -8'd82;
    dense_w[16] = -8'd42;
    dense_w[17] = 8'd118;
    dense_w[18] = -8'd13;
    dense_w[19] = -8'd4;
    dense_w[20] = 8'd25;
    dense_w[21] = 8'd7;
    dense_w[22] = 8'd62;
    dense_w[23] = -8'd55;
    dense_w[24] = 8'd95;
    dense_w[25] = 8'd39;
    dense_w[26] = 8'd111;
    dense_w[27] = -8'd23;
    dense_w[28] = -8'd100;
    dense_w[29] = -8'd99;
    dense_w[30] = -8'd94;
    dense_w[31] = -8'd53;
    dense_w[32] = 8'd101;
    dense_w[33] = -8'd60;
    dense_w[34] = 8'd108;
    dense_w[35] = 8'd92;
    dense_w[36] = -8'd110;
    dense_w[37] = 8'd28;
    dense_w[38] = 8'd47;
    dense_w[39] = 8'd6;
    dense_w[40] = 8'd127;
    dense_w[41] = 8'd10;
    dense_w[42] = -8'd16;
    dense_w[43] = 8'd106;
    dense_w[44] = 8'd30;
    dense_w[45] = -8'd55;
    dense_w[46] = 8'd23;
    dense_w[47] = 8'd79;
    dense_w[48] = 8'd67;
    dense_w[49] = -8'd15;
    dense_w[50] = 8'd56;
    dense_w[51] = 8'd38;
    dense_w[52] = -8'd20;
    dense_w[53] = -8'd2;
    dense_w[54] = -8'd16;
    dense_w[55] = 8'd75;
    dense_w[56] = -8'd4;
    dense_w[57] = 8'd17;
    dense_w[58] = 8'd110;
    dense_w[59] = 8'd99;
    dense_w[60] = -8'd54;
    dense_w[61] = -8'd67;
    dense_w[62] = -8'd96;
    dense_w[63] = 8'd82;
    dense_w[64] = -8'd57;
    dense_w[65] = 8'd54;
    dense_w[66] = 8'd87;
    dense_w[67] = 8'd35;
    dense_w[68] = 8'd39;
    dense_w[69] = 8'd0;
    dense_w[70] = 8'd16;
    dense_w[71] = 8'd62;
    dense_w[72] = -8'd12;
    dense_w[73] = 8'd98;
    dense_w[74] = -8'd85;
    dense_w[75] = 8'd34;
    dense_w[76] = 8'd59;
    dense_w[77] = 8'd87;
    dense_w[78] = 8'd123;
    dense_w[79] = -8'd7;
    dense_w[80] = 8'd46;
    dense_w[81] = -8'd66;
    dense_w[82] = 8'd113;
    dense_w[83] = 8'd113;
    dense_w[84] = 8'd54;
    dense_w[85] = 8'd63;
    dense_w[86] = 8'd4;
    dense_w[87] = 8'd103;
    dense_w[88] = 8'd86;
    dense_w[89] = -8'd10;
    dense_w[90] = 8'd10;
    dense_w[91] = 8'd66;
    dense_w[92] = -8'd66;
    dense_w[93] = -8'd73;
    dense_w[94] = -8'd111;
    dense_w[95] = -8'd50;
    dense_w[96] = 8'd94;
    dense_w[97] = -8'd34;
    dense_w[98] = 8'd100;
    dense_w[99] = 8'd12;
    dense_w[100] = 8'd39;
    dense_w[101] = 8'd63;
    dense_w[102] = 8'd37;
    dense_w[103] = 8'd65;
    dense_w[104] = 8'd107;
    dense_w[105] = 8'd5;
    dense_w[106] = -8'd9;
    dense_w[107] = 8'd109;
    dense_w[108] = -8'd36;
    dense_w[109] = -8'd39;
    dense_w[110] = 8'd45;
    dense_w[111] = 8'd87;
    dense_w[112] = -8'd24;
    dense_w[113] = -8'd106;
    dense_w[114] = 8'd111;
    dense_w[115] = 8'd72;
    dense_w[116] = 8'd33;
    dense_w[117] = -8'd12;
    dense_w[118] = -8'd90;
    dense_w[119] = 8'd11;
    dense_w[120] = -8'd42;
    dense_w[121] = 8'd105;
    dense_w[122] = 8'd0;
    dense_w[123] = -8'd122;
    dense_w[124] = 8'd70;
    dense_w[125] = -8'd5;
    dense_w[126] = 8'd109;
    dense_w[127] = -8'd64;
end

reg signed [7:0] dense_b [0:7];
initial begin
    dense_b[0] = -8'd42;
    dense_b[1] = 8'd94;
    dense_b[2] = -8'd58;
    dense_b[3] = -8'd73;
    dense_b[4] = 8'd127;
    dense_b[5] = 8'd106;
    dense_b[6] = 8'd99;
    dense_b[7] = -8'd55;
end



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
      input_buffer[pix_cnt[9:0]] <= pixel_in;
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
          stage2_data <= $signed({8'b0, input_buffer[stage1_step[9:0]]}) - 16'sd128;
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





// --- From src/camera_interface.v ---
// ============================================================
// CAMERA INTERFACE MODULE
// ============================================================
// Captures 8-bit grayscale data from camera
// Generates internal frame/line valid signals
// ============================================================




module camera_interface (
    input wire clk,
    input wire rst_n,
    
    // External Camera Signals
    input wire cam_vsync,
    input wire cam_href,
    input wire [7:0] cam_data,
    
    // Internal Interface
    output reg [7:0] pixel_out,
    output reg pixel_valid,
    output reg frame_start,
    output reg frame_done
);

    // Synchronize camera signals to system clock
    // (Simplified synchronization for simulation)
    // In real hardware, we'd use proper 2-stage synchronizers or FIFO
    
    reg vsync_d;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_out <= 0;
            pixel_valid <= 0;
            frame_start <= 0;
            frame_done <= 0;
            vsync_d <= 0;
        end else begin
            // Edge detection for VSYNC
            vsync_d <= cam_vsync;
            
            // Frame Start (Rising edge of VSYNC for active high, or based on protocol)
            // Typically VSYNC inactive->active means start of frame
            frame_start <= (cam_vsync && !vsync_d);
            
            // Frame Done (Falling edge of VSYNC)
            frame_done <= (!cam_vsync && vsync_d);
            
            // Capture data when HREF is high (valid line)
            // We sample on system clock, assuming pclk is slower or synchronized
            if (cam_href) begin
                pixel_out <= cam_data;
                pixel_valid <= 1;
            end else begin
                pixel_valid <= 0;
            end
        end
    end

endmodule


// --- From src/uart_rx.v ---
// ============================================================
// UART RECEIVER MODULE
// ============================================================
// Baud rate: 115200
// Data bits: 8
// Stop bits: 1
// Parity: None
// ============================================================




module uart_rx #(
    parameter CLKS_PER_BIT = 434  // 50MHz / 115200 = 434
)(
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] rx_data,
    output reg rx_valid
);

// State machine
localparam IDLE = 2'd0;
localparam START = 2'd1;
localparam DATA = 2'd2;
localparam STOP = 2'd3;

reg [1:0] state;
reg [11:0] clk_count;
reg [2:0] bit_index;
reg [7:0] rx_byte;

// Synchronize RX input
reg rx_sync1, rx_sync2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_sync1 <= 1'b1;
        rx_sync2 <= 1'b1;
    end else begin
        rx_sync1 <= rx;
        rx_sync2 <= rx_sync1;
    end
end

// UART RX state machine
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        clk_count <= 0;
        bit_index <= 0;
        rx_data <= 0;
        rx_valid <= 0;
        rx_byte <= 0;
    end else begin
        rx_valid <= 0;  // Pulse, not level
        
        case (state)
            IDLE: begin
                clk_count <= 0;
                bit_index <= 0;
                if (rx_sync2 == 1'b0) begin  // Start bit detected
                    state <= START;
                end
            end
            
            START: begin
                if (clk_count == (CLKS_PER_BIT - 1) / 2) begin
                    if (rx_sync2 == 1'b0) begin  // Verify start bit
                        clk_count <= 0;
                        state <= DATA;
                    end else begin
                        state <= IDLE;  // False start
                    end
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            DATA: begin
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    rx_byte[bit_index] <= rx_sync2;
                    
                    if (bit_index == 7) begin
                        bit_index <= 0;
                        state <= STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            STOP: begin
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    if (rx_sync2 == 1'b1) begin  // Valid stop bit
                        rx_data <= rx_byte;
                        rx_valid <= 1'b1;
                    end
                    state <= IDLE;
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule

`default_nettype wire


// --- From src/uart_tx.v ---
// ============================================================
// UART TRANSMITTER MODULE
// ============================================================
// Baud rate: 115200
// Data bits: 8
// Stop bits: 1
// Parity: None
// ============================================================




module uart_tx #(
    parameter CLKS_PER_BIT = 434  // 50MHz / 115200 = 434
)(
    input wire clk,
    input wire rst_n,
    input wire [7:0] tx_data,
    input wire tx_valid,
    output reg tx,
    output wire tx_ready
);

// State machine
localparam IDLE = 2'd0;
localparam START = 2'd1;
localparam DATA = 2'd2;
localparam STOP = 2'd3;

reg [1:0] state;
reg [11:0] clk_count;
reg [2:0] bit_index;
reg [7:0] tx_byte;

assign tx_ready = (state == IDLE);

// UART TX state machine
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        clk_count <= 0;
        bit_index <= 0;
        tx <= 1'b1;  // Idle high
        tx_byte <= 0;
    end else begin
        case (state)
            IDLE: begin
                tx <= 1'b1;  // Idle high
                clk_count <= 0;
                bit_index <= 0;
                
                if (tx_valid) begin
                    tx_byte <= tx_data;
                    state <= START;
                end
            end
            
            START: begin
                tx <= 1'b0;  // Start bit (low)
                
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    state <= DATA;
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            DATA: begin
                tx <= tx_byte[bit_index];
                
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    
                    if (bit_index == 7) begin
                        bit_index <= 0;
                        state <= STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            STOP: begin
                tx <= 1'b1;  // Stop bit (high)
                
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    state <= IDLE;
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule

`default_nettype wire

