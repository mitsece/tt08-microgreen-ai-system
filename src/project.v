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
  wire [7:0] status_leds;
  wire [2:0] alert_level;
  wire harvest_alert;
  wire uart_tx;
  wire fault_detected;

  main_processor_asic main_proc (
    .clk(clk), .rst_n(rst_n & ena), .cam_pclk(clk), .cam_href(uio_in[5]),
    .cam_vsync(uio_in[6]), .cam_data(ui_in), .uart_rx(uio_in[0]), .uart_tx(uart_tx),
    .harvest_alert(harvest_alert), .alert_level(alert_level), .status_leds(status_leds), .fault_detected(fault_detected)
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
    output wire [7:0] status_leds, // Status indicators
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
wire uart_tx_ready;
wire uart_tx_valid;

// Sensor data from co-processor
reg [1:0] sensor_temp;
reg [1:0] sensor_humidity;
reg [1:0] sensor_light;
reg [1:0] sensor_soil;
reg [7:0] fault_flags;
reg [7:0] actuator_status;

// Decision signals
reg harvest_ready_cnn;
reg harvest_ready_sensors;
reg harvest_ready_final;

// ============================================================
// CAMERA INTERFACE
// ============================================================

camera_interface cam_if (
    .clk(clk),
    .rst_n(rst_n),
    .cam_pclk(cam_pclk),
    .cam_href(cam_href),
    .cam_vsync(cam_vsync),
    .cam_data(cam_data),
    .pixel_out(cnn_pixel),
    .pixel_valid(cnn_pixel_valid),
    .frame_start(cnn_frame_start)
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
    .tx_ready(uart_tx_ready)
);

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
        actuator_status <= 0;
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
                actuator_status <= uart_rx_data;
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

(* IOB = "true" *) reg [7:0] status_leds_reg;

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
        status_leds_reg[7] <= 1'b1;                  // Heartbeat (always on)
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

// Layer: conv2d_1
(* ram_style = "block" *) reg signed [7:0] conv2d_1_w [0:1151];
initial begin
    conv2d_1_w[0] = -8'd17;
    conv2d_1_w[1] = 8'd35;
    conv2d_1_w[2] = -8'd39;
    conv2d_1_w[3] = 8'd17;
    conv2d_1_w[4] = -8'd13;
    conv2d_1_w[5] = 8'd39;
    conv2d_1_w[6] = -8'd11;
    conv2d_1_w[7] = 8'd9;
    conv2d_1_w[8] = 8'd1;
    conv2d_1_w[9] = 8'd9;
    conv2d_1_w[10] = 8'd18;
    conv2d_1_w[11] = 8'd2;
    conv2d_1_w[12] = 8'd54;
    conv2d_1_w[13] = -8'd35;
    conv2d_1_w[14] = 8'd8;
    conv2d_1_w[15] = -8'd86;
    conv2d_1_w[16] = 8'd10;
    conv2d_1_w[17] = 8'd4;
    conv2d_1_w[18] = -8'd30;
    conv2d_1_w[19] = -8'd19;
    conv2d_1_w[20] = 8'd53;
    conv2d_1_w[21] = -8'd15;
    conv2d_1_w[22] = -8'd5;
    conv2d_1_w[23] = 8'd39;
    conv2d_1_w[24] = 8'd63;
    conv2d_1_w[25] = 8'd76;
    conv2d_1_w[26] = -8'd77;
    conv2d_1_w[27] = -8'd45;
    conv2d_1_w[28] = -8'd18;
    conv2d_1_w[29] = -8'd7;
    conv2d_1_w[30] = -8'd40;
    conv2d_1_w[31] = -8'd35;
    conv2d_1_w[32] = -8'd30;
    conv2d_1_w[33] = 8'd24;
    conv2d_1_w[34] = -8'd17;
    conv2d_1_w[35] = 8'd42;
    conv2d_1_w[36] = -8'd36;
    conv2d_1_w[37] = -8'd63;
    conv2d_1_w[38] = -8'd16;
    conv2d_1_w[39] = 8'd10;
    conv2d_1_w[40] = -8'd72;
    conv2d_1_w[41] = 8'd34;
    conv2d_1_w[42] = -8'd43;
    conv2d_1_w[43] = -8'd67;
    conv2d_1_w[44] = -8'd32;
    conv2d_1_w[45] = -8'd23;
    conv2d_1_w[46] = 8'd23;
    conv2d_1_w[47] = 8'd6;
    conv2d_1_w[48] = -8'd20;
    conv2d_1_w[49] = -8'd46;
    conv2d_1_w[50] = 8'd2;
    conv2d_1_w[51] = 8'd25;
    conv2d_1_w[52] = -8'd42;
    conv2d_1_w[53] = -8'd3;
    conv2d_1_w[54] = -8'd57;
    conv2d_1_w[55] = -8'd3;
    conv2d_1_w[56] = 8'd37;
    conv2d_1_w[57] = 8'd52;
    conv2d_1_w[58] = -8'd14;
    conv2d_1_w[59] = -8'd18;
    conv2d_1_w[60] = 8'd6;
    conv2d_1_w[61] = -8'd37;
    conv2d_1_w[62] = -8'd54;
    conv2d_1_w[63] = -8'd8;
    conv2d_1_w[64] = -8'd51;
    conv2d_1_w[65] = 8'd21;
    conv2d_1_w[66] = -8'd17;
    conv2d_1_w[67] = 8'd1;
    conv2d_1_w[68] = 8'd19;
    conv2d_1_w[69] = 8'd14;
    conv2d_1_w[70] = 8'd13;
    conv2d_1_w[71] = -8'd23;
    conv2d_1_w[72] = 8'd2;
    conv2d_1_w[73] = 8'd35;
    conv2d_1_w[74] = -8'd31;
    conv2d_1_w[75] = -8'd65;
    conv2d_1_w[76] = -8'd38;
    conv2d_1_w[77] = -8'd65;
    conv2d_1_w[78] = -8'd85;
    conv2d_1_w[79] = -8'd47;
    conv2d_1_w[80] = -8'd54;
    conv2d_1_w[81] = -8'd3;
    conv2d_1_w[82] = 8'd47;
    conv2d_1_w[83] = -8'd4;
    conv2d_1_w[84] = 8'd27;
    conv2d_1_w[85] = -8'd47;
    conv2d_1_w[86] = -8'd22;
    conv2d_1_w[87] = -8'd29;
    conv2d_1_w[88] = 8'd7;
    conv2d_1_w[89] = -8'd27;
    conv2d_1_w[90] = -8'd6;
    conv2d_1_w[91] = -8'd36;
    conv2d_1_w[92] = 8'd9;
    conv2d_1_w[93] = -8'd34;
    conv2d_1_w[94] = 8'd3;
    conv2d_1_w[95] = 8'd23;
    conv2d_1_w[96] = -8'd98;
    conv2d_1_w[97] = -8'd79;
    conv2d_1_w[98] = 8'd12;
    conv2d_1_w[99] = -8'd30;
    conv2d_1_w[100] = -8'd6;
    conv2d_1_w[101] = -8'd58;
    conv2d_1_w[102] = -8'd5;
    conv2d_1_w[103] = -8'd90;
    conv2d_1_w[104] = -8'd38;
    conv2d_1_w[105] = 8'd3;
    conv2d_1_w[106] = -8'd21;
    conv2d_1_w[107] = -8'd19;
    conv2d_1_w[108] = 8'd69;
    conv2d_1_w[109] = 8'd49;
    conv2d_1_w[110] = 8'd13;
    conv2d_1_w[111] = -8'd49;
    conv2d_1_w[112] = 8'd18;
    conv2d_1_w[113] = 8'd1;
    conv2d_1_w[114] = -8'd31;
    conv2d_1_w[115] = -8'd47;
    conv2d_1_w[116] = 8'd21;
    conv2d_1_w[117] = 8'd24;
    conv2d_1_w[118] = -8'd39;
    conv2d_1_w[119] = -8'd11;
    conv2d_1_w[120] = 8'd9;
    conv2d_1_w[121] = -8'd26;
    conv2d_1_w[122] = 8'd12;
    conv2d_1_w[123] = 8'd45;
    conv2d_1_w[124] = 8'd20;
    conv2d_1_w[125] = -8'd35;
    conv2d_1_w[126] = 8'd15;
    conv2d_1_w[127] = -8'd23;
    conv2d_1_w[128] = -8'd53;
    conv2d_1_w[129] = -8'd40;
    conv2d_1_w[130] = 8'd22;
    conv2d_1_w[131] = -8'd21;
    conv2d_1_w[132] = 8'd4;
    conv2d_1_w[133] = -8'd16;
    conv2d_1_w[134] = -8'd85;
    conv2d_1_w[135] = 8'd14;
    conv2d_1_w[136] = 8'd65;
    conv2d_1_w[137] = 8'd1;
    conv2d_1_w[138] = 8'd18;
    conv2d_1_w[139] = -8'd28;
    conv2d_1_w[140] = 8'd23;
    conv2d_1_w[141] = 8'd39;
    conv2d_1_w[142] = 8'd30;
    conv2d_1_w[143] = -8'd14;
    conv2d_1_w[144] = 8'd0;
    conv2d_1_w[145] = -8'd79;
    conv2d_1_w[146] = -8'd54;
    conv2d_1_w[147] = -8'd42;
    conv2d_1_w[148] = 8'd23;
    conv2d_1_w[149] = -8'd25;
    conv2d_1_w[150] = -8'd49;
    conv2d_1_w[151] = -8'd18;
    conv2d_1_w[152] = -8'd36;
    conv2d_1_w[153] = -8'd35;
    conv2d_1_w[154] = 8'd14;
    conv2d_1_w[155] = -8'd18;
    conv2d_1_w[156] = 8'd24;
    conv2d_1_w[157] = 8'd9;
    conv2d_1_w[158] = -8'd52;
    conv2d_1_w[159] = 8'd22;
    conv2d_1_w[160] = 8'd28;
    conv2d_1_w[161] = -8'd44;
    conv2d_1_w[162] = 8'd1;
    conv2d_1_w[163] = 8'd24;
    conv2d_1_w[164] = -8'd2;
    conv2d_1_w[165] = -8'd55;
    conv2d_1_w[166] = 8'd21;
    conv2d_1_w[167] = 8'd5;
    conv2d_1_w[168] = -8'd110;
    conv2d_1_w[169] = -8'd14;
    conv2d_1_w[170] = -8'd29;
    conv2d_1_w[171] = 8'd16;
    conv2d_1_w[172] = -8'd82;
    conv2d_1_w[173] = 8'd5;
    conv2d_1_w[174] = -8'd59;
    conv2d_1_w[175] = -8'd5;
    conv2d_1_w[176] = -8'd18;
    conv2d_1_w[177] = -8'd87;
    conv2d_1_w[178] = -8'd31;
    conv2d_1_w[179] = -8'd12;
    conv2d_1_w[180] = -8'd54;
    conv2d_1_w[181] = -8'd7;
    conv2d_1_w[182] = -8'd29;
    conv2d_1_w[183] = -8'd117;
    conv2d_1_w[184] = -8'd38;
    conv2d_1_w[185] = -8'd31;
    conv2d_1_w[186] = -8'd66;
    conv2d_1_w[187] = -8'd6;
    conv2d_1_w[188] = -8'd51;
    conv2d_1_w[189] = -8'd35;
    conv2d_1_w[190] = -8'd33;
    conv2d_1_w[191] = 8'd21;
    conv2d_1_w[192] = -8'd83;
    conv2d_1_w[193] = -8'd9;
    conv2d_1_w[194] = -8'd30;
    conv2d_1_w[195] = -8'd50;
    conv2d_1_w[196] = 8'd21;
    conv2d_1_w[197] = 8'd1;
    conv2d_1_w[198] = 8'd7;
    conv2d_1_w[199] = -8'd24;
    conv2d_1_w[200] = 8'd0;
    conv2d_1_w[201] = -8'd4;
    conv2d_1_w[202] = -8'd48;
    conv2d_1_w[203] = 8'd3;
    conv2d_1_w[204] = -8'd40;
    conv2d_1_w[205] = -8'd19;
    conv2d_1_w[206] = -8'd51;
    conv2d_1_w[207] = 8'd38;
    conv2d_1_w[208] = 8'd53;
    conv2d_1_w[209] = 8'd39;
    conv2d_1_w[210] = 8'd13;
    conv2d_1_w[211] = 8'd56;
    conv2d_1_w[212] = 8'd37;
    conv2d_1_w[213] = 8'd20;
    conv2d_1_w[214] = 8'd5;
    conv2d_1_w[215] = 8'd41;
    conv2d_1_w[216] = 8'd50;
    conv2d_1_w[217] = 8'd54;
    conv2d_1_w[218] = 8'd46;
    conv2d_1_w[219] = 8'd20;
    conv2d_1_w[220] = 8'd29;
    conv2d_1_w[221] = 8'd64;
    conv2d_1_w[222] = 8'd90;
    conv2d_1_w[223] = 8'd21;
    conv2d_1_w[224] = 8'd54;
    conv2d_1_w[225] = 8'd11;
    conv2d_1_w[226] = 8'd30;
    conv2d_1_w[227] = 8'd72;
    conv2d_1_w[228] = 8'd55;
    conv2d_1_w[229] = 8'd42;
    conv2d_1_w[230] = 8'd1;
    conv2d_1_w[231] = 8'd60;
    conv2d_1_w[232] = 8'd26;
    conv2d_1_w[233] = 8'd71;
    conv2d_1_w[234] = 8'd12;
    conv2d_1_w[235] = 8'd69;
    conv2d_1_w[236] = -8'd5;
    conv2d_1_w[237] = 8'd39;
    conv2d_1_w[238] = 8'd62;
    conv2d_1_w[239] = 8'd34;
    conv2d_1_w[240] = 8'd38;
    conv2d_1_w[241] = 8'd16;
    conv2d_1_w[242] = 8'd41;
    conv2d_1_w[243] = 8'd60;
    conv2d_1_w[244] = 8'd25;
    conv2d_1_w[245] = -8'd19;
    conv2d_1_w[246] = 8'd46;
    conv2d_1_w[247] = 8'd21;
    conv2d_1_w[248] = 8'd8;
    conv2d_1_w[249] = 8'd13;
    conv2d_1_w[250] = 8'd71;
    conv2d_1_w[251] = 8'd15;
    conv2d_1_w[252] = 8'd86;
    conv2d_1_w[253] = 8'd19;
    conv2d_1_w[254] = -8'd4;
    conv2d_1_w[255] = -8'd7;
    conv2d_1_w[256] = 8'd30;
    conv2d_1_w[257] = 8'd73;
    conv2d_1_w[258] = 8'd13;
    conv2d_1_w[259] = 8'd90;
    conv2d_1_w[260] = 8'd91;
    conv2d_1_w[261] = -8'd6;
    conv2d_1_w[262] = 8'd47;
    conv2d_1_w[263] = 8'd0;
    conv2d_1_w[264] = 8'd5;
    conv2d_1_w[265] = -8'd6;
    conv2d_1_w[266] = 8'd80;
    conv2d_1_w[267] = 8'd36;
    conv2d_1_w[268] = 8'd1;
    conv2d_1_w[269] = 8'd46;
    conv2d_1_w[270] = 8'd73;
    conv2d_1_w[271] = -8'd13;
    conv2d_1_w[272] = 8'd12;
    conv2d_1_w[273] = 8'd22;
    conv2d_1_w[274] = 8'd52;
    conv2d_1_w[275] = 8'd63;
    conv2d_1_w[276] = 8'd64;
    conv2d_1_w[277] = 8'd58;
    conv2d_1_w[278] = 8'd17;
    conv2d_1_w[279] = 8'd14;
    conv2d_1_w[280] = -8'd31;
    conv2d_1_w[281] = -8'd39;
    conv2d_1_w[282] = 8'd6;
    conv2d_1_w[283] = 8'd19;
    conv2d_1_w[284] = 8'd19;
    conv2d_1_w[285] = 8'd33;
    conv2d_1_w[286] = -8'd18;
    conv2d_1_w[287] = 8'd31;
    conv2d_1_w[288] = 8'd56;
    conv2d_1_w[289] = 8'd56;
    conv2d_1_w[290] = 8'd94;
    conv2d_1_w[291] = 8'd79;
    conv2d_1_w[292] = 8'd24;
    conv2d_1_w[293] = 8'd11;
    conv2d_1_w[294] = 8'd17;
    conv2d_1_w[295] = 8'd41;
    conv2d_1_w[296] = 8'd127;
    conv2d_1_w[297] = 8'd19;
    conv2d_1_w[298] = 8'd85;
    conv2d_1_w[299] = 8'd62;
    conv2d_1_w[300] = -8'd2;
    conv2d_1_w[301] = 8'd47;
    conv2d_1_w[302] = 8'd48;
    conv2d_1_w[303] = 8'd66;
    conv2d_1_w[304] = 8'd57;
    conv2d_1_w[305] = 8'd98;
    conv2d_1_w[306] = 8'd21;
    conv2d_1_w[307] = 8'd22;
    conv2d_1_w[308] = 8'd0;
    conv2d_1_w[309] = -8'd26;
    conv2d_1_w[310] = 8'd50;
    conv2d_1_w[311] = 8'd35;
    conv2d_1_w[312] = -8'd20;
    conv2d_1_w[313] = 8'd55;
    conv2d_1_w[314] = 8'd38;
    conv2d_1_w[315] = -8'd11;
    conv2d_1_w[316] = 8'd58;
    conv2d_1_w[317] = 8'd24;
    conv2d_1_w[318] = 8'd14;
    conv2d_1_w[319] = 8'd53;
    conv2d_1_w[320] = 8'd68;
    conv2d_1_w[321] = 8'd56;
    conv2d_1_w[322] = 8'd70;
    conv2d_1_w[323] = 8'd79;
    conv2d_1_w[324] = 8'd46;
    conv2d_1_w[325] = 8'd2;
    conv2d_1_w[326] = -8'd17;
    conv2d_1_w[327] = 8'd68;
    conv2d_1_w[328] = 8'd70;
    conv2d_1_w[329] = 8'd82;
    conv2d_1_w[330] = 8'd52;
    conv2d_1_w[331] = 8'd30;
    conv2d_1_w[332] = 8'd100;
    conv2d_1_w[333] = 8'd63;
    conv2d_1_w[334] = 8'd30;
    conv2d_1_w[335] = 8'd53;
    conv2d_1_w[336] = 8'd24;
    conv2d_1_w[337] = 8'd21;
    conv2d_1_w[338] = 8'd93;
    conv2d_1_w[339] = 8'd63;
    conv2d_1_w[340] = 8'd75;
    conv2d_1_w[341] = 8'd82;
    conv2d_1_w[342] = 8'd2;
    conv2d_1_w[343] = -8'd22;
    conv2d_1_w[344] = 8'd58;
    conv2d_1_w[345] = -8'd3;
    conv2d_1_w[346] = -8'd4;
    conv2d_1_w[347] = 8'd0;
    conv2d_1_w[348] = -8'd9;
    conv2d_1_w[349] = 8'd11;
    conv2d_1_w[350] = -8'd5;
    conv2d_1_w[351] = -8'd1;
    conv2d_1_w[352] = 8'd23;
    conv2d_1_w[353] = 8'd52;
    conv2d_1_w[354] = -8'd42;
    conv2d_1_w[355] = -8'd15;
    conv2d_1_w[356] = -8'd4;
    conv2d_1_w[357] = 8'd45;
    conv2d_1_w[358] = 8'd45;
    conv2d_1_w[359] = -8'd44;
    conv2d_1_w[360] = 8'd7;
    conv2d_1_w[361] = 8'd48;
    conv2d_1_w[362] = 8'd116;
    conv2d_1_w[363] = 8'd103;
    conv2d_1_w[364] = 8'd1;
    conv2d_1_w[365] = 8'd91;
    conv2d_1_w[366] = 8'd53;
    conv2d_1_w[367] = 8'd26;
    conv2d_1_w[368] = 8'd57;
    conv2d_1_w[369] = 8'd29;
    conv2d_1_w[370] = 8'd80;
    conv2d_1_w[371] = 8'd38;
    conv2d_1_w[372] = 8'd43;
    conv2d_1_w[373] = 8'd25;
    conv2d_1_w[374] = 8'd51;
    conv2d_1_w[375] = 8'd83;
    conv2d_1_w[376] = 8'd18;
    conv2d_1_w[377] = 8'd27;
    conv2d_1_w[378] = 8'd70;
    conv2d_1_w[379] = 8'd36;
    conv2d_1_w[380] = 8'd73;
    conv2d_1_w[381] = 8'd65;
    conv2d_1_w[382] = 8'd78;
    conv2d_1_w[383] = 8'd38;
    conv2d_1_w[384] = 8'd76;
    conv2d_1_w[385] = 8'd81;
    conv2d_1_w[386] = 8'd101;
    conv2d_1_w[387] = 8'd45;
    conv2d_1_w[388] = -8'd26;
    conv2d_1_w[389] = -8'd8;
    conv2d_1_w[390] = 8'd24;
    conv2d_1_w[391] = 8'd62;
    conv2d_1_w[392] = 8'd54;
    conv2d_1_w[393] = 8'd55;
    conv2d_1_w[394] = 8'd60;
    conv2d_1_w[395] = 8'd63;
    conv2d_1_w[396] = 8'd53;
    conv2d_1_w[397] = -8'd11;
    conv2d_1_w[398] = -8'd6;
    conv2d_1_w[399] = 8'd71;
    conv2d_1_w[400] = -8'd8;
    conv2d_1_w[401] = 8'd103;
    conv2d_1_w[402] = 8'd68;
    conv2d_1_w[403] = 8'd21;
    conv2d_1_w[404] = 8'd14;
    conv2d_1_w[405] = 8'd58;
    conv2d_1_w[406] = -8'd5;
    conv2d_1_w[407] = 8'd90;
    conv2d_1_w[408] = 8'd31;
    conv2d_1_w[409] = 8'd67;
    conv2d_1_w[410] = 8'd37;
    conv2d_1_w[411] = 8'd31;
    conv2d_1_w[412] = 8'd9;
    conv2d_1_w[413] = 8'd95;
    conv2d_1_w[414] = 8'd14;
    conv2d_1_w[415] = 8'd75;
    conv2d_1_w[416] = 8'd1;
    conv2d_1_w[417] = 8'd23;
    conv2d_1_w[418] = 8'd64;
    conv2d_1_w[419] = -8'd17;
    conv2d_1_w[420] = 8'd41;
    conv2d_1_w[421] = 8'd93;
    conv2d_1_w[422] = 8'd44;
    conv2d_1_w[423] = -8'd36;
    conv2d_1_w[424] = 8'd45;
    conv2d_1_w[425] = 8'd35;
    conv2d_1_w[426] = 8'd5;
    conv2d_1_w[427] = 8'd34;
    conv2d_1_w[428] = -8'd32;
    conv2d_1_w[429] = -8'd4;
    conv2d_1_w[430] = -8'd23;
    conv2d_1_w[431] = 8'd2;
    conv2d_1_w[432] = 8'd48;
    conv2d_1_w[433] = 8'd86;
    conv2d_1_w[434] = 8'd90;
    conv2d_1_w[435] = 8'd57;
    conv2d_1_w[436] = 8'd79;
    conv2d_1_w[437] = 8'd101;
    conv2d_1_w[438] = 8'd59;
    conv2d_1_w[439] = 8'd70;
    conv2d_1_w[440] = 8'd107;
    conv2d_1_w[441] = 8'd13;
    conv2d_1_w[442] = 8'd24;
    conv2d_1_w[443] = 8'd74;
    conv2d_1_w[444] = -8'd2;
    conv2d_1_w[445] = 8'd10;
    conv2d_1_w[446] = 8'd86;
    conv2d_1_w[447] = 8'd86;
    conv2d_1_w[448] = 8'd42;
    conv2d_1_w[449] = 8'd49;
    conv2d_1_w[450] = 8'd69;
    conv2d_1_w[451] = 8'd95;
    conv2d_1_w[452] = 8'd23;
    conv2d_1_w[453] = 8'd43;
    conv2d_1_w[454] = 8'd42;
    conv2d_1_w[455] = 8'd57;
    conv2d_1_w[456] = 8'd58;
    conv2d_1_w[457] = 8'd22;
    conv2d_1_w[458] = 8'd122;
    conv2d_1_w[459] = 8'd69;
    conv2d_1_w[460] = -8'd1;
    conv2d_1_w[461] = 8'd80;
    conv2d_1_w[462] = 8'd47;
    conv2d_1_w[463] = 8'd91;
    conv2d_1_w[464] = 8'd99;
    conv2d_1_w[465] = 8'd31;
    conv2d_1_w[466] = 8'd40;
    conv2d_1_w[467] = 8'd104;
    conv2d_1_w[468] = 8'd11;
    conv2d_1_w[469] = 8'd41;
    conv2d_1_w[470] = 8'd78;
    conv2d_1_w[471] = 8'd24;
    conv2d_1_w[472] = 8'd93;
    conv2d_1_w[473] = 8'd31;
    conv2d_1_w[474] = 8'd2;
    conv2d_1_w[475] = 8'd110;
    conv2d_1_w[476] = 8'd42;
    conv2d_1_w[477] = 8'd13;
    conv2d_1_w[478] = 8'd17;
    conv2d_1_w[479] = 8'd37;
    conv2d_1_w[480] = 8'd42;
    conv2d_1_w[481] = 8'd36;
    conv2d_1_w[482] = 8'd33;
    conv2d_1_w[483] = 8'd25;
    conv2d_1_w[484] = 8'd55;
    conv2d_1_w[485] = 8'd80;
    conv2d_1_w[486] = 8'd83;
    conv2d_1_w[487] = 8'd84;
    conv2d_1_w[488] = 8'd16;
    conv2d_1_w[489] = -8'd7;
    conv2d_1_w[490] = -8'd4;
    conv2d_1_w[491] = 8'd10;
    conv2d_1_w[492] = 8'd61;
    conv2d_1_w[493] = 8'd68;
    conv2d_1_w[494] = 8'd65;
    conv2d_1_w[495] = 8'd19;
    conv2d_1_w[496] = 8'd28;
    conv2d_1_w[497] = 8'd2;
    conv2d_1_w[498] = 8'd58;
    conv2d_1_w[499] = -8'd9;
    conv2d_1_w[500] = 8'd13;
    conv2d_1_w[501] = 8'd35;
    conv2d_1_w[502] = 8'd3;
    conv2d_1_w[503] = -8'd26;
    conv2d_1_w[504] = 8'd26;
    conv2d_1_w[505] = 8'd9;
    conv2d_1_w[506] = 8'd115;
    conv2d_1_w[507] = 8'd92;
    conv2d_1_w[508] = 8'd23;
    conv2d_1_w[509] = 8'd47;
    conv2d_1_w[510] = 8'd18;
    conv2d_1_w[511] = 8'd34;
    conv2d_1_w[512] = 8'd96;
    conv2d_1_w[513] = 8'd5;
    conv2d_1_w[514] = 8'd53;
    conv2d_1_w[515] = 8'd39;
    conv2d_1_w[516] = 8'd73;
    conv2d_1_w[517] = 8'd1;
    conv2d_1_w[518] = 8'd21;
    conv2d_1_w[519] = 8'd42;
    conv2d_1_w[520] = 8'd63;
    conv2d_1_w[521] = 8'd3;
    conv2d_1_w[522] = 8'd1;
    conv2d_1_w[523] = 8'd84;
    conv2d_1_w[524] = 8'd22;
    conv2d_1_w[525] = 8'd38;
    conv2d_1_w[526] = 8'd27;
    conv2d_1_w[527] = 8'd81;
    conv2d_1_w[528] = 8'd81;
    conv2d_1_w[529] = 8'd42;
    conv2d_1_w[530] = 8'd0;
    conv2d_1_w[531] = -8'd3;
    conv2d_1_w[532] = -8'd23;
    conv2d_1_w[533] = 8'd54;
    conv2d_1_w[534] = 8'd36;
    conv2d_1_w[535] = -8'd9;
    conv2d_1_w[536] = 8'd62;
    conv2d_1_w[537] = 8'd19;
    conv2d_1_w[538] = 8'd68;
    conv2d_1_w[539] = 8'd75;
    conv2d_1_w[540] = 8'd20;
    conv2d_1_w[541] = -8'd12;
    conv2d_1_w[542] = 8'd68;
    conv2d_1_w[543] = 8'd12;
    conv2d_1_w[544] = 8'd34;
    conv2d_1_w[545] = 8'd57;
    conv2d_1_w[546] = 8'd62;
    conv2d_1_w[547] = 8'd24;
    conv2d_1_w[548] = 8'd27;
    conv2d_1_w[549] = 8'd73;
    conv2d_1_w[550] = -8'd7;
    conv2d_1_w[551] = 8'd66;
    conv2d_1_w[552] = 8'd68;
    conv2d_1_w[553] = 8'd80;
    conv2d_1_w[554] = 8'd10;
    conv2d_1_w[555] = -8'd21;
    conv2d_1_w[556] = 8'd12;
    conv2d_1_w[557] = 8'd100;
    conv2d_1_w[558] = 8'd58;
    conv2d_1_w[559] = 8'd30;
    conv2d_1_w[560] = 8'd54;
    conv2d_1_w[561] = 8'd62;
    conv2d_1_w[562] = -8'd12;
    conv2d_1_w[563] = 8'd44;
    conv2d_1_w[564] = 8'd6;
    conv2d_1_w[565] = 8'd60;
    conv2d_1_w[566] = 8'd15;
    conv2d_1_w[567] = -8'd12;
    conv2d_1_w[568] = 8'd40;
    conv2d_1_w[569] = -8'd9;
    conv2d_1_w[570] = 8'd3;
    conv2d_1_w[571] = -8'd9;
    conv2d_1_w[572] = 8'd0;
    conv2d_1_w[573] = 8'd33;
    conv2d_1_w[574] = -8'd33;
    conv2d_1_w[575] = -8'd3;
    conv2d_1_w[576] = 8'd11;
    conv2d_1_w[577] = 8'd37;
    conv2d_1_w[578] = 8'd97;
    conv2d_1_w[579] = 8'd53;
    conv2d_1_w[580] = 8'd61;
    conv2d_1_w[581] = 8'd66;
    conv2d_1_w[582] = 8'd25;
    conv2d_1_w[583] = 8'd36;
    conv2d_1_w[584] = 8'd97;
    conv2d_1_w[585] = 8'd14;
    conv2d_1_w[586] = 8'd76;
    conv2d_1_w[587] = 8'd31;
    conv2d_1_w[588] = 8'd15;
    conv2d_1_w[589] = 8'd15;
    conv2d_1_w[590] = 8'd76;
    conv2d_1_w[591] = 8'd30;
    conv2d_1_w[592] = 8'd22;
    conv2d_1_w[593] = 8'd38;
    conv2d_1_w[594] = 8'd36;
    conv2d_1_w[595] = 8'd94;
    conv2d_1_w[596] = -8'd2;
    conv2d_1_w[597] = 8'd27;
    conv2d_1_w[598] = 8'd66;
    conv2d_1_w[599] = 8'd48;
    conv2d_1_w[600] = -8'd3;
    conv2d_1_w[601] = 8'd7;
    conv2d_1_w[602] = -8'd5;
    conv2d_1_w[603] = 8'd5;
    conv2d_1_w[604] = 8'd59;
    conv2d_1_w[605] = 8'd8;
    conv2d_1_w[606] = 8'd16;
    conv2d_1_w[607] = -8'd6;
    conv2d_1_w[608] = 8'd18;
    conv2d_1_w[609] = 8'd63;
    conv2d_1_w[610] = 8'd50;
    conv2d_1_w[611] = 8'd17;
    conv2d_1_w[612] = 8'd56;
    conv2d_1_w[613] = 8'd10;
    conv2d_1_w[614] = -8'd8;
    conv2d_1_w[615] = 8'd69;
    conv2d_1_w[616] = 8'd40;
    conv2d_1_w[617] = 8'd62;
    conv2d_1_w[618] = 8'd70;
    conv2d_1_w[619] = 8'd24;
    conv2d_1_w[620] = -8'd2;
    conv2d_1_w[621] = -8'd16;
    conv2d_1_w[622] = -8'd8;
    conv2d_1_w[623] = 8'd57;
    conv2d_1_w[624] = 8'd44;
    conv2d_1_w[625] = 8'd10;
    conv2d_1_w[626] = 8'd90;
    conv2d_1_w[627] = 8'd47;
    conv2d_1_w[628] = 8'd14;
    conv2d_1_w[629] = 8'd31;
    conv2d_1_w[630] = -8'd12;
    conv2d_1_w[631] = 8'd39;
    conv2d_1_w[632] = 8'd7;
    conv2d_1_w[633] = -8'd11;
    conv2d_1_w[634] = 8'd63;
    conv2d_1_w[635] = 8'd73;
    conv2d_1_w[636] = -8'd23;
    conv2d_1_w[637] = 8'd8;
    conv2d_1_w[638] = 8'd53;
    conv2d_1_w[639] = 8'd38;
    conv2d_1_w[640] = 8'd3;
    conv2d_1_w[641] = 8'd57;
    conv2d_1_w[642] = -8'd9;
    conv2d_1_w[643] = 8'd17;
    conv2d_1_w[644] = 8'd52;
    conv2d_1_w[645] = 8'd35;
    conv2d_1_w[646] = 8'd20;
    conv2d_1_w[647] = 8'd36;
    conv2d_1_w[648] = 8'd2;
    conv2d_1_w[649] = 8'd9;
    conv2d_1_w[650] = -8'd37;
    conv2d_1_w[651] = -8'd73;
    conv2d_1_w[652] = 8'd23;
    conv2d_1_w[653] = -8'd33;
    conv2d_1_w[654] = -8'd43;
    conv2d_1_w[655] = -8'd3;
    conv2d_1_w[656] = -8'd47;
    conv2d_1_w[657] = 8'd42;
    conv2d_1_w[658] = 8'd55;
    conv2d_1_w[659] = 8'd22;
    conv2d_1_w[660] = -8'd8;
    conv2d_1_w[661] = -8'd33;
    conv2d_1_w[662] = -8'd89;
    conv2d_1_w[663] = 8'd10;
    conv2d_1_w[664] = -8'd43;
    conv2d_1_w[665] = -8'd49;
    conv2d_1_w[666] = -8'd94;
    conv2d_1_w[667] = -8'd7;
    conv2d_1_w[668] = 8'd9;
    conv2d_1_w[669] = -8'd9;
    conv2d_1_w[670] = 8'd31;
    conv2d_1_w[671] = 8'd0;
    conv2d_1_w[672] = -8'd45;
    conv2d_1_w[673] = 8'd33;
    conv2d_1_w[674] = -8'd43;
    conv2d_1_w[675] = -8'd5;
    conv2d_1_w[676] = -8'd83;
    conv2d_1_w[677] = -8'd59;
    conv2d_1_w[678] = -8'd54;
    conv2d_1_w[679] = -8'd84;
    conv2d_1_w[680] = -8'd58;
    conv2d_1_w[681] = -8'd84;
    conv2d_1_w[682] = -8'd44;
    conv2d_1_w[683] = -8'd47;
    conv2d_1_w[684] = 8'd49;
    conv2d_1_w[685] = -8'd8;
    conv2d_1_w[686] = -8'd42;
    conv2d_1_w[687] = 8'd21;
    conv2d_1_w[688] = -8'd71;
    conv2d_1_w[689] = -8'd36;
    conv2d_1_w[690] = 8'd20;
    conv2d_1_w[691] = 8'd8;
    conv2d_1_w[692] = -8'd38;
    conv2d_1_w[693] = 8'd1;
    conv2d_1_w[694] = -8'd44;
    conv2d_1_w[695] = -8'd6;
    conv2d_1_w[696] = -8'd26;
    conv2d_1_w[697] = 8'd7;
    conv2d_1_w[698] = 8'd2;
    conv2d_1_w[699] = 8'd40;
    conv2d_1_w[700] = -8'd24;
    conv2d_1_w[701] = -8'd71;
    conv2d_1_w[702] = -8'd3;
    conv2d_1_w[703] = 8'd22;
    conv2d_1_w[704] = 8'd43;
    conv2d_1_w[705] = -8'd2;
    conv2d_1_w[706] = -8'd35;
    conv2d_1_w[707] = -8'd70;
    conv2d_1_w[708] = -8'd61;
    conv2d_1_w[709] = -8'd46;
    conv2d_1_w[710] = -8'd46;
    conv2d_1_w[711] = 8'd68;
    conv2d_1_w[712] = 8'd46;
    conv2d_1_w[713] = 8'd89;
    conv2d_1_w[714] = 8'd52;
    conv2d_1_w[715] = 8'd30;
    conv2d_1_w[716] = 8'd29;
    conv2d_1_w[717] = 8'd39;
    conv2d_1_w[718] = 8'd30;
    conv2d_1_w[719] = 8'd24;
    conv2d_1_w[720] = 8'd59;
    conv2d_1_w[721] = -8'd4;
    conv2d_1_w[722] = 8'd72;
    conv2d_1_w[723] = 8'd100;
    conv2d_1_w[724] = 8'd83;
    conv2d_1_w[725] = 8'd68;
    conv2d_1_w[726] = 8'd34;
    conv2d_1_w[727] = 8'd91;
    conv2d_1_w[728] = 8'd113;
    conv2d_1_w[729] = 8'd79;
    conv2d_1_w[730] = 8'd51;
    conv2d_1_w[731] = -8'd3;
    conv2d_1_w[732] = 8'd42;
    conv2d_1_w[733] = 8'd51;
    conv2d_1_w[734] = 8'd14;
    conv2d_1_w[735] = 8'd57;
    conv2d_1_w[736] = 8'd67;
    conv2d_1_w[737] = 8'd41;
    conv2d_1_w[738] = 8'd15;
    conv2d_1_w[739] = 8'd38;
    conv2d_1_w[740] = 8'd19;
    conv2d_1_w[741] = 8'd15;
    conv2d_1_w[742] = 8'd4;
    conv2d_1_w[743] = 8'd1;
    conv2d_1_w[744] = -8'd10;
    conv2d_1_w[745] = 8'd35;
    conv2d_1_w[746] = 8'd13;
    conv2d_1_w[747] = -8'd17;
    conv2d_1_w[748] = 8'd28;
    conv2d_1_w[749] = 8'd41;
    conv2d_1_w[750] = -8'd2;
    conv2d_1_w[751] = 8'd32;
    conv2d_1_w[752] = 8'd64;
    conv2d_1_w[753] = 8'd37;
    conv2d_1_w[754] = 8'd22;
    conv2d_1_w[755] = 8'd29;
    conv2d_1_w[756] = -8'd2;
    conv2d_1_w[757] = 8'd9;
    conv2d_1_w[758] = 8'd43;
    conv2d_1_w[759] = 8'd41;
    conv2d_1_w[760] = 8'd22;
    conv2d_1_w[761] = 8'd58;
    conv2d_1_w[762] = 8'd20;
    conv2d_1_w[763] = 8'd90;
    conv2d_1_w[764] = 8'd74;
    conv2d_1_w[765] = 8'd6;
    conv2d_1_w[766] = 8'd14;
    conv2d_1_w[767] = -8'd3;
    conv2d_1_w[768] = 8'd64;
    conv2d_1_w[769] = -8'd3;
    conv2d_1_w[770] = 8'd38;
    conv2d_1_w[771] = 8'd35;
    conv2d_1_w[772] = 8'd75;
    conv2d_1_w[773] = 8'd59;
    conv2d_1_w[774] = 8'd76;
    conv2d_1_w[775] = 8'd25;
    conv2d_1_w[776] = 8'd69;
    conv2d_1_w[777] = 8'd60;
    conv2d_1_w[778] = -8'd12;
    conv2d_1_w[779] = 8'd43;
    conv2d_1_w[780] = 8'd36;
    conv2d_1_w[781] = 8'd4;
    conv2d_1_w[782] = 8'd85;
    conv2d_1_w[783] = -8'd27;
    conv2d_1_w[784] = 8'd28;
    conv2d_1_w[785] = 8'd1;
    conv2d_1_w[786] = 8'd20;
    conv2d_1_w[787] = -8'd47;
    conv2d_1_w[788] = -8'd1;
    conv2d_1_w[789] = 8'd7;
    conv2d_1_w[790] = 8'd25;
    conv2d_1_w[791] = 8'd57;
    conv2d_1_w[792] = 8'd39;
    conv2d_1_w[793] = 8'd11;
    conv2d_1_w[794] = 8'd46;
    conv2d_1_w[795] = 8'd28;
    conv2d_1_w[796] = 8'd28;
    conv2d_1_w[797] = 8'd50;
    conv2d_1_w[798] = 8'd52;
    conv2d_1_w[799] = 8'd87;
    conv2d_1_w[800] = 8'd36;
    conv2d_1_w[801] = 8'd43;
    conv2d_1_w[802] = 8'd73;
    conv2d_1_w[803] = -8'd10;
    conv2d_1_w[804] = 8'd57;
    conv2d_1_w[805] = -8'd7;
    conv2d_1_w[806] = 8'd27;
    conv2d_1_w[807] = 8'd59;
    conv2d_1_w[808] = 8'd25;
    conv2d_1_w[809] = 8'd56;
    conv2d_1_w[810] = 8'd38;
    conv2d_1_w[811] = 8'd30;
    conv2d_1_w[812] = 8'd64;
    conv2d_1_w[813] = 8'd68;
    conv2d_1_w[814] = 8'd34;
    conv2d_1_w[815] = 8'd14;
    conv2d_1_w[816] = 8'd73;
    conv2d_1_w[817] = 8'd45;
    conv2d_1_w[818] = 8'd49;
    conv2d_1_w[819] = -8'd18;
    conv2d_1_w[820] = 8'd16;
    conv2d_1_w[821] = -8'd20;
    conv2d_1_w[822] = -8'd14;
    conv2d_1_w[823] = 8'd56;
    conv2d_1_w[824] = 8'd19;
    conv2d_1_w[825] = -8'd11;
    conv2d_1_w[826] = 8'd25;
    conv2d_1_w[827] = 8'd57;
    conv2d_1_w[828] = 8'd36;
    conv2d_1_w[829] = 8'd7;
    conv2d_1_w[830] = 8'd8;
    conv2d_1_w[831] = 8'd75;
    conv2d_1_w[832] = 8'd28;
    conv2d_1_w[833] = 8'd55;
    conv2d_1_w[834] = 8'd40;
    conv2d_1_w[835] = 8'd97;
    conv2d_1_w[836] = 8'd26;
    conv2d_1_w[837] = 8'd36;
    conv2d_1_w[838] = 8'd66;
    conv2d_1_w[839] = -8'd2;
    conv2d_1_w[840] = 8'd47;
    conv2d_1_w[841] = 8'd66;
    conv2d_1_w[842] = 8'd71;
    conv2d_1_w[843] = -8'd3;
    conv2d_1_w[844] = 8'd37;
    conv2d_1_w[845] = 8'd56;
    conv2d_1_w[846] = 8'd62;
    conv2d_1_w[847] = 8'd30;
    conv2d_1_w[848] = 8'd58;
    conv2d_1_w[849] = 8'd29;
    conv2d_1_w[850] = 8'd13;
    conv2d_1_w[851] = 8'd17;
    conv2d_1_w[852] = 8'd66;
    conv2d_1_w[853] = 8'd23;
    conv2d_1_w[854] = 8'd86;
    conv2d_1_w[855] = 8'd50;
    conv2d_1_w[856] = 8'd4;
    conv2d_1_w[857] = 8'd22;
    conv2d_1_w[858] = -8'd16;
    conv2d_1_w[859] = -8'd34;
    conv2d_1_w[860] = 8'd33;
    conv2d_1_w[861] = -8'd19;
    conv2d_1_w[862] = -8'd7;
    conv2d_1_w[863] = 8'd13;
    conv2d_1_w[864] = 8'd79;
    conv2d_1_w[865] = 8'd58;
    conv2d_1_w[866] = 8'd62;
    conv2d_1_w[867] = 8'd96;
    conv2d_1_w[868] = 8'd2;
    conv2d_1_w[869] = 8'd28;
    conv2d_1_w[870] = 8'd59;
    conv2d_1_w[871] = 8'd37;
    conv2d_1_w[872] = 8'd31;
    conv2d_1_w[873] = 8'd71;
    conv2d_1_w[874] = 8'd13;
    conv2d_1_w[875] = 8'd17;
    conv2d_1_w[876] = 8'd34;
    conv2d_1_w[877] = 8'd36;
    conv2d_1_w[878] = -8'd1;
    conv2d_1_w[879] = 8'd41;
    conv2d_1_w[880] = 8'd14;
    conv2d_1_w[881] = 8'd18;
    conv2d_1_w[882] = 8'd84;
    conv2d_1_w[883] = 8'd71;
    conv2d_1_w[884] = 8'd21;
    conv2d_1_w[885] = 8'd85;
    conv2d_1_w[886] = 8'd91;
    conv2d_1_w[887] = -8'd1;
    conv2d_1_w[888] = 8'd4;
    conv2d_1_w[889] = 8'd51;
    conv2d_1_w[890] = 8'd9;
    conv2d_1_w[891] = 8'd21;
    conv2d_1_w[892] = 8'd38;
    conv2d_1_w[893] = 8'd6;
    conv2d_1_w[894] = 8'd25;
    conv2d_1_w[895] = 8'd23;
    conv2d_1_w[896] = 8'd57;
    conv2d_1_w[897] = 8'd49;
    conv2d_1_w[898] = 8'd24;
    conv2d_1_w[899] = 8'd42;
    conv2d_1_w[900] = 8'd35;
    conv2d_1_w[901] = 8'd27;
    conv2d_1_w[902] = 8'd13;
    conv2d_1_w[903] = 8'd30;
    conv2d_1_w[904] = 8'd36;
    conv2d_1_w[905] = 8'd7;
    conv2d_1_w[906] = 8'd45;
    conv2d_1_w[907] = 8'd29;
    conv2d_1_w[908] = 8'd60;
    conv2d_1_w[909] = 8'd29;
    conv2d_1_w[910] = 8'd25;
    conv2d_1_w[911] = 8'd23;
    conv2d_1_w[912] = 8'd8;
    conv2d_1_w[913] = 8'd63;
    conv2d_1_w[914] = 8'd74;
    conv2d_1_w[915] = 8'd45;
    conv2d_1_w[916] = 8'd61;
    conv2d_1_w[917] = 8'd65;
    conv2d_1_w[918] = 8'd68;
    conv2d_1_w[919] = 8'd70;
    conv2d_1_w[920] = 8'd22;
    conv2d_1_w[921] = 8'd50;
    conv2d_1_w[922] = 8'd68;
    conv2d_1_w[923] = -8'd1;
    conv2d_1_w[924] = 8'd36;
    conv2d_1_w[925] = 8'd76;
    conv2d_1_w[926] = 8'd78;
    conv2d_1_w[927] = 8'd43;
    conv2d_1_w[928] = -8'd28;
    conv2d_1_w[929] = 8'd35;
    conv2d_1_w[930] = 8'd13;
    conv2d_1_w[931] = 8'd45;
    conv2d_1_w[932] = 8'd55;
    conv2d_1_w[933] = 8'd45;
    conv2d_1_w[934] = 8'd44;
    conv2d_1_w[935] = 8'd45;
    conv2d_1_w[936] = 8'd84;
    conv2d_1_w[937] = 8'd45;
    conv2d_1_w[938] = 8'd39;
    conv2d_1_w[939] = 8'd66;
    conv2d_1_w[940] = 8'd75;
    conv2d_1_w[941] = 8'd51;
    conv2d_1_w[942] = 8'd111;
    conv2d_1_w[943] = 8'd76;
    conv2d_1_w[944] = 8'd98;
    conv2d_1_w[945] = 8'd46;
    conv2d_1_w[946] = 8'd21;
    conv2d_1_w[947] = 8'd5;
    conv2d_1_w[948] = -8'd28;
    conv2d_1_w[949] = 8'd69;
    conv2d_1_w[950] = 8'd16;
    conv2d_1_w[951] = 8'd89;
    conv2d_1_w[952] = 8'd30;
    conv2d_1_w[953] = 8'd59;
    conv2d_1_w[954] = 8'd26;
    conv2d_1_w[955] = 8'd21;
    conv2d_1_w[956] = 8'd98;
    conv2d_1_w[957] = 8'd24;
    conv2d_1_w[958] = 8'd69;
    conv2d_1_w[959] = 8'd27;
    conv2d_1_w[960] = 8'd62;
    conv2d_1_w[961] = 8'd68;
    conv2d_1_w[962] = 8'd57;
    conv2d_1_w[963] = 8'd64;
    conv2d_1_w[964] = 8'd72;
    conv2d_1_w[965] = 8'd35;
    conv2d_1_w[966] = 8'd61;
    conv2d_1_w[967] = 8'd50;
    conv2d_1_w[968] = 8'd39;
    conv2d_1_w[969] = 8'd48;
    conv2d_1_w[970] = 8'd90;
    conv2d_1_w[971] = 8'd56;
    conv2d_1_w[972] = -8'd12;
    conv2d_1_w[973] = 8'd109;
    conv2d_1_w[974] = -8'd8;
    conv2d_1_w[975] = 8'd40;
    conv2d_1_w[976] = 8'd56;
    conv2d_1_w[977] = 8'd21;
    conv2d_1_w[978] = 8'd91;
    conv2d_1_w[979] = 8'd69;
    conv2d_1_w[980] = 8'd97;
    conv2d_1_w[981] = 8'd27;
    conv2d_1_w[982] = 8'd80;
    conv2d_1_w[983] = 8'd68;
    conv2d_1_w[984] = 8'd23;
    conv2d_1_w[985] = 8'd8;
    conv2d_1_w[986] = 8'd26;
    conv2d_1_w[987] = 8'd70;
    conv2d_1_w[988] = -8'd7;
    conv2d_1_w[989] = 8'd76;
    conv2d_1_w[990] = 8'd26;
    conv2d_1_w[991] = -8'd11;
    conv2d_1_w[992] = -8'd4;
    conv2d_1_w[993] = 8'd15;
    conv2d_1_w[994] = 8'd31;
    conv2d_1_w[995] = 8'd37;
    conv2d_1_w[996] = 8'd18;
    conv2d_1_w[997] = 8'd71;
    conv2d_1_w[998] = 8'd65;
    conv2d_1_w[999] = 8'd0;
    conv2d_1_w[1000] = 8'd36;
    conv2d_1_w[1001] = 8'd50;
    conv2d_1_w[1002] = 8'd10;
    conv2d_1_w[1003] = 8'd9;
    conv2d_1_w[1004] = -8'd21;
    conv2d_1_w[1005] = 8'd6;
    conv2d_1_w[1006] = 8'd20;
    conv2d_1_w[1007] = -8'd19;
    conv2d_1_w[1008] = 8'd76;
    conv2d_1_w[1009] = 8'd32;
    conv2d_1_w[1010] = 8'd24;
    conv2d_1_w[1011] = 8'd43;
    conv2d_1_w[1012] = 8'd16;
    conv2d_1_w[1013] = 8'd18;
    conv2d_1_w[1014] = 8'd61;
    conv2d_1_w[1015] = 8'd40;
    conv2d_1_w[1016] = 8'd91;
    conv2d_1_w[1017] = 8'd45;
    conv2d_1_w[1018] = -8'd7;
    conv2d_1_w[1019] = 8'd23;
    conv2d_1_w[1020] = 8'd52;
    conv2d_1_w[1021] = 8'd48;
    conv2d_1_w[1022] = 8'd88;
    conv2d_1_w[1023] = 8'd108;
    conv2d_1_w[1024] = 8'd29;
    conv2d_1_w[1025] = 8'd43;
    conv2d_1_w[1026] = 8'd37;
    conv2d_1_w[1027] = 8'd80;
    conv2d_1_w[1028] = 8'd60;
    conv2d_1_w[1029] = 8'd57;
    conv2d_1_w[1030] = 8'd50;
    conv2d_1_w[1031] = 8'd70;
    conv2d_1_w[1032] = 8'd97;
    conv2d_1_w[1033] = 8'd53;
    conv2d_1_w[1034] = 8'd16;
    conv2d_1_w[1035] = -8'd6;
    conv2d_1_w[1036] = 8'd40;
    conv2d_1_w[1037] = 8'd42;
    conv2d_1_w[1038] = 8'd24;
    conv2d_1_w[1039] = 8'd9;
    conv2d_1_w[1040] = 8'd35;
    conv2d_1_w[1041] = 8'd71;
    conv2d_1_w[1042] = 8'd16;
    conv2d_1_w[1043] = 8'd32;
    conv2d_1_w[1044] = 8'd19;
    conv2d_1_w[1045] = 8'd60;
    conv2d_1_w[1046] = 8'd66;
    conv2d_1_w[1047] = 8'd38;
    conv2d_1_w[1048] = 8'd75;
    conv2d_1_w[1049] = 8'd16;
    conv2d_1_w[1050] = 8'd71;
    conv2d_1_w[1051] = 8'd101;
    conv2d_1_w[1052] = 8'd15;
    conv2d_1_w[1053] = -8'd18;
    conv2d_1_w[1054] = 8'd52;
    conv2d_1_w[1055] = 8'd2;
    conv2d_1_w[1056] = 8'd1;
    conv2d_1_w[1057] = 8'd34;
    conv2d_1_w[1058] = 8'd66;
    conv2d_1_w[1059] = 8'd16;
    conv2d_1_w[1060] = -8'd1;
    conv2d_1_w[1061] = 8'd85;
    conv2d_1_w[1062] = 8'd44;
    conv2d_1_w[1063] = 8'd50;
    conv2d_1_w[1064] = 8'd58;
    conv2d_1_w[1065] = 8'd69;
    conv2d_1_w[1066] = 8'd25;
    conv2d_1_w[1067] = 8'd37;
    conv2d_1_w[1068] = 8'd87;
    conv2d_1_w[1069] = -8'd11;
    conv2d_1_w[1070] = 8'd82;
    conv2d_1_w[1071] = -8'd30;
    conv2d_1_w[1072] = -8'd4;
    conv2d_1_w[1073] = 8'd14;
    conv2d_1_w[1074] = 8'd15;
    conv2d_1_w[1075] = -8'd23;
    conv2d_1_w[1076] = -8'd20;
    conv2d_1_w[1077] = 8'd43;
    conv2d_1_w[1078] = 8'd39;
    conv2d_1_w[1079] = 8'd23;
    conv2d_1_w[1080] = 8'd19;
    conv2d_1_w[1081] = 8'd21;
    conv2d_1_w[1082] = 8'd0;
    conv2d_1_w[1083] = 8'd17;
    conv2d_1_w[1084] = 8'd18;
    conv2d_1_w[1085] = -8'd94;
    conv2d_1_w[1086] = -8'd23;
    conv2d_1_w[1087] = -8'd9;
    conv2d_1_w[1088] = -8'd49;
    conv2d_1_w[1089] = -8'd22;
    conv2d_1_w[1090] = 8'd56;
    conv2d_1_w[1091] = -8'd97;
    conv2d_1_w[1092] = -8'd27;
    conv2d_1_w[1093] = 8'd56;
    conv2d_1_w[1094] = -8'd90;
    conv2d_1_w[1095] = 8'd6;
    conv2d_1_w[1096] = -8'd3;
    conv2d_1_w[1097] = -8'd56;
    conv2d_1_w[1098] = 8'd14;
    conv2d_1_w[1099] = -8'd7;
    conv2d_1_w[1100] = -8'd100;
    conv2d_1_w[1101] = 8'd2;
    conv2d_1_w[1102] = 8'd65;
    conv2d_1_w[1103] = 8'd14;
    conv2d_1_w[1104] = 8'd22;
    conv2d_1_w[1105] = 8'd67;
    conv2d_1_w[1106] = -8'd65;
    conv2d_1_w[1107] = -8'd61;
    conv2d_1_w[1108] = -8'd38;
    conv2d_1_w[1109] = -8'd73;
    conv2d_1_w[1110] = -8'd38;
    conv2d_1_w[1111] = -8'd36;
    conv2d_1_w[1112] = 8'd20;
    conv2d_1_w[1113] = -8'd34;
    conv2d_1_w[1114] = -8'd75;
    conv2d_1_w[1115] = -8'd49;
    conv2d_1_w[1116] = -8'd110;
    conv2d_1_w[1117] = 8'd40;
    conv2d_1_w[1118] = 8'd18;
    conv2d_1_w[1119] = -8'd124;
    conv2d_1_w[1120] = 8'd36;
    conv2d_1_w[1121] = -8'd64;
    conv2d_1_w[1122] = -8'd109;
    conv2d_1_w[1123] = -8'd2;
    conv2d_1_w[1124] = -8'd12;
    conv2d_1_w[1125] = -8'd38;
    conv2d_1_w[1126] = 8'd23;
    conv2d_1_w[1127] = 8'd26;
    conv2d_1_w[1128] = -8'd72;
    conv2d_1_w[1129] = 8'd20;
    conv2d_1_w[1130] = 8'd11;
    conv2d_1_w[1131] = -8'd5;
    conv2d_1_w[1132] = 8'd56;
    conv2d_1_w[1133] = 8'd41;
    conv2d_1_w[1134] = -8'd13;
    conv2d_1_w[1135] = 8'd8;
    conv2d_1_w[1136] = -8'd72;
    conv2d_1_w[1137] = -8'd39;
    conv2d_1_w[1138] = 8'd27;
    conv2d_1_w[1139] = 8'd5;
    conv2d_1_w[1140] = 8'd33;
    conv2d_1_w[1141] = 8'd45;
    conv2d_1_w[1142] = -8'd58;
    conv2d_1_w[1143] = -8'd28;
    conv2d_1_w[1144] = 8'd25;
    conv2d_1_w[1145] = 8'd37;
    conv2d_1_w[1146] = -8'd21;
    conv2d_1_w[1147] = 8'd18;
    conv2d_1_w[1148] = 8'd58;
    conv2d_1_w[1149] = -8'd21;
    conv2d_1_w[1150] = -8'd21;
    conv2d_1_w[1151] = 8'd17;
end

reg signed [7:0] conv2d_1_b [0:15];
initial begin
    conv2d_1_b[0] = -8'd9;
    conv2d_1_b[1] = 8'd2;
    conv2d_1_b[2] = 8'd100;
    conv2d_1_b[3] = -8'd64;
    conv2d_1_b[4] = -8'd93;
    conv2d_1_b[5] = -8'd73;
    conv2d_1_b[6] = -8'd127;
    conv2d_1_b[7] = -8'd69;
    conv2d_1_b[8] = -8'd38;
    conv2d_1_b[9] = 8'd59;
    conv2d_1_b[10] = -8'd82;
    conv2d_1_b[11] = -8'd92;
    conv2d_1_b[12] = -8'd78;
    conv2d_1_b[13] = -8'd110;
    conv2d_1_b[14] = -8'd89;
    conv2d_1_b[15] = 8'd0;
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

// Layer: dense_1
reg signed [7:0] dense_1_w [0:7];
initial begin
    dense_1_w[0] = -8'd103;
    dense_1_w[1] = 8'd127;
    dense_1_w[2] = -8'd105;
    dense_1_w[3] = -8'd93;
    dense_1_w[4] = 8'd45;
    dense_1_w[5] = 8'd81;
    dense_1_w[6] = 8'd108;
    dense_1_w[7] = -8'd95;
end

reg signed [7:0] dense_1_b [0:0];
initial begin
    dense_1_b[0] = 8'd127;
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
    input wire cam_pclk,
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
    
    reg vsync_d, href_d;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_out <= 0;
            pixel_valid <= 0;
            frame_start <= 0;
            frame_done <= 0;
            vsync_d <= 0;
            href_d <= 0;
        end else begin
            // Edge detection for VSYNC
            vsync_d <= cam_vsync;
            href_d <= cam_href;
            
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

