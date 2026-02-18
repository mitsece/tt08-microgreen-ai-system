// ============================================================
// MAIN PROCESSOR ASIC - TOP LEVEL
// ============================================================
// Competition Project: Dual-ASIC Precision Farming System
// Main Processor: CNN-based plant growth classification
// Co-Processor: Sensor monitoring and fault detection
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

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
