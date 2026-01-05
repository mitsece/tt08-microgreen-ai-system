// ============================================================
// TINY TAPEOUT WRAPPER - PRECISION FARMING ASIC
// ============================================================
// Wraps the main processor ASIC for Tiny Tapeout submission
// Tile Size: 2x2 (for CNN + UART + Camera Interface)
// ============================================================

`default_nettype none

module tt_um_precision_farming (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // Always 1 when the design is powered
    input  wire       clk,      // Clock
    input  wire       rst_n     // Reset (active low)
);

  // ============================================================
  // PIN MAPPING
  // ============================================================
  // Inputs (ui_in):
  //   [7:0] - Camera/Sensor Data (8-bit pixel or sensor reading)
  //
  // Bidirectional (uio):
  //   [0]   - Sensor Select 0 (INPUT)
  //   [1]   - Sensor Select 1 (INPUT)
  //   [5]   - Camera HREF (INPUT)
  //   [6]   - Camera VSYNC (INPUT)
  //   [7]   - Mode Select (INPUT)
  //
  // Outputs (uo_out):
  //   [6:0] - Status LEDs
  //   [7]   - Harvest Alert / Buzzer
  
  // ============================================================
  // INTERNAL SIGNALS
  // ============================================================
  
  // Camera interface signals
  wire cam_pclk;
  wire cam_href;
  wire cam_vsync;
  wire [7:0] cam_data;
  
  // UART signals (simplified for TT - use as debug/config)
  wire uart_rx;
  wire uart_tx;
  
  // Output signals from main processor
  wire harvest_alert;
  wire [2:0] alert_level;
  wire [7:0] status_leds;
  wire fault_detected;
  
  // ============================================================
  // INPUT MAPPING
  // ============================================================
  
  // Camera/Sensor data from dedicated inputs
  assign cam_data = ui_in[7:0];
  
  // Control signals from bidirectional pins (configured as inputs)
  assign cam_href  = uio_in[5];
  assign cam_vsync = uio_in[6];
  
  // Use system clock as camera pixel clock (simplified)
  assign cam_pclk = clk;
  
  // UART RX from bidirectional (if needed for config)
  assign uart_rx = uio_in[0];  // Reuse sensor select as UART RX
  
  // ============================================================
  // OUTPUT MAPPING
  // ============================================================
  
  // Status LEDs on dedicated outputs
  assign uo_out[6:0] = status_leds[6:0];
  
  // Harvest alert on MSB
  assign uo_out[7] = harvest_alert;
  
  // Bidirectional pins configured as inputs (no outputs needed)
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;  // All bidirectional pins as inputs
  
  // ============================================================
  // MAIN PROCESSOR INSTANTIATION
  // ============================================================
  
  main_processor_top main_proc (
    .clk(clk),
    .rst_n(rst_n & ena),  // Combine reset with enable
    
    // Camera interface
    .cam_pclk(cam_pclk),
    .cam_href(cam_href),
    .cam_vsync(cam_vsync),
    .cam_data(cam_data),
    
    // UART interface (optional for TT)
    .uart_rx(uart_rx),
    .uart_tx(uart_tx),
    
    // Outputs
    .harvest_alert(harvest_alert),
    .alert_level(alert_level),
    .status_leds(status_leds),
    .fault_detected(fault_detected)
  );
  
  // ============================================================
  // UNUSED SIGNAL HANDLING
  // ============================================================
  // Prevent warnings for unused signals
  wire _unused = &{ena, alert_level, fault_detected, uart_tx, uio_in[4:2], uio_in[7], 1'b0};

endmodule
