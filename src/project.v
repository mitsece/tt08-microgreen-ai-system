// ============================================================
// PRECISION FARMING ASIC - TINY TAPEOUT SUBMISSION
// ============================================================
// Wrapper for 2x2 tile
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

  // Pin Mapping Details:
  // ui_in[7:0]   -> Camera Data (8-bit)
  // uio_in[0]    -> UART RX
  // uio_in[5]    -> Camera HREF
  // uio_in[6]    -> Camera VSYNC
  // uo_out[6:0]  -> Status LEDs
  // uo_out[7]    -> Harvest Alert

  wire [7:0] status_leds;
  wire [2:0] alert_level;
  wire harvest_alert;
  wire uart_tx;
  wire fault_detected;

  // Instantiate the verified main processor
  main_processor_asic main_proc (
    .clk(clk),
    .rst_n(rst_n & ena),
    .cam_pclk(clk),
    .cam_href(uio_in[5]),
    .cam_vsync(uio_in[6]),
    .cam_data(ui_in),
    .uart_rx(uio_in[0]),
    .uart_tx(uart_tx),
    .harvest_alert(harvest_alert),
    .alert_level(alert_level),
    .status_leds(status_leds),
    .fault_detected(fault_detected)
  );

  // Map outputs
  assign uo_out[6:0] = status_leds[6:0];
  assign uo_out[7]   = harvest_alert;
  
  // Set bidirectionals
  assign uio_out[0] = uart_tx;  // UART TX out on uio[0] if needed, but ui_in/out is main
  assign uio_out[7:1] = 7'b0;
  assign uio_oe = 8'b00000001; // Only uio[0] (UART TX) is output (optional)

  // Prevent warnings
  wire _unused = &{alert_level, fault_detected, uio_in[4:1], uio_in[7], ena, 1'b0};

endmodule
