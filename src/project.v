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
  wire uart_tx_sig;
  wire fault_detected;

  main_processor_asic main_proc (
    .clk         (clk),
    .rst_n       (rst_n & ena),
    .cam_href    (uio_in[5]),
    .cam_vsync   (uio_in[6]),
    .cam_data    (ui_in),
    .uart_rx     (uio_in[0]),
    .uart_tx     (uart_tx_sig),
    .harvest_alert  (harvest_alert),
    .alert_level    (alert_level),
    .status_leds    (status_leds),
    .fault_detected (fault_detected)
  );

  assign uo_out[6:0]  = status_leds[6:0];
  assign uo_out[7]    = harvest_alert;
  assign uio_out[0]   = uart_tx_sig;
  assign uio_out[7:1] = 7'b0;
  assign uio_oe       = 8'b00000001;

  wire _unused = &{alert_level, fault_detected, uio_in[4:1], uio_in[7], 1'b0};
endmodule

// ============================================================
// MAIN PROCESSOR ASIC - TOP LEVEL
// ============================================================

module main_processor_asic (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       cam_href,
    input  wire       cam_vsync,
    input  wire [7:0] cam_data,
    input  wire       uart_rx,
    output wire       uart_tx,
    output wire       harvest_alert,
    output wire [2:0] alert_level,
    output wire [6:0] status_leds,
    output wire       fault_detected
);

  wire [7:0] cnn_pixel;
  wire       cnn_pixel_valid;
  wire       cnn_frame_start;
  wire       cnn_classification;
  wire [7:0] cnn_confidence;
  wire       cnn_ready;
  wire       cnn_busy;

  wire [7:0] uart_rx_data;
  wire       uart_rx_valid;
  wire [7:0] uart_tx_data;
  wire       uart_tx_valid;
  wire       unused_tx_ready;
  wire       unused_frame_done;

  reg [1:0] sensor_temp;
  reg [1:0] sensor_humidity;
  reg [1:0] sensor_light;
  reg [1:0] sensor_soil;
  reg [7:0] fault_flags;

  reg harvest_ready_cnn;
  reg harvest_ready_sensors;
  reg harvest_ready_final;
  reg [2:0] alert_level_reg;
  reg [6:0] status_leds_reg;
  reg [2:0] rx_byte_count;
  reg [7:0] rx_checksum;

  camera_interface cam_if (
    .clk        (clk),
    .rst_n      (rst_n),
    .cam_href   (cam_href),
    .cam_vsync  (cam_vsync),
    .cam_data   (cam_data),
    .pixel_out  (cnn_pixel),
    .pixel_valid(cnn_pixel_valid),
    .frame_start(cnn_frame_start),
    .frame_done (unused_frame_done)
  );

  cnn_inference cnn (
    .clk            (clk),
    .rst_n          (rst_n),
    .pixel_in       (cnn_pixel),
    .pixel_valid    (cnn_pixel_valid),
    .frame_start    (cnn_frame_start),
    .classification (cnn_classification),
    .confidence     (cnn_confidence),
    .ready          (cnn_ready),
    .busy           (cnn_busy)
  );

  uart_rx #(.CLKS_PER_BIT(434)) uart_rx_inst (
    .clk      (clk),
    .rst_n    (rst_n),
    .rx       (uart_rx),
    .rx_data  (uart_rx_data),
    .rx_valid (uart_rx_valid)
  );

  uart_tx #(.CLKS_PER_BIT(434)) uart_tx_inst (
    .clk      (clk),
    .rst_n    (rst_n),
    .tx_data  (uart_tx_data),
    .tx_valid (uart_tx_valid),
    .tx       (uart_tx),
    .tx_ready (unused_tx_ready)
  );

  // UART Packet Parser
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_byte_count   <= 3'd0;
      rx_checksum     <= 8'd0;
      sensor_temp     <= 2'd0;
      sensor_humidity <= 2'd0;
      sensor_light    <= 2'd0;
      sensor_soil     <= 2'd0;
      fault_flags     <= 8'd0;
    end else if (uart_rx_valid) begin
      case (rx_byte_count)
        3'd0: begin
          if (uart_rx_data == 8'hAA) begin
            rx_byte_count <= 3'd1;
            rx_checksum   <= 8'd0;
          end
        end
        3'd1: begin sensor_temp     <= uart_rx_data[1:0]; rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd2; end
        3'd2: begin sensor_humidity <= uart_rx_data[1:0]; rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd3; end
        3'd3: begin sensor_light    <= uart_rx_data[1:0]; rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd4; end
        3'd4: begin sensor_soil     <= uart_rx_data[1:0]; rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd5; end
        3'd5: begin fault_flags     <= uart_rx_data;      rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd6; end
        3'd6: begin                                        rx_checksum <= rx_checksum + uart_rx_data; rx_byte_count <= 3'd7; end
        3'd7: begin rx_byte_count <= 3'd0; end
        default: rx_byte_count <= 3'd0;
      endcase
    end
  end

  // Decision Fusion
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      harvest_ready_cnn     <= 1'b0;
      harvest_ready_sensors <= 1'b0;
      harvest_ready_final   <= 1'b0;
    end else begin
      if (cnn_ready)
        harvest_ready_cnn <= cnn_classification && (cnn_confidence > 8'd40);
      harvest_ready_sensors <= (sensor_temp == 2'd2) && (sensor_humidity == 2'd2) &&
                               (sensor_light == 2'd2) && (sensor_soil == 2'd2);
      harvest_ready_final   <= harvest_ready_cnn && harvest_ready_sensors;
    end
  end

  // Alert Level
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      alert_level_reg <= 3'd0;
    end else begin
      if      (|fault_flags)         alert_level_reg <= 3'd7;
      else if (harvest_ready_final)  alert_level_reg <= 3'd6;
      else if (harvest_ready_cnn)    alert_level_reg <= 3'd4;
      else if (!harvest_ready_sensors) alert_level_reg <= 3'd2;
      else                           alert_level_reg <= 3'd0;
    end
  end

  // Status LEDs
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      status_leds_reg <= 7'd0;
    end else begin
      status_leds_reg[0] <= cnn_busy;
      status_leds_reg[1] <= cnn_ready;
      status_leds_reg[2] <= harvest_ready_cnn;
      status_leds_reg[3] <= harvest_ready_sensors;
      status_leds_reg[4] <= harvest_ready_final;
      status_leds_reg[5] <= |fault_flags;
      status_leds_reg[6] <= uart_rx_valid;
    end
  end

  assign harvest_alert  = harvest_ready_final;
  assign alert_level    = alert_level_reg;
  assign status_leds    = status_leds_reg;
  assign fault_detected = |fault_flags;
  assign uart_tx_data   = 8'h55;
  assign uart_tx_valid  = uart_rx_valid;

  wire _unused_main = &{rx_checksum, cnn_confidence, unused_tx_ready, unused_frame_done, 1'b0};

endmodule

// ============================================================
// CNN INFERENCE ENGINE
// ============================================================

module cnn_inference (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] pixel_in,
    input  wire       pixel_valid,
    input  wire       frame_start,
    output reg        classification,
    output reg  [7:0] confidence,
    output reg        ready,
    output reg        busy
);

  reg signed [7:0] conv2d_w [0:63];
  initial begin
    conv2d_w[ 0]=-8'd17; conv2d_w[ 1]= 8'd35; conv2d_w[ 2]=-8'd39; conv2d_w[ 3]= 8'd17;
    conv2d_w[ 4]=-8'd13; conv2d_w[ 5]= 8'd39; conv2d_w[ 6]=-8'd11; conv2d_w[ 7]= 8'd9;
    conv2d_w[ 8]= 8'd1;  conv2d_w[ 9]= 8'd9;  conv2d_w[10]= 8'd18; conv2d_w[11]= 8'd2;
    conv2d_w[12]= 8'd54; conv2d_w[13]=-8'd35; conv2d_w[14]= 8'd8;  conv2d_w[15]=-8'd86;
    conv2d_w[16]= 8'd10; conv2d_w[17]= 8'd4;  conv2d_w[18]=-8'd6;  conv2d_w[19]= 8'd22;
    conv2d_w[20]= 8'd0;  conv2d_w[21]= 8'd7;  conv2d_w[22]= 8'd3;  conv2d_w[23]= 8'd5;
    conv2d_w[24]= 8'd14; conv2d_w[25]= 8'd53; conv2d_w[26]=-8'd27; conv2d_w[27]= 8'd16;
    conv2d_w[28]=-8'd78; conv2d_w[29]= 8'd70; conv2d_w[30]=-8'd41; conv2d_w[31]= 8'd127;
    conv2d_w[32]=-8'd84; conv2d_w[33]= 8'd22; conv2d_w[34]=-8'd47; conv2d_w[35]= 8'd68;
    conv2d_w[36]= 8'd27; conv2d_w[37]=-8'd102;conv2d_w[38]=-8'd29; conv2d_w[39]= 8'd111;
    conv2d_w[40]= 8'd14; conv2d_w[41]=-8'd93; conv2d_w[42]= 8'd92; conv2d_w[43]= 8'd29;
    conv2d_w[44]=-8'd40; conv2d_w[45]=-8'd10; conv2d_w[46]=-8'd42; conv2d_w[47]=-8'd47;
    conv2d_w[48]= 8'd48; conv2d_w[49]=-8'd5;  conv2d_w[50]=-8'd72; conv2d_w[51]=-8'd49;
    conv2d_w[52]= 8'd80; conv2d_w[53]=-8'd15; conv2d_w[54]=-8'd37; conv2d_w[55]= 8'd59;
    conv2d_w[56]=-8'd29; conv2d_w[57]= 8'd66; conv2d_w[58]=-8'd61; conv2d_w[59]= 8'd78;
    conv2d_w[60]=-8'd54; conv2d_w[61]= 8'd20; conv2d_w[62]= 8'd18; conv2d_w[63]=-8'd87;
  end

  reg signed [7:0] conv2d_b [0:7];
  initial begin
    conv2d_b[0]=-8'd81;  conv2d_b[1]=-8'd39;  conv2d_b[2]=-8'd47;  conv2d_b[3]=-8'd127;
    conv2d_b[4]=-8'd25;  conv2d_b[5]=-8'd111; conv2d_b[6]=-8'd54;  conv2d_b[7]=-8'd81;
  end

  reg signed [23:0] acc0, acc1, acc2, acc3, acc4, acc5, acc6, acc7;
  reg [10:0] pix_cnt;
  reg [3:0]  w_phase;
  reg        frame_done_flag;

  // Combinatorial majority vote - avoids local reg declaration in always block
  wire [3:0] votes = (acc0[23] ? 4'd0 : 4'd1)
                   + (acc1[23] ? 4'd0 : 4'd1)
                   + (acc2[23] ? 4'd0 : 4'd1)
                   + (acc3[23] ? 4'd0 : 4'd1)
                   + (acc4[23] ? 4'd0 : 4'd1)
                   + (acc5[23] ? 4'd0 : 4'd1)
                   + (acc6[23] ? 4'd0 : 4'd1)
                   + (acc7[23] ? 4'd0 : 4'd1);

  wire [5:0] widx0 = {3'd0, w_phase[2:0]};
  wire [5:0] widx1 = {3'd1, w_phase[2:0]};
  wire [5:0] widx2 = {3'd2, w_phase[2:0]};
  wire [5:0] widx3 = {3'd3, w_phase[2:0]};
  wire [5:0] widx4 = {3'd4, w_phase[2:0]};
  wire [5:0] widx5 = {3'd5, w_phase[2:0]};
  wire [5:0] widx6 = {3'd6, w_phase[2:0]};
  wire [5:0] widx7 = {3'd7, w_phase[2:0]};

  wire signed [8:0] px_c = $signed({1'b0, pixel_in}) - 9'sd128;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      classification  <= 1'b0;
      confidence      <= 8'd0;
      ready           <= 1'b0;
      busy            <= 1'b0;
      pix_cnt         <= 11'd0;
      w_phase         <= 4'd0;
      frame_done_flag <= 1'b0;
      acc0 <= 24'd0; acc1 <= 24'd0; acc2 <= 24'd0; acc3 <= 24'd0;
      acc4 <= 24'd0; acc5 <= 24'd0; acc6 <= 24'd0; acc7 <= 24'd0;
    end else begin
      if (frame_start) begin
        pix_cnt         <= 11'd0;
        w_phase         <= 4'd0;
        ready           <= 1'b0;
        busy            <= 1'b1;
        frame_done_flag <= 1'b0;
        acc0 <= {{16{conv2d_b[0][7]}}, conv2d_b[0]};
        acc1 <= {{16{conv2d_b[1][7]}}, conv2d_b[1]};
        acc2 <= {{16{conv2d_b[2][7]}}, conv2d_b[2]};
        acc3 <= {{16{conv2d_b[3][7]}}, conv2d_b[3]};
        acc4 <= {{16{conv2d_b[4][7]}}, conv2d_b[4]};
        acc5 <= {{16{conv2d_b[5][7]}}, conv2d_b[5]};
        acc6 <= {{16{conv2d_b[6][7]}}, conv2d_b[6]};
        acc7 <= {{16{conv2d_b[7][7]}}, conv2d_b[7]};
      end else if (pixel_valid && !frame_done_flag) begin
        acc0    <= acc0 + px_c * conv2d_w[widx0];
        acc1    <= acc1 + px_c * conv2d_w[widx1];
        acc2    <= acc2 + px_c * conv2d_w[widx2];
        acc3    <= acc3 + px_c * conv2d_w[widx3];
        acc4    <= acc4 + px_c * conv2d_w[widx4];
        acc5    <= acc5 + px_c * conv2d_w[widx5];
        acc6    <= acc6 + px_c * conv2d_w[widx6];
        acc7    <= acc7 + px_c * conv2d_w[widx7];
        w_phase <= (w_phase == 4'd7) ? 4'd0 : w_phase + 4'd1;
        pix_cnt <= pix_cnt + 11'd1;
      end else if (pix_cnt >= 11'd512 && !frame_done_flag && !frame_start) begin
        frame_done_flag <= 1'b1;
        busy            <= 1'b0;
        ready           <= 1'b1;
        classification  <= (votes >= 4'd4) ? 1'b1 : 1'b0;
        confidence      <= {votes, 4'b0000};
      end
    end
  end

endmodule

// ============================================================
// CAMERA INTERFACE MODULE
// ============================================================

module camera_interface (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       cam_vsync,
    input  wire       cam_href,
    input  wire [7:0] cam_data,
    output reg  [7:0] pixel_out,
    output reg        pixel_valid,
    output reg        frame_start,
    output reg        frame_done
);

  reg vsync_d;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pixel_out   <= 8'd0;
      pixel_valid <= 1'b0;
      frame_start <= 1'b0;
      frame_done  <= 1'b0;
      vsync_d     <= 1'b0;
    end else begin
      vsync_d     <= cam_vsync;
      frame_start <= cam_vsync  && !vsync_d;
      frame_done  <= !cam_vsync && vsync_d;
      if (cam_href) begin
        pixel_out   <= cam_data;
        pixel_valid <= 1'b1;
      end else begin
        pixel_valid <= 1'b0;
      end
    end
  end

endmodule

// ============================================================
// UART RECEIVER MODULE
// ============================================================

module uart_rx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

  localparam IDLE  = 2'd0;
  localparam START = 2'd1;
  localparam DATA  = 2'd2;
  localparam STOP  = 2'd3;

  reg [1:0]  state;
  reg [11:0] clk_count;
  reg [2:0]  bit_index;
  reg [7:0]  rx_byte;
  reg        rx_sync1, rx_sync2;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_sync1 <= 1'b1;
      rx_sync2 <= 1'b1;
    end else begin
      rx_sync1 <= rx;
      rx_sync2 <= rx_sync1;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= IDLE;
      clk_count <= 12'd0;
      bit_index <= 3'd0;
      rx_data   <= 8'd0;
      rx_valid  <= 1'b0;
      rx_byte   <= 8'd0;
    end else begin
      rx_valid <= 1'b0;

      case (state)
        IDLE: begin
          clk_count <= 12'd0;
          bit_index <= 3'd0;
          if (rx_sync2 == 1'b0) state <= START;
        end
        START: begin
          if (clk_count == (CLKS_PER_BIT - 1) / 2) begin
            if (rx_sync2 == 1'b0) begin clk_count <= 12'd0; state <= DATA; end
            else state <= IDLE;
          end else clk_count <= clk_count + 12'd1;
        end
        DATA: begin
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count          <= 12'd0;
            rx_byte[bit_index] <= rx_sync2;
            if (bit_index == 3'd7) begin bit_index <= 3'd0; state <= STOP; end
            else bit_index <= bit_index + 3'd1;
          end else clk_count <= clk_count + 12'd1;
        end
        STOP: begin
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 12'd0;
            if (rx_sync2 == 1'b1) begin rx_data <= rx_byte; rx_valid <= 1'b1; end
            state <= IDLE;
          end else clk_count <= clk_count + 12'd1;
        end
        default: state <= IDLE;
      endcase
    end
  end

endmodule

// ============================================================
// UART TRANSMITTER MODULE
// ============================================================

module uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output reg        tx,
    output wire       tx_ready
);

  localparam IDLE  = 2'd0;
  localparam START = 2'd1;
  localparam DATA  = 2'd2;
  localparam STOP  = 2'd3;

  reg [1:0]  state;
  reg [11:0] clk_count;
  reg [2:0]  bit_index;
  reg [7:0]  tx_byte;

  assign tx_ready = (state == IDLE);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= IDLE;
      clk_count <= 12'd0;
      bit_index <= 3'd0;
      tx        <= 1'b1;
      tx_byte   <= 8'd0;
    end else begin
      case (state)
        IDLE: begin
          tx <= 1'b1; clk_count <= 12'd0; bit_index <= 3'd0;
          if (tx_valid) begin tx_byte <= tx_data; state <= START; end
        end
        START: begin
          tx <= 1'b0;
          if (clk_count == CLKS_PER_BIT - 1) begin clk_count <= 12'd0; state <= DATA; end
          else clk_count <= clk_count + 12'd1;
        end
        DATA: begin
          tx <= tx_byte[bit_index];
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 12'd0;
            if (bit_index == 3'd7) begin bit_index <= 3'd0; state <= STOP; end
            else bit_index <= bit_index + 3'd1;
          end else clk_count <= clk_count + 12'd1;
        end
        STOP: begin
          tx <= 1'b1;
          if (clk_count == CLKS_PER_BIT - 1) begin clk_count <= 12'd0; state <= IDLE; end
          else clk_count <= clk_count + 12'd1;
        end
        default: state <= IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
