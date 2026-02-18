// ============================================================
// UART RECEIVER MODULE
// ============================================================
// Baud rate: 115200
// Data bits: 8
// Stop bits: 1
// Parity: None
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

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
