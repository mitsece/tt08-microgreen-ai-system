// ============================================================
// UART TRANSMITTER MODULE
// ============================================================
// Baud rate: 115200
// Data bits: 8
// Stop bits: 1
// Parity: None
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

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
reg [8:0] clk_count;
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
