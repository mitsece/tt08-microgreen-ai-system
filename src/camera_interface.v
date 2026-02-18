// ============================================================
// CAMERA INTERFACE MODULE
// ============================================================
// Captures 8-bit grayscale data from camera
// Generates internal frame/line valid signals
// ============================================================

`default_nettype none

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

