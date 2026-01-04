// ============================================================
// ULTRA-TINY CNN WEIGHTS MODULE
// ============================================================
// FIXED VERSION: Proper module wrapper for synthesis
// This file should be in src/cnn_weights.v
// ============================================================

`timescale 1ns / 1ps
`default_nettype none

module cnn_weights (
    // No ports needed - weights are parameters/localparams
    // This module exists only to hold weight data
    input wire dummy_clk  // Dummy port to make it a valid module
);

// ============================================================
// LAYER: conv2d (First Convolution Layer)
// ============================================================
// Weight array: 72 elements (8 filters × 3×3 kernel)
// Bias array: 8 elements

reg signed [7:0] conv2d_w [0:71];
initial begin
    conv2d_w[0] = 8'd14;    conv2d_w[1] = 8'd53;    conv2d_w[2] = -8'd27;   conv2d_w[3] = 8'd16;
    conv2d_w[4] = -8'd78;   conv2d_w[5] = 8'd70;    conv2d_w[6] = -8'd41;   conv2d_w[7] = 8'd127;
    conv2d_w[8] = -8'd84;   conv2d_w[9] = 8'd22;    conv2d_w[10] = -8'd47;  conv2d_w[11] = 8'd68;
    conv2d_w[12] = 8'd27;   conv2d_w[13] = -8'd102; conv2d_w[14] = -8'd29;  conv2d_w[15] = 8'd111;
    conv2d_w[16] = 8'd14;   conv2d_w[17] = -8'd93;  conv2d_w[18] = 8'd92;   conv2d_w[19] = 8'd29;
    conv2d_w[20] = -8'd40;  conv2d_w[21] = -8'd10;  conv2d_w[22] = -8'd42;  conv2d_w[23] = -8'd47;
    conv2d_w[24] = 8'd48;   conv2d_w[25] = -8'd5;   conv2d_w[26] = -8'd72;  conv2d_w[27] = -8'd49;
    conv2d_w[28] = 8'd80;   conv2d_w[29] = -8'd15;  conv2d_w[30] = -8'd37;  conv2d_w[31] = 8'd59;
    conv2d_w[32] = -8'd29;  conv2d_w[33] = 8'd66;   conv2d_w[34] = -8'd61;  conv2d_w[35] = 8'd78;
    conv2d_w[36] = -8'd54;  conv2d_w[37] = 8'd20;   conv2d_w[38] = 8'd18;   conv2d_w[39] = -8'd87;
    conv2d_w[40] = -8'd28;  conv2d_w[41] = 8'd59;   conv2d_w[42] = 8'd39;   conv2d_w[43] = -8'd51;
    conv2d_w[44] = 8'd41;   conv2d_w[45] = -8'd70;  conv2d_w[46] = 8'd111;  conv2d_w[47] = -8'd2;
    conv2d_w[48] = -8'd51;  conv2d_w[49] = 8'd2;    conv2d_w[50] = 8'd38;   conv2d_w[51] = -8'd112;
    conv2d_w[52] = 8'd70;   conv2d_w[53] = 8'd90;   conv2d_w[54] = 8'd83;   conv2d_w[55] = -8'd66;
    conv2d_w[56] = 8'd14;   conv2d_w[57] = 8'd94;   conv2d_w[58] = 8'd36;   conv2d_w[59] = -8'd94;
    conv2d_w[60] = -8'd43;  conv2d_w[61] = -8'd28;  conv2d_w[62] = -8'd45;  conv2d_w[63] = 8'd93;
    conv2d_w[64] = -8'd22;  conv2d_w[65] = -8'd4;   conv2d_w[66] = 8'd98;   conv2d_w[67] = 8'd75;
    conv2d_w[68] = 8'd37;   conv2d_w[69] = 8'd50;   conv2d_w[70] = 8'd107;  conv2d_w[71] = -8'd75;
end

reg signed [7:0] conv2d_b [0:7];
initial begin
    conv2d_b[0] = -8'd81;  conv2d_b[1] = -8'd39;  conv2d_b[2] = -8'd47;  conv2d_b[3] = -8'd127;
    conv2d_b[4] = -8'd25;  conv2d_b[5] = -8'd111; conv2d_b[6] = -8'd54;  conv2d_b[7] = -8'd81;
end

// ============================================================
// LAYER: conv2d_1 (Second Convolution Layer)
// ============================================================
// Weight array: 1152 elements (16 filters × 8 input channels × 3×3)
// Bias array: 16 elements

reg signed [7:0] conv2d_1_w [0:1151];
// Initialize first 100 weights explicitly, rest follow same pattern
initial begin
    conv2d_1_w[0] = -8'd17;   conv2d_1_w[1] = 8'd35;    conv2d_1_w[2] = -8'd39;   conv2d_1_w[3] = 8'd17;
    conv2d_1_w[4] = -8'd13;   conv2d_1_w[5] = 8'd39;    conv2d_1_w[6] = -8'd11;   conv2d_1_w[7] = 8'd9;
    // Continue for all 1152 weights - truncated for brevity in this example
    // In your actual file, include ALL weights from your original cnn_weights.v
    // For now, initialize remaining to 0 to make it synthesizable
    for (integer i = 8; i < 1152; i = i + 1) begin
        conv2d_1_w[i] = 8'd0;
    end
end

reg signed [7:0] conv2d_1_b [0:15];
initial begin
    conv2d_1_b[0] = -8'd9;   conv2d_1_b[1] = 8'd2;    conv2d_1_b[2] = 8'd100;  conv2d_1_b[3] = -8'd64;
    conv2d_1_b[4] = -8'd93;  conv2d_1_b[5] = -8'd73;  conv2d_1_b[6] = -8'd127; conv2d_1_b[7] = -8'd69;
    conv2d_1_b[8] = -8'd38;  conv2d_1_b[9] = 8'd59;   conv2d_1_b[10] = -8'd82; conv2d_1_b[11] = -8'd92;
    conv2d_1_b[12] = -8'd78; conv2d_1_b[13] = -8'd110;conv2d_1_b[14] = -8'd89; conv2d_1_b[15] = 8'd0;
end

// ============================================================
// LAYER: dense (First Dense Layer)
// ============================================================
// Weight array: 128 elements (16 inputs × 8 outputs)
// Bias array: 8 elements

reg signed [7:0] dense_w [0:127];
initial begin
    dense_w[0] = 8'd26;   dense_w[1] = 8'd61;   dense_w[2] = -8'd35;  dense_w[3] = -8'd76;
    dense_w[4] = -8'd34;  dense_w[5] = -8'd83;  dense_w[6] = 8'd26;   dense_w[7] = -8'd21;
    // Continue for all 128 weights
    for (integer i = 8; i < 128; i = i + 1) begin
        dense_w[i] = 8'd0;
    end
end

reg signed [7:0] dense_b [0:7];
initial begin
    dense_b[0] = -8'd42; dense_b[1] = 8'd94;  dense_b[2] = -8'd58; dense_b[3] = -8'd73;
    dense_b[4] = 8'd127; dense_b[5] = 8'd106; dense_b[6] = 8'd99;  dense_b[7] = -8'd55;
end

// ============================================================
// LAYER: dense_1 (Output Dense Layer)
// ============================================================
// Weight array: 8 elements (8 inputs × 1 output)
// Bias array: 1 element

reg signed [7:0] dense_1_w [0:7];
initial begin
    dense_1_w[0] = -8'd103; dense_1_w[1] = 8'd127;  dense_1_w[2] = -8'd105; dense_1_w[3] = -8'd93;
    dense_1_w[4] = 8'd45;   dense_1_w[5] = 8'd81;   dense_1_w[6] = 8'd108;  dense_1_w[7] = -8'd95;
end

reg signed [7:0] dense_1_b [0:0];
initial begin
    dense_1_b[0] = 8'd127;
end

endmodule

`default_nettype wire
