// ============================================================
// Ultra-Tiny CNN Weights for 2x2 ASIC
// ============================================================
// WEIGHT DECLARATIONS ONLY
// Initial values are set in cnn_inference.v after include
// This file contains ONLY the reg declarations
// ============================================================

// Layer: conv2d (8 filters, 3x3 kernel, 1 input channel)
reg signed [7:0] conv2d_w [0:71];
reg signed [7:0] conv2d_b [0:7];

// Layer: conv2d_1 (16 filters, 3x3 kernel, 8 input channels)
reg signed [7:0] conv2d_1_w [0:1151];
reg signed [7:0] conv2d_1_b [0:15];

// Layer: dense (16 to 8)
reg signed [7:0] dense_w [0:127];
reg signed [7:0] dense_b [0:7];

// Layer: dense_1 (8 to 1)
reg signed [7:0] dense_1_w [0:7];
reg signed [7:0] dense_1_b [0:0];
