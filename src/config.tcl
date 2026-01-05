# OpenLane Configuration for Precision Farming ASIC
# This file provides the VERILOG_FILES variable that config.json is missing

# Design name
set ::env(DESIGN_NAME) "tt_um_precision_farming"

# Verilog source files
# NOTE: cnn_weights.v is NOT listed here because it's included via `include directive in cnn_inference.v
set ::env(VERILOG_FILES) "\
    $::env(DESIGN_DIR)/project.v \
    $::env(DESIGN_DIR)/main_processor_top.v \
    $::env(DESIGN_DIR)/cnn_inference.v \
    $::env(DESIGN_DIR)/camera_interface.v \
    $::env(DESIGN_DIR)/uart_rx.v \
    $::env(DESIGN_DIR)/uart_tx.v"

# Clock configuration
set ::env(CLOCK_PERIOD) "20"
set ::env(CLOCK_PORT) "clk"

# Die area for 2x2 tile (320um x 500um)
set ::env(DIE_AREA) "0 0 320 500"
set ::env(FP_SIZING) absolute

# Placement density (can increase if needed)
set ::env(PL_TARGET_DENSITY) 0.55

# Allow routing congestion for large designs
set ::env(GRT_ALLOW_CONGESTION) 1

# Don't buffer clock on outputs
set ::env(PL_RESIZER_BUFFER_OUTPUT_PORTS) 0

# Synthesis strategy
set ::env(SYNTH_STRATEGY) "DELAY 0"
set ::env(SYNTH_MAX_FANOUT) 6

# Routing
set ::env(RT_MAX_LAYER) "met4"

# For designs with many registers
set ::env(FP_PDN_MULTILAYER) 0
