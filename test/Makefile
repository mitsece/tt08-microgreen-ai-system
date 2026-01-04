# test/Makefile
# Cocotb testbench configuration

# Simulator (icarus for open-source, or verilator)
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Source directory
SRC_DIR = $(PWD)/../src

# Your Verilog source files
PROJECT_SOURCES = project.v \
                  main_processor_top.v \
                  cnn_inference.v \
                  camera_interface.v \
                  uart_rx.v \
                  uart_tx.v \
                  asic_weights_tiny/cnn_weights.v

# Determine if RTL or gate-level simulation
ifneq ($(GATES),yes)
    # RTL simulation (default)
    SIM_BUILD = sim_build/rtl
    VERILOG_SOURCES = $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
    COMPILE_ARGS += -I$(SRC_DIR)
else
    # Gate-level simulation (after GDS generation)
    SIM_BUILD = sim_build/gl
    COMPILE_ARGS += -DGL_TEST
    COMPILE_ARGS += -DFUNCTIONAL
    COMPILE_ARGS += -DUSE_POWER_PINS
    COMPILE_ARGS += -DSIM
    COMPILE_ARGS += -DUNIT_DELAY=\#1
    VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
    VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v
    VERILOG_SOURCES += $(PWD)/gate_level_netlist.v
endif

# Testbench wrapper
VERILOG_SOURCES += $(PWD)/tb.v

# Top-level module (from tb.v)
TOPLEVEL = tb

# Python test module (test.py)
MODULE = test

# Include cocotb's make rules
include $(shell cocotb-config --makefiles)/Makefile.sim
