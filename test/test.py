# Simple cocotb test for Tiny Tapeout workflow
# This is a minimal test to pass GDS build validation

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

@cocotb.test()
async def test_basic(dut):
    """Test basic functionality"""
    dut._log.info("Starting basic test")
    
    # Create clock
    clock = Clock(dut.clk, 10, units="ns")  # 100MHz
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    
    # Run for a few cycles
    await ClockCycles(dut.clk, 100)
    
    dut._log.info("Test completed successfully")
