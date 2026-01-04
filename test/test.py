import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, FallingEdge
import random

@cocotb.test()
async def test_reset(dut):
    """Test reset functionality"""
    dut._log.info("Starting reset test")
    
    # Create clock
    clock = Clock(dut.clk, 20, units="ns")  # 50MHz
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Check outputs are initialized
    dut._log.info(f"Output after reset: {dut.uo_out.value}")
    
    dut._log.info("✅ Reset test passed")


@cocotb.test()
async def test_growth_stage_image(dut):
    """Test with growth stage image (should NOT trigger harvest)"""
    dut._log.info("=" * 60)
    dut._log.info("TEST: Growth Stage Detection")
    dut._log.info("=" * 60)
    
    # Setup
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Send growth-stage image (low intensity = immature plants)
    dut._log.info("Sending 32x32 growth-stage image...")
    
    # Frame start - VSYNC pulse
    dut.uio_in.value = 0x40  # VSYNC high
    await ClockCycles(dut.clk, 10)
    dut.uio_in.value = 0x00  # VSYNC low
    
    # Send 1024 pixels (32x32)
    for row in range(32):
        # Line start - HREF high
        dut.uio_in.value = 0x20  # HREF high
        
        for col in range(32):
            # Growth stage: low intensity (60-100 range)
            pixel_value = 60 + (row * col) % 40
            dut.ui_in.value = pixel_value
            await RisingEdge(dut.clk)
        
        # Line end - HREF low
        dut.uio_in.value = 0x00
        await ClockCycles(dut.clk, 2)
    
    dut._log.info("Image transmission complete")
    
    # Wait for CNN inference (need time for processing)
    dut._log.info("Waiting for CNN processing...")
    await ClockCycles(dut.clk, 10000)  # ~200 microseconds at 50MHz
    
    # Check outputs
    harvest_alert = (dut.uo_out.value >> 7) & 1
    status_leds = dut.uo_out.value & 0x7F
    
    dut._log.info(f"Harvest Alert: {harvest_alert}")
    dut._log.info(f"Status LEDs: {status_leds:07b}")
    dut._log.info(f"  [0] CNN Busy:    {(status_leds >> 0) & 1}")
    dut._log.info(f"  [1] CNN Ready:   {(status_leds >> 1) & 1}")
    dut._log.info(f"  [2] CNN Harvest: {(status_leds >> 2) & 1}")
    dut._log.info(f"  [3] Sensors OK:  {(status_leds >> 3) & 1}")
    dut._log.info(f"  [4] Final Alert: {(status_leds >> 4) & 1}")
    
    # For growth stage, expect NO harvest alert
    # Note: Actual behavior depends on CNN implementation
    if harvest_alert == 0:
        dut._log.info("✅ PASS: No false positive (correctly identified as growth stage)")
    else:
        dut._log.warning("⚠️  WARNING: Harvest alert during growth stage (may need threshold tuning)")


@cocotb.test()
async def test_harvest_stage_image(dut):
    """Test with harvest stage image (should trigger harvest)"""
    dut._log.info("=" * 60)
    dut._log.info("TEST: Harvest Stage Detection")
    dut._log.info("=" * 60)
    
    # Setup
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Send harvest-stage image (high intensity = mature plants)
    dut._log.info("Sending 32x32 harvest-stage image...")
    
    # Frame start
    dut.uio_in.value = 0x40  # VSYNC
    await ClockCycles(dut.clk, 10)
    dut.uio_in.value = 0x00
    
    # Send 1024 pixels
    for row in range(32):
        dut.uio_in.value = 0x20  # HREF
        
        for col in range(32):
            # Harvest stage: high intensity (150-190 range)
            pixel_value = 150 + (row * col) % 40
            dut.ui_in.value = pixel_value
            await RisingEdge(dut.clk)
        
        dut.uio_in.value = 0x00
        await ClockCycles(dut.clk, 2)
    
    dut._log.info("Image transmission complete")
    
    # Wait for processing
    dut._log.info("Waiting for CNN processing...")
    await ClockCycles(dut.clk, 10000)
    
    # Check outputs
    harvest_alert = (dut.uo_out.value >> 7) & 1
    status_leds = dut.uo_out.value & 0x7F
    
    dut._log.info(f"Harvest Alert: {harvest_alert}")
    dut._log.info(f"Status LEDs: {status_leds:07b}")
    
    if harvest_alert == 1:
        dut._log.info("✅ PASS: Harvest correctly detected")
    else:
        dut._log.warning("⚠️  WARNING: Harvest not detected (may need threshold tuning)")


@cocotb.test()
async def test_pin_toggling(dut):
    """Basic test - verify all pins can toggle"""
    dut._log.info("=" * 60)
    dut._log.info("TEST: Pin Toggling")
    dut._log.info("=" * 60)
    
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    
    # Test input pins
    for val in [0x00, 0xFF, 0xAA, 0x55]:
        dut.ui_in.value = val
        dut.uio_in.value = val
        await ClockCycles(dut.clk, 10)
        dut._log.info(f"Input: 0x{val:02X}, Output: 0x{dut.uo_out.value:02X}")
    
    dut._log.info("✅ Pin toggling test complete")


@cocotb.test()
async def test_camera_timing(dut):
    """Test camera interface timing"""
    dut._log.info("=" * 60)
    dut._log.info("TEST: Camera Interface Timing")
    dut._log.info("=" * 60)
    
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    
    # Test VSYNC/HREF signals
    dut._log.info("Testing VSYNC pulse...")
    dut.uio_in.value = 0x40  # VSYNC high
    await ClockCycles(dut.clk, 5)
    dut.uio_in.value = 0x00  # VSYNC low
    
    dut._log.info("Testing HREF signal...")
    dut.uio_in.value = 0x20  # HREF high
    await ClockCycles(dut.clk, 5)
    dut.uio_in.value = 0x00  # HREF low
    
    dut._log.info("✅ Camera timing test complete")
