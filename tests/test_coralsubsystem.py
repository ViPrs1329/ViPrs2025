"""
Tests for the CORAL subsystem using the test-specific implementation.
"""
import pytest
from test_coral_implementation import TestCoralSubsystem

@pytest.fixture
def coral_subsystem():
    """
    Creates a test CORAL subsystem for testing.
    """
    return TestCoralSubsystem()

def test_initialization(coral_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert coral_subsystem is not None
    assert coral_subsystem.current_speed == 0.0
    assert len(coral_subsystem.set_speed_calls) == 0

def test_speed_control(coral_subsystem):
    """
    Tests speed control functions.
    """
    # Test setting speed
    coral_subsystem.setSpeed(0.5)
    
    # Check that speed was set
    assert coral_subsystem.current_speed == 0.5
    assert coral_subsystem.set_speed_calls == [0.5]
    
    # Test stopping
    coral_subsystem.stop()
    
    # Check that speed was set to 0
    assert coral_subsystem.current_speed == 0.0
    assert coral_subsystem.set_speed_calls == [0.5, 0.0]

def test_current_monitoring(coral_subsystem):
    """
    Tests current monitoring functions.
    """
    # Set different current values for the two motors
    coral_subsystem.set_motor_currents(10.0, 20.0)
    
    # Test getting average current
    assert coral_subsystem.getCurrent() == 15.0

def test_stall_detection(coral_subsystem):
    """
    Tests stall detection based on current draw.
    """
    # Test not stalled
    coral_subsystem.set_motor_currents(15.0, 15.0)
    assert coral_subsystem.isStalled() is False
    
    # Test stalled
    coral_subsystem.set_motor_currents(35.0, 35.0)
    assert coral_subsystem.isStalled() is True 