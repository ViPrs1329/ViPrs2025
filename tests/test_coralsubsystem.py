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
    assert coral_subsystem.current_left_speed == 0.0
    assert coral_subsystem.current_right_speed == 0.0
    assert len(coral_subsystem.set_left_speed_calls) == 0
    assert len(coral_subsystem.set_right_speed_calls) == 0

def test_speed_control(coral_subsystem):
    """
    Tests speed control functions for both wheels.
    """
    # Test setting speeds
    coral_subsystem.setLeftSpeed(0.8)
    coral_subsystem.setRightSpeed(0.8)
    
    # Check that speeds were set
    assert coral_subsystem.current_left_speed == 0.8
    assert coral_subsystem.current_right_speed == 0.8
    assert coral_subsystem.set_left_speed_calls == [0.8]
    assert coral_subsystem.set_right_speed_calls == [0.8]
    
    # Test stopping
    coral_subsystem.stop()
    
    # Check that speeds were set to 0
    assert coral_subsystem.current_left_speed == 0.0
    assert coral_subsystem.current_right_speed == 0.0
    assert coral_subsystem.set_left_speed_calls == [0.8, 0.0]
    assert coral_subsystem.set_right_speed_calls == [0.8, 0.0]

def test_speed_limits(coral_subsystem):
    """
    Tests speed limit handling for both wheels.
    """
    # Test maximum speed (1.0)
    coral_subsystem.setLeftSpeed(1.0)
    coral_subsystem.setRightSpeed(1.0)
    assert coral_subsystem.current_left_speed == 1.0
    assert coral_subsystem.current_right_speed == 1.0
    
    # Test minimum speed (-1.0)
    coral_subsystem.setLeftSpeed(-1.0)
    coral_subsystem.setRightSpeed(-1.0)
    assert coral_subsystem.current_left_speed == -1.0
    assert coral_subsystem.current_right_speed == -1.0
    
    # Test speed clamping
    coral_subsystem.setLeftSpeed(1.5)  # Above max
    coral_subsystem.setRightSpeed(-1.5)  # Below min
    assert coral_subsystem.current_left_speed == 1.0
    assert coral_subsystem.current_right_speed == -1.0

def test_motor_current_monitoring(coral_subsystem):
    """
    Tests motor current monitoring functionality.
    """
    # Test setting motor currents
    coral_subsystem.set_motor_currents(left_current=30.0, right_current=25.0)
    
    # Check that currents were set
    assert coral_subsystem.left_motor_current == 30.0
    assert coral_subsystem.right_motor_current == 25.0

def test_stop_behavior(coral_subsystem):
    """
    Tests stop behavior in various states.
    """
    # Test stopping while moving
    coral_subsystem.setLeftSpeed(0.8)
    coral_subsystem.setRightSpeed(0.8)
    coral_subsystem.stop()
    
    assert coral_subsystem.current_left_speed == 0.0
    assert coral_subsystem.current_right_speed == 0.0
    
    # Test stopping while already stopped
    coral_subsystem.stop()
    assert coral_subsystem.current_left_speed == 0.0
    assert coral_subsystem.current_right_speed == 0.0

def test_opposite_direction_control(coral_subsystem):
    """
    Tests controlling wheels in opposite directions.
    """
    coral_subsystem.setLeftSpeed(0.8)   # Forward
    coral_subsystem.setRightSpeed(-0.8)  # Backward
    
    assert coral_subsystem.current_left_speed == 0.8
    assert coral_subsystem.current_right_speed == -0.8

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