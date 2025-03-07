"""
Tests for the Elevator subsystem using the test-specific implementation.
"""
import pytest
from test_elevator_implementation import TestElevatorSubsystem
from robot.constants import (
    ELEVATOR_MIN_HEIGHT,
    ELEVATOR_L1_HEIGHT,
    ELEVATOR_L2_HEIGHT,
    ELEVATOR_L3_HEIGHT,
    ELEVATOR_L4_HEIGHT,
    ELEVATOR_MAX_HEIGHT
)

@pytest.fixture
def elevator_subsystem():
    """
    Creates a test Elevator subsystem for testing.
    """
    return TestElevatorSubsystem()

def test_initialization(elevator_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert elevator_subsystem is not None
    assert elevator_subsystem.current_position == 0.0
    assert elevator_subsystem.current_speed == 0.0
    assert len(elevator_subsystem.set_position_calls) == 0
    assert len(elevator_subsystem.set_speed_calls) == 0

def test_position_control(elevator_subsystem):
    """
    Tests position control functions.
    """
    # Test setting position
    elevator_subsystem.setPosition(ELEVATOR_L1_HEIGHT)
    
    # Check that position was set
    assert elevator_subsystem.current_position == ELEVATOR_L1_HEIGHT
    assert elevator_subsystem.set_position_calls == [ELEVATOR_L1_HEIGHT]
    
    # Test getting position
    assert elevator_subsystem.getPosition() == ELEVATOR_L1_HEIGHT
    
    # Test stopping
    elevator_subsystem.stop()
    
    # Check that position was maintained
    assert elevator_subsystem.current_position == ELEVATOR_L1_HEIGHT

def test_speed_control(elevator_subsystem):
    """
    Tests speed control functions.
    """
    # Test setting speed
    elevator_subsystem.setSpeed(0.5)
    
    # Check that speed was set
    assert elevator_subsystem.current_speed == 0.5
    assert elevator_subsystem.set_speed_calls == [0.5]
    
    # Test stopping
    elevator_subsystem.stop()
    
    # Check that speed was set to 0
    assert elevator_subsystem.current_speed == 0.0
    assert elevator_subsystem.set_speed_calls == [0.5, 0.0]

def test_speed_limits(elevator_subsystem):
    """
    Tests speed limit handling.
    """
    # Test maximum speed (1.0)
    elevator_subsystem.setSpeed(1.0)
    assert elevator_subsystem.current_speed == 1.0
    
    # Test minimum speed (-1.0)
    elevator_subsystem.setSpeed(-1.0)
    assert elevator_subsystem.current_speed == -1.0
    
    # Test speed clamping
    elevator_subsystem.setSpeed(1.5)  # Above max
    assert elevator_subsystem.current_speed == 1.0
    
    elevator_subsystem.setSpeed(-1.5)  # Below min
    assert elevator_subsystem.current_speed == -1.0

def test_motor_current_monitoring(elevator_subsystem):
    """
    Tests motor current monitoring functionality.
    """
    # Test setting motor currents
    elevator_subsystem.set_motor_currents(current=30.0)
    
    # Check that current was set
    assert elevator_subsystem.motor_current == 30.0

def test_stop_behavior(elevator_subsystem):
    """
    Tests stop behavior in various states.
    """
    # Test stopping while moving
    elevator_subsystem.setPosition(ELEVATOR_L1_HEIGHT)
    elevator_subsystem.setSpeed(0.5)
    elevator_subsystem.stop()
    
    assert elevator_subsystem.current_speed == 0.0
    assert elevator_subsystem.current_position == ELEVATOR_L1_HEIGHT  # Position should be maintained
    
    # Test stopping while already stopped
    elevator_subsystem.stop()
    assert elevator_subsystem.current_speed == 0.0
    assert elevator_subsystem.current_position == ELEVATOR_L1_HEIGHT

def test_height_presets(elevator_subsystem):
    """
    Tests all height preset positions.
    """
    presets = {
        "BASE": ELEVATOR_MIN_HEIGHT,
        "L1": ELEVATOR_L1_HEIGHT,
        "L2": ELEVATOR_L2_HEIGHT,
        "L3": ELEVATOR_L3_HEIGHT,
        "L4": ELEVATOR_L4_HEIGHT
    }
    
    for name, height in presets.items():
        elevator_subsystem.setPosition(height)
        assert elevator_subsystem.current_position == height
        assert elevator_subsystem.getPosition() == height

def test_setpoint_reached(elevator_subsystem):
    """
    Tests setpoint reached detection.
    """
    # Test with elevator not at setpoint
    elevator_subsystem.setPosition(ELEVATOR_L1_HEIGHT)
    elevator_subsystem.current_position = ELEVATOR_L1_HEIGHT + 5.0
    assert elevator_subsystem.atSetpoint() is False
    
    # Test with elevator at setpoint
    elevator_subsystem.current_position = ELEVATOR_L1_HEIGHT
    assert elevator_subsystem.atSetpoint() is True

def test_height_limits(elevator_subsystem):
    """
    Tests height limit handling.
    """
    # Enable height limit enforcement
    elevator_subsystem.set_enforce_height_limits(True)
    
    # Test minimum height
    elevator_subsystem.setPosition(ELEVATOR_MIN_HEIGHT)
    assert elevator_subsystem.current_position == ELEVATOR_MIN_HEIGHT

    # Test maximum height
    elevator_subsystem.setPosition(ELEVATOR_MAX_HEIGHT)
    assert elevator_subsystem.current_position == ELEVATOR_MAX_HEIGHT

    # Test below minimum (should be clamped)
    elevator_subsystem.setPosition(ELEVATOR_MIN_HEIGHT - 1.0)
    assert elevator_subsystem.current_position == ELEVATOR_MIN_HEIGHT