"""
Tests for the elevator subsystem using the test-specific implementation.
"""
import pytest
from test_elevator_implementation import TestElevatorSubsystem
from robot.constants import ELEVATOR_MIN_HEIGHT

@pytest.fixture
def elevator_subsystem():
    """
    Creates a test elevator subsystem for testing.
    """
    return TestElevatorSubsystem()

def test_initialization(elevator_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert elevator_subsystem is not None
    assert elevator_subsystem.SCORING_HEIGHTS["BASE"] == ELEVATOR_MIN_HEIGHT
    assert len(elevator_subsystem.SCORING_HEIGHTS) == 5  # Base + 4 levels

def test_position_control(elevator_subsystem):
    """
    Tests position control functions.
    """
    # Test setting position to 30 inches
    elevator_subsystem.setPosition(30.0)
    
    # Check the position was set
    assert elevator_subsystem.target_position == 30.0
    assert elevator_subsystem.set_position_calls == [30.0]
    
    # Test getting position
    elevator_subsystem.set_current_position(35.0)
    assert elevator_subsystem.getPosition() == 35.0

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

def test_current_monitoring(elevator_subsystem):
    """
    Tests current monitoring functions.
    """
    # Set different current values for the two motors
    elevator_subsystem.set_motor_currents(10.0, 20.0)
    
    # Test getting average current
    assert elevator_subsystem.getCurrent() == 15.0

def test_setpoint_reached(elevator_subsystem):
    """
    Tests setpoint reached detection.
    """
    # Test with elevator not at setpoint
    elevator_subsystem.set_at_setpoint(False)
    assert elevator_subsystem.atSetpoint() is False
    
    # Test with elevator at setpoint
    elevator_subsystem.set_at_setpoint(True)
    assert elevator_subsystem.atSetpoint() is True