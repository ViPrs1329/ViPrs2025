"""
Tests for the elevator subsystem.
"""
import pytest
from unittest.mock import MagicMock, patch
from wpimath.controller import PIDController
import rev

from robot.subsystems.elevatorsubsystem import ElevatorSubsystem
from robot.constants import ELEVATOR_MIN_HEIGHT

@pytest.fixture
def elevator_subsystem():
    """
    Creates an elevator subsystem for testing.
    """
    with patch('rev.SparkFlex') as mock_sparkflex, \
         patch('wpimath.controller.PIDController') as mock_pid:
        # Configure mock to handle both deviceID and type arguments
        mock_sparkflex.side_effect = lambda device_id, motor_type: MagicMock()
        
        # Configure PID controller mock
        mock_pid.return_value = MagicMock()
        
        # Create subsystem
        subsystem = ElevatorSubsystem()
        return subsystem

def test_initialization(elevator_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert elevator_subsystem.motor1 is not None
    assert elevator_subsystem.motor2 is not None
    assert elevator_subsystem.encoder is not None
    assert elevator_subsystem.pid is not None
    assert elevator_subsystem.SCORING_HEIGHTS["BASE"] == ELEVATOR_MIN_HEIGHT

def test_position_control(elevator_subsystem):
    """
    Tests position control functions.
    """
    # Test setting position
    elevator_subsystem.setPosition(30.0)
    assert elevator_subsystem.pid.setSetpoint.call_args[0][0] == pytest.approx(0.762)  # 30 inches in meters
    
    # Test getting position
    elevator_subsystem.encoder.getPosition.return_value = 30.0
    assert elevator_subsystem.getPosition() == 30.0

def test_speed_control(elevator_subsystem):
    """
    Tests speed control functions.
    """
    # Test setting speed
    elevator_subsystem.setSpeed(0.5)
    elevator_subsystem.motor1.set.assert_called_once_with(0.5)
    elevator_subsystem.motor2.set.assert_called_once_with(0.5)
    
    # Test stopping
    elevator_subsystem.stop()
    elevator_subsystem.motor1.set.assert_called_with(0)
    elevator_subsystem.motor2.set.assert_called_with(0)

def test_current_monitoring(elevator_subsystem):
    """
    Tests current monitoring functions.
    """
    # Configure mock current values
    elevator_subsystem.motor1.getOutputCurrent.return_value = 10.0
    elevator_subsystem.motor2.getOutputCurrent.return_value = 20.0
    
    # Test getting average current
    assert elevator_subsystem.getCurrent() == 15.0

def test_setpoint_reached(elevator_subsystem):
    """
    Tests setpoint reached detection.
    """
    # Configure mock PID controller
    elevator_subsystem.pid.atSetpoint = MagicMock(return_value=True)
    assert elevator_subsystem.atSetpoint() is True
    
    elevator_subsystem.pid.atSetpoint = MagicMock(return_value=False)
    assert elevator_subsystem.atSetpoint() is False 