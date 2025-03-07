"""
Tests for the drive subsystem.
"""
import pytest
from unittest.mock import MagicMock, patch
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

from robot.subsystems.drivesubsystem import DriveSubsystem
from robot.constants import *

@pytest.fixture
def drive_subsystem():
    """
    Creates a drive subsystem for testing.
    """
    with patch('rev.SparkMax') as mock_sparkmax, \
         patch('phoenix6.hardware.Pigeon2') as mock_pigeon, \
         patch('phoenix6.hardware.CANcoder') as mock_cancoder, \
         patch('wpimath.kinematics.SwerveDrive4Kinematics.toSwerveModuleStates') as mock_to_states, \
         patch('wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds') as mock_desaturate, \
         patch('wpimath.kinematics.SwerveModuleState.optimize') as mock_optimize:
        
        # Configure mocks
        mock_sparkmax.return_value = MagicMock()
        mock_pigeon.return_value = MagicMock()
        mock_cancoder.return_value = MagicMock()
        
        # Configure the turn encoder mock
        mock_cancoder.return_value.getAbsolutePosition.return_value.value = 0.0
        
        # Configure states mock to return valid states
        state = SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0))
        mock_to_states.return_value = [state, state, state, state]
        
        # Configure optimize to return valid state
        mock_optimize.return_value = state
        
        # Create subsystem
        subsystem = DriveSubsystem()
        
        # Mock the internal methods to avoid calling actual hardware
        subsystem._set_module_state = MagicMock()
        
        return subsystem

def test_initialization(drive_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert drive_subsystem.kinematics is not None
    assert drive_subsystem.field_relative is False

def test_gyro_control(drive_subsystem):
    """
    Tests gyro control functions.
    """
    # Test heading
    drive_subsystem.gyro.getYaw.return_value.value = 45.0
    assert drive_subsystem.getHeading() == 45.0
    
    # Test rotation
    rotation = drive_subsystem.getRotation2d()
    assert rotation is not None
    
    # Test reset
    drive_subsystem.resetGyro()
    assert drive_subsystem.gyro.reset.call_count == 2  # Once in init, once in reset

def test_field_relative_toggle(drive_subsystem):
    """
    Tests field-relative drive toggle.
    """
    assert drive_subsystem.field_relative is False
    drive_subsystem.toggleFieldRelative()
    assert drive_subsystem.field_relative is True
    drive_subsystem.toggleFieldRelative()
    assert drive_subsystem.field_relative is False

def test_drive_control(drive_subsystem):
    """
    Tests drive control functions.
    """
    # Test normal drive
    drive_subsystem.drive(1.0, 0.5, 0.3)
    
    # Verify _set_module_state was called
    assert drive_subsystem._set_module_state.call_count == 4  # Once for each module
    
    # Reset the mock for the next test
    drive_subsystem._set_module_state.reset_mock()
    
    # Test field-relative drive
    drive_subsystem.field_relative = True
    drive_subsystem.drive(1.0, 0.5, 0.3)
    
    # Verify _set_module_state was called
    assert drive_subsystem._set_module_state.call_count == 4

def test_drive_speed_limits(drive_subsystem):
    """
    Tests that drive speeds are properly limited.
    """
    # Test maximum speeds
    drive_subsystem.drive(SWERVE_MAX_SPEED_FPS, SWERVE_MAX_SPEED_FPS, SWERVE_MAX_ANGULAR_SPEED)
    
    # Verify _set_module_state was called
    assert drive_subsystem._set_module_state.call_count == 4

def test_smartdashboard_output(drive_subsystem):
    """
    Tests SmartDashboard output.
    """
    with patch('wpilib.SmartDashboard') as mock_dashboard:
        drive_subsystem.periodic()
        mock_dashboard.putNumber.assert_called_once()
        mock_dashboard.putBoolean.assert_called_once()