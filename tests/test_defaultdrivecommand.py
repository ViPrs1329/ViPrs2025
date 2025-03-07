"""
Tests for the default drive command.
"""
import pytest
import wpilib
import commands2
from unittest.mock import MagicMock, patch

from robot.commands.defaultdrivecommand import DefaultDriveCommand
from robot.subsystems.drivesubsystem import DriveSubsystem
from robot.constants import *

@pytest.fixture
def drive_subsystem():
    """
    Creates a mock drive subsystem for testing.
    """
    return MagicMock(spec=DriveSubsystem)

@pytest.fixture
def drive_command(drive_subsystem):
    """
    Creates a default drive command for testing.
    """
    # Create mock input suppliers
    x_speed = MagicMock(return_value=1.0)
    y_speed = MagicMock(return_value=0.5)
    rot_speed = MagicMock(return_value=0.3)
    precision_mode = MagicMock(return_value=False)
    
    return DefaultDriveCommand(
        drive_subsystem,
        x_speed,
        y_speed,
        rot_speed,
        precision_mode
    )

def test_initialization(drive_command, drive_subsystem):
    """
    Tests that the command initializes correctly.
    """
    assert drive_command.drive_subsystem == drive_subsystem
    assert drive_command.x_speed_supplier is not None
    assert drive_command.y_speed_supplier is not None
    assert drive_command.rot_supplier is not None
    assert drive_command.precision_mode_supplier is not None

def test_execute(drive_command, drive_subsystem):
    """
    Tests the execute method of the command.
    """
    # Execute the command
    drive_command.execute()
    
    # Verify that drive was called with the correct values
    # Note: The values should be scaled by the max speeds
    drive_subsystem.drive.assert_called_once()
    args = drive_subsystem.drive.call_args[0]
    
    # Print actual values for debugging
    print(f"Actual x_speed: {args[0]}, Expected: {SWERVE_MAX_SPEED_FPS}")
    print(f"Actual y_speed: {args[1]}, Expected: {SWERVE_MAX_SPEED_FPS * 0.5 * 0.5}")
    print(f"Actual rot: {args[2]}, Expected: {SWERVE_MAX_ANGULAR_SPEED * 0.3 * 0.3}")
    
    assert abs(args[0] - SWERVE_MAX_SPEED_FPS) < 0.1  # x_speed (1.0 * 1.0 * SWERVE_MAX_SPEED_FPS)
    assert abs(args[1] - SWERVE_MAX_SPEED_FPS * 0.5 * 0.5) < 0.1  # y_speed (0.5 * 0.5 * SWERVE_MAX_SPEED_FPS)
    assert abs(args[2] - SWERVE_MAX_ANGULAR_SPEED * 0.3 * 0.3) < 0.1  # rotation (0.3 * 0.3 * SWERVE_MAX_ANGULAR_SPEED)

def test_precision_mode(drive_command, drive_subsystem):
    """
    Tests that precision mode correctly reduces speeds.
    """
    # Set precision mode to True
    drive_command.precision_mode_supplier.return_value = True
    
    # Execute the command
    drive_command.execute()
    
    # Verify that drive was called with reduced speeds
    args = drive_subsystem.drive.call_args[0]
    assert abs(args[0] - SWERVE_MAX_SPEED_FPS * 0.5) < 0.1  # x_speed (1.0 * 1.0 * 0.5 * SWERVE_MAX_SPEED_FPS)
    assert abs(args[1] - SWERVE_MAX_SPEED_FPS * 0.125) < 0.1  # y_speed (0.5 * 0.5 * 0.5 * SWERVE_MAX_SPEED_FPS)
    assert abs(args[2] - SWERVE_MAX_ANGULAR_SPEED * 0.045) < 0.1  # rotation (0.3 * 0.3 * 0.5 * SWERVE_MAX_ANGULAR_SPEED)

def test_deadband(drive_command, drive_subsystem):
    """
    Tests that the deadband is correctly applied.
    """
    # Set input values below deadband
    drive_command.x_speed_supplier.return_value = 0.05  # Below DEADBAND
    drive_command.y_speed_supplier.return_value = 0.05
    drive_command.rot_supplier.return_value = 0.05
    
    # Execute the command
    drive_command.execute()
    
    # Verify that drive was called with zero speeds
    args = drive_subsystem.drive.call_args[0]
    assert abs(args[0]) < 0.1
    assert abs(args[1]) < 0.1
    assert abs(args[2]) < 0.1

def test_end(drive_command, drive_subsystem):
    """
    Tests that the command stops the robot when ended.
    """
    # End the command
    drive_command.end(False)
    
    # Verify that drive was called with zero speeds
    drive_subsystem.drive.assert_called_once_with(0, 0, 0) 