import pytest
from unittest.mock import MagicMock, patch
from wpilib import XboxController
from robotcontainer import RobotContainer
import rev
import phoenix6.hardware
import wpilib.simulation

@pytest.fixture(autouse=True)
def mock_hardware():
    """Mock hardware components for testing."""
    # Create mock objects
    mock_encoder = MagicMock()
    mock_encoder.getPosition.return_value = 0.0
    mock_encoder.getVelocity.return_value = 0.0
    
    mock_pid = MagicMock()
    mock_motor = MagicMock()
    mock_motor.getEncoder.return_value = mock_encoder
    mock_motor.getPIDController.return_value = mock_pid
    
    mock_cancoder = MagicMock()
    mock_cancoder.get_position.return_value = MagicMock(value=0.0)
    
    mock_lasercan = MagicMock()
    mock_lasercan.get.return_value = False
    
    # Create patches
    patches = [
        patch('rev.SparkMax', return_value=mock_motor),
        patch('phoenix6.hardware.CANcoder', return_value=mock_cancoder),
        patch('libgrapplefrc.LaserCAN', return_value=mock_lasercan),
        patch('wpilib.ADIS16470_IMU', return_value=MagicMock()),
    ]
    
    # Apply all patches
    for p in patches:
        p.start()
    
    yield
     
    # Remove all patches
    for p in patches:
        p.stop()

def test_robot_container_init():
    """Test that RobotContainer initializes without errors"""
    container = RobotContainer()
    assert container is not None
    assert isinstance(container.driver_controller, XboxController)
    assert isinstance(container.operator_controller, XboxController)

def test_get_autonomous_command():
    """Test that getAutonomousCommand returns a command"""
    container = RobotContainer()
    auto_command = container.getAutonomousCommand()
    # Initially returns None since we haven't set up autonomous commands yet
    assert auto_command is None 