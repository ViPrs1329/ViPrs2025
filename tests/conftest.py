"""
Pytest configuration for RobotPy testing.
"""
import pytest
import wpilib
import commands2

@pytest.fixture(scope="session", autouse=True)
def wpilib_setup():
    """
    Sets up WPILib for testing.
    """
    # Initialize WPILib
    wpilib.DriverStation.silenceJoystickConnectionWarning(True)
    
    # Create a command scheduler
    scheduler = commands2.CommandScheduler.getInstance()
    
    # Yield the scheduler
    yield scheduler
    
    # Cleanup
    scheduler.cancelAll() 