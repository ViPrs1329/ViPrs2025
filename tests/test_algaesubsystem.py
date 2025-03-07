"""
Tests for the ALGAE subsystem using the test-specific implementation.
"""
import pytest
from test_algae_implementation import TestAlgaeSubsystem
from robot.constants import ALGAE_ARM_REST_ANGLE, ALGAE_ARM_WORKING_ANGLE

@pytest.fixture
def algae_subsystem():
    """
    Creates a test ALGAE subsystem for testing.
    """
    return TestAlgaeSubsystem()

def test_initialization(algae_subsystem):
    """
    Tests that the subsystem initializes correctly.
    """
    assert algae_subsystem is not None
    assert algae_subsystem.current_arm_position == 0.0
    assert algae_subsystem.current_intake_speed == 0.0
    assert len(algae_subsystem.set_arm_position_calls) == 0
    assert len(algae_subsystem.set_intake_speed_calls) == 0

def test_arm_position_control(algae_subsystem):
    """
    Tests arm position control functions.
    """
    # Test setting arm position
    algae_subsystem.setArmPosition(ALGAE_ARM_WORKING_ANGLE)
    
    # Check that position was set
    assert algae_subsystem.current_arm_position == ALGAE_ARM_WORKING_ANGLE
    assert algae_subsystem.set_arm_position_calls == [ALGAE_ARM_WORKING_ANGLE]
    
    # Test getting position
    assert algae_subsystem.getArmPosition() == ALGAE_ARM_WORKING_ANGLE
    
    # Test stopping
    algae_subsystem.stop()
    
    # Check that position was maintained
    assert algae_subsystem.current_arm_position == ALGAE_ARM_WORKING_ANGLE

def test_intake_speed_control(algae_subsystem):
    """
    Tests intake speed control functions.
    """
    # Test setting intake speed
    algae_subsystem.setIntakeSpeed(0.8)
    
    # Check that speed was set
    assert algae_subsystem.current_intake_speed == 0.8
    assert algae_subsystem.set_intake_speed_calls == [0.8]
    
    # Test stopping
    algae_subsystem.stop()
    
    # Check that speed was set to 0
    assert algae_subsystem.current_intake_speed == 0.0
    assert algae_subsystem.set_intake_speed_calls == [0.8, 0.0]

def test_setpoint_reached(algae_subsystem):
    """
    Tests setpoint reached detection.
    """
    # Test with arm not at setpoint
    algae_subsystem.setArmPosition(ALGAE_ARM_WORKING_ANGLE)
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE + 5.0
    assert algae_subsystem.atSetpoint() is False
    
    # Test with arm at setpoint
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE
    assert algae_subsystem.atSetpoint() is True

def test_bottom_position_detection(algae_subsystem):
    """
    Tests bottom position detection.
    """
    # Test not at bottom
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE
    assert algae_subsystem.isAtBottom() is False
    
    # Test at bottom
    algae_subsystem.current_arm_position = ALGAE_ARM_REST_ANGLE
    assert algae_subsystem.isAtBottom() is True

def test_motor_current_monitoring(algae_subsystem):
    """
    Tests motor current monitoring functionality.
    """
    # Test setting motor currents
    algae_subsystem.set_motor_currents(arm_current=30.0, intake_current=25.0)
    
    # Check that currents were set
    assert algae_subsystem.arm_motor_current == 30.0
    assert algae_subsystem.intake_motor_current == 25.0

def test_multiple_position_changes(algae_subsystem):
    """
    Tests multiple position changes in sequence.
    """
    # Test sequence of position changes
    positions = [ALGAE_ARM_REST_ANGLE, ALGAE_ARM_WORKING_ANGLE, ALGAE_ARM_REST_ANGLE]
    
    for pos in positions:
        algae_subsystem.setArmPosition(pos)
        assert algae_subsystem.current_arm_position == pos
    
    # Verify all calls were recorded
    assert algae_subsystem.set_arm_position_calls == positions

def test_intake_speed_limits(algae_subsystem):
    """
    Tests intake speed limit handling.
    """
    # Test maximum speed (1.0)
    algae_subsystem.setIntakeSpeed(1.0)
    assert algae_subsystem.current_intake_speed == 1.0
    
    # Test minimum speed (-1.0)
    algae_subsystem.setIntakeSpeed(-1.0)
    assert algae_subsystem.current_intake_speed == -1.0
    
    # Test speed clamping (should be handled by the actual subsystem)
    algae_subsystem.setIntakeSpeed(1.5)  # Above max
    assert algae_subsystem.current_intake_speed == 1.0
    
    algae_subsystem.setIntakeSpeed(-1.5)  # Below min
    assert algae_subsystem.current_intake_speed == -1.0

def test_stop_behavior(algae_subsystem):
    """
    Tests stop behavior in various states.
    """
    # Test stopping while moving
    algae_subsystem.setArmPosition(ALGAE_ARM_WORKING_ANGLE)
    algae_subsystem.setIntakeSpeed(0.8)
    algae_subsystem.stop()
    
    assert algae_subsystem.current_intake_speed == 0.0
    assert algae_subsystem.current_arm_position == ALGAE_ARM_WORKING_ANGLE  # Position should be maintained
    
    # Test stopping while already stopped
    algae_subsystem.stop()
    assert algae_subsystem.current_intake_speed == 0.0
    assert algae_subsystem.current_arm_position == ALGAE_ARM_WORKING_ANGLE

def test_position_accuracy(algae_subsystem):
    """
    Tests position accuracy and tolerance.
    """
    # Test exact position
    algae_subsystem.setArmPosition(ALGAE_ARM_WORKING_ANGLE)
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE
    assert algae_subsystem.atSetpoint() is True
    
    # Test within tolerance (1 degree)
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE + 0.5
    assert algae_subsystem.atSetpoint() is True
    
    # Test outside tolerance
    algae_subsystem.current_arm_position = ALGAE_ARM_WORKING_ANGLE + 1.5
    assert algae_subsystem.atSetpoint() is False 