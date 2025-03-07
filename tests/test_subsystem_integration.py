"""
Tests for subsystem integration and interactions.
"""
import pytest
from unittest.mock import MagicMock
import commands2

from robot.commands.elevatorcommands import SetElevatorHeight
from robot.commands.coralcommands import IntakeCoral, EjectCoral
from robot.commands.algaecommands import IntakeAlgae, EjectAlgae
from robot.subsystems.elevatorsubsystem import ElevatorSubsystem
from robot.subsystems.coralsubsystem import CoralSubsystem
from robot.subsystems.algaesubsystem import AlgaeSubsystem
from robot.constants import (
    ELEVATOR_MIN_HEIGHT,
    ELEVATOR_L1_HEIGHT,
    ELEVATOR_L2_HEIGHT,
    ELEVATOR_L3_HEIGHT,
    ELEVATOR_L4_HEIGHT,
    ALGAE_ARM_REST_ANGLE,
    ALGAE_ARM_WORKING_ANGLE
)

@pytest.fixture
def mock_subsystems():
    """Creates mock subsystems for testing."""
    elevator = MagicMock(spec=ElevatorSubsystem)
    coral = MagicMock(spec=CoralSubsystem)
    algae = MagicMock(spec=AlgaeSubsystem)
    return elevator, coral, algae

def test_coral_elevator_interaction(mock_subsystems):
    """
    Tests that coral subsystem works correctly with elevator at different heights.
    """
    elevator, coral, _ = mock_subsystems
    
    # Test coral intake at different elevator positions
    heights = [ELEVATOR_MIN_HEIGHT, ELEVATOR_L1_HEIGHT, ELEVATOR_L2_HEIGHT]
    
    for height in heights:
        # Setup elevator position
        elevator.getPosition.return_value = height
        
        # Run coral intake
        intake_cmd = IntakeCoral(coral)
        intake_cmd.initialize()
        intake_cmd.execute()
        
        # Verify coral subsystem was called correctly
        coral.setSpeed.assert_called_with(0.8)

def test_algae_elevator_interaction(mock_subsystems):
    """
    Tests that algae subsystem works correctly with elevator at different heights.
    """
    elevator, _, algae = mock_subsystems
    
    # Test algae operations at different elevator positions
    heights = [ELEVATOR_MIN_HEIGHT, ELEVATOR_L1_HEIGHT, ELEVATOR_L2_HEIGHT]
    
    for height in heights:
        # Setup elevator position
        elevator.getPosition.return_value = height
        
        # Test intake
        intake_cmd = IntakeAlgae(algae)
        intake_cmd.initialize()
        intake_cmd.execute()
        
        # Verify algae subsystem was called correctly for intake
        algae.setArmPosition.assert_called_with(ALGAE_ARM_WORKING_ANGLE)
        algae.setIntakeSpeed.assert_called_with(0.8)

def test_subsystem_safety_interactions(mock_subsystems):
    """
    Tests safety interactions between subsystems.
    """
    elevator, coral, algae = mock_subsystems
    
    # Test that coral and algae operations are safe at elevator extremes
    
    # At minimum height
    elevator.getPosition.return_value = ELEVATOR_MIN_HEIGHT
    
    # Verify coral operations
    intake_coral = IntakeCoral(coral)
    intake_coral.initialize()
    intake_coral.execute()
    coral.setSpeed.assert_called_with(0.8)
    
    # Verify algae operations
    intake_algae = IntakeAlgae(algae)
    intake_algae.initialize()
    intake_algae.execute()
    algae.setArmPosition.assert_called_with(ALGAE_ARM_WORKING_ANGLE)
    algae.setIntakeSpeed.assert_called_with(0.8)

    # At maximum height
    elevator.getPosition.return_value = ELEVATOR_L4_HEIGHT
    
    # Verify coral operations still work
    eject_coral = EjectCoral(coral)
    eject_coral.initialize()
    eject_coral.execute()
    coral.setSpeed.assert_called_with(-0.8)
    
    # Verify algae operations still work
    eject_algae = EjectAlgae(algae)
    eject_algae.initialize()
    eject_algae.execute()
    algae.setIntakeSpeed.assert_called_with(-0.8) 