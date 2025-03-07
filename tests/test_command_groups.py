"""
Tests for command groups and sequences.
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
    ELEVATOR_L1_HEIGHT,
    ELEVATOR_L2_HEIGHT,
    ELEVATOR_L3_HEIGHT,
    ALGAE_ARM_WORKING_ANGLE
)

@pytest.fixture
def mock_subsystems():
    """Creates mock subsystems for testing."""
    elevator = MagicMock(spec=ElevatorSubsystem)
    coral = MagicMock(spec=CoralSubsystem)
    algae = MagicMock(spec=AlgaeSubsystem)
    return elevator, coral, algae

def test_score_coral_sequence(mock_subsystems):
    """
    Tests a scoring sequence for CORAL game pieces.
    """
    elevator, coral, _ = mock_subsystems
    
    # Define scoring sequence (move elevator to L2, then eject)
    sequence = commands2.SequentialCommandGroup(
        SetElevatorHeight(elevator, ELEVATOR_L2_HEIGHT),
        EjectCoral(coral)
    )
    
    # Initialize sequence
    sequence.initialize()
    
    # First step: Move elevator
    sequence.execute()
    elevator.setPosition.assert_called_with(ELEVATOR_L2_HEIGHT)
    
    # Simulate elevator reaching position
    elevator.atSetpoint.return_value = True
    
    # Next step: Eject coral
    sequence.execute()
    coral.setSpeed.assert_called_with(-0.8)

def test_collect_algae_sequence(mock_subsystems):
    """
    Tests a collection sequence for ALGAE game pieces.
    """
    elevator, _, algae = mock_subsystems
    
    # Define collection sequence (move elevator to L1, then intake)
    sequence = commands2.SequentialCommandGroup(
        SetElevatorHeight(elevator, ELEVATOR_L1_HEIGHT),
        IntakeAlgae(algae)
    )
    
    # Initialize sequence
    sequence.initialize()
    
    # First step: Move elevator
    sequence.execute()
    elevator.setPosition.assert_called_with(ELEVATOR_L1_HEIGHT)
    
    # Simulate elevator reaching position
    elevator.atSetpoint.return_value = True
    
    # Next step: Intake algae
    sequence.execute()
    algae.setArmPosition.assert_called_with(ALGAE_ARM_WORKING_ANGLE)
    algae.setIntakeSpeed.assert_called_with(0.8)

def test_parallel_operations(mock_subsystems):
    """
    Tests parallel command groups.
    """
    elevator, coral, algae = mock_subsystems
    
    # Define parallel operations (move elevator while running intake)
    parallel_group = commands2.ParallelCommandGroup(
        SetElevatorHeight(elevator, ELEVATOR_L3_HEIGHT),
        IntakeCoral(coral)
    )
    
    # Initialize and execute
    parallel_group.initialize()
    parallel_group.execute()
    
    # Verify both operations started
    elevator.setPosition.assert_called_with(ELEVATOR_L3_HEIGHT)
    coral.setSpeed.assert_called_with(0.8)

def test_command_interruption(mock_subsystems):
    """
    Tests that commands can be safely interrupted.
    """
    elevator, coral, _ = mock_subsystems
    
    # Create a sequence
    sequence = commands2.SequentialCommandGroup(
        SetElevatorHeight(elevator, ELEVATOR_L2_HEIGHT),
        EjectCoral(coral)
    )
    
    # Start sequence
    sequence.initialize()
    sequence.execute()
    
    # Interrupt before completion
    sequence.end(True)  # True indicates interrupted
    
    # Verify subsystems are in safe state
    elevator.stop.assert_called()
    coral.stop.assert_called() 