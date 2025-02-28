# commands/ScoringCommands.py
import commands2
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
from commands.IntakeCommands import EjectCoralCommand
from commands.ElevatorCommands import (
    ElevatorLowPositionCommand, 
    ElevatorMediumPositionCommand, 
    ElevatorHighPositionCommand,
    ElevatorHomePositionCommand
)
import wpilib

class ScoreLowCommand(commands2.SequentialCommandGroup):
    """Command sequence to score coral at the low level."""
    
    def __init__(self, elevator: Elevator, endEffector: EndEffector):
        """Initialize the ScoreLowCommand.
        
        Args:
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
        """
        super().__init__()
        self.setName("ScoreLowCommand")
        
        # Add commands to the sequential group
        self.addCommands(
            # First, move elevator to low position
            ElevatorLowPositionCommand(elevator),
            
            # Then, eject the coral (1 second)
            EjectCoralCommand(endEffector, 0.7, 1.0),
            
            # Finally, return to home position
            ElevatorHomePositionCommand(elevator)
        )

class ScoreMediumCommand(commands2.SequentialCommandGroup):
    """Command sequence to score coral at the medium level."""
    
    def __init__(self, elevator: Elevator, endEffector: EndEffector):
        """Initialize the ScoreMediumCommand.
        
        Args:
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
        """
        super().__init__()
        self.setName("ScoreMediumCommand")
        
        # Add commands to the sequential group
        self.addCommands(
            # First, move elevator to medium position
            ElevatorMediumPositionCommand(elevator),
            
            # Then, eject the coral (1 second)
            EjectCoralCommand(endEffector, 0.7, 1.0),
            
            # Finally, return to home position
            ElevatorHomePositionCommand(elevator)
        )

class ScoreHighCommand(commands2.SequentialCommandGroup):
    """Command sequence to score coral at the high level."""
    
    def __init__(self, elevator: Elevator, endEffector: EndEffector):
        """Initialize the ScoreHighCommand.
        
        Args:
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
        """
        super().__init__()
        self.setName("ScoreHighCommand")
        
        # Add commands to the sequential group
        self.addCommands(
            # First, move elevator to high position
            ElevatorHighPositionCommand(elevator),
            
            # Then, eject the coral (1 second)
            EjectCoralCommand(endEffector, 0.7, 1.0),
            
            # Finally, return to home position
            ElevatorHomePositionCommand(elevator)
        )

class QuickScoreSequence(commands2.SequentialCommandGroup):
    """Quick scoring sequence triggered by button combination."""
    
    def __init__(self, elevator: Elevator, endEffector: EndEffector, level: str = "low"):
        """Initialize the QuickScoreSequence.
        
        Args:
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
            level (str, optional): Scoring level ("low", "medium", "high"). Defaults to "low".
        """
        super().__init__()
        self.setName(f"QuickScore-{level}")
        
        # Select the appropriate scoring command based on level
        if level.lower() == "low":
            self.addCommands(ScoreLowCommand(elevator, endEffector))
        elif level.lower() == "medium":
            self.addCommands(ScoreMediumCommand(elevator, endEffector))
        elif level.lower() == "high":
            self.addCommands(ScoreHighCommand(elevator, endEffector))
        else:
            # Default to low if invalid level specified
            self.addCommands(ScoreLowCommand(elevator, endEffector))