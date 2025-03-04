from commands2 import Command
from subsystems.elevator_subsystem import ElevatorSubsystem
import wpilib

class ElevatorToPositionCommand(Command):
    def __init__(
        self, 
        elevator: ElevatorSubsystem, 
        target_position: float,
        timeout: float = 5.0
    ) -> None:
        """
        Creates a new ElevatorToPositionCommand.
        
        :param elevator: The elevator subsystem to use
        :param target_position: The target position in meters
        :param timeout: Maximum time to wait for position to be reached (in seconds)
        """
        super().__init__()
        self.elevator = elevator
        self.target_position = target_position
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(elevator)
    
    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.elevator.setPosition(self.target_position)
        self.timer.restart()
    
    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        pass  # PID control is handled by the SparkMax
    
    def end(self, interrupted: bool) -> None:
        """
        Called once the command ends or is interrupted.
        
        :param interrupted: whether the command was interrupted
        """
        self.timer.stop()
    
    def isFinished(self) -> bool:
        """
        Returns true when the command should end.
        
        :return: whether the command should end
        """
        # End if we're at position or if we've timed out
        return self.elevator.isAtPosition() or self.timer.hasElapsed(self.timeout) 