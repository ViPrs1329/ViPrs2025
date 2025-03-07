"""
Commands for the elevator subsystem.
"""
import commands2
from robot.subsystems.elevatorsubsystem import ElevatorSubsystem

class SetElevatorHeight(commands2.Command):
    """
    Command to set the elevator to a specific height.
    """
    
    def __init__(self, subsystem: ElevatorSubsystem, height: str):
        """
        Creates a new SetElevatorHeight command.
        
        Parameters
        ----------
        subsystem : ElevatorSubsystem
            The elevator subsystem to use
        height : str
            The height to set ("BASE", "L1", "L2", "L3", "L4")
        """
        super().__init__()
        
        self.subsystem = subsystem
        self.height = height
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the elevator to the specified height.
        """
        self.subsystem.setPosition(self.subsystem.SCORING_HEIGHTS[self.height])
        
    def isFinished(self) -> bool:
        """
        Returns whether the command has finished.
        
        Returns
        -------
        bool
            True if the elevator has reached its target position
        """
        return self.subsystem.atSetpoint()
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        if interrupted:
            self.subsystem.stop()

class ManualElevatorControl(commands2.Command):
    """
    Command for manual elevator control.
    """
    
    def __init__(self, subsystem: ElevatorSubsystem, speed: float):
        """
        Creates a new ManualElevatorControl command.
        
        Parameters
        ----------
        subsystem : ElevatorSubsystem
            The elevator subsystem to use
        speed : float
            The speed to set (-1 to 1)
        """
        super().__init__()
        
        self.subsystem = subsystem
        self.speed = speed
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the elevator speed.
        """
        self.subsystem.setSpeed(self.speed)
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        self.subsystem.stop() 