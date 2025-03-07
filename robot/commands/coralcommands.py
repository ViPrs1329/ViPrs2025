"""
Commands for the CORAL subsystem.
"""
import commands2
from subsystems.coralsubsystem import CoralSubsystem

class IntakeCoral(commands2.Command):
    """
    Command to intake CORAL.
    """
    
    def __init__(self, subsystem: CoralSubsystem):
        """
        Creates a new IntakeCoral command.
        
        Parameters
        ----------
        subsystem : CoralSubsystem
            The CORAL subsystem to use
        """
        super().__init__()
        
        self.subsystem = subsystem
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the CORAL wheels to intake speed.
        """
        self.subsystem.setSpeed(0.8)  # 80% speed for intake
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        self.subsystem.stop()

class EjectCoral(commands2.Command):
    """
    Command to eject CORAL.
    """
    
    def __init__(self, subsystem: CoralSubsystem):
        """
        Creates a new EjectCoral command.
        
        Parameters
        ----------
        subsystem : CoralSubsystem
            The CORAL subsystem to use
        """
        super().__init__()
        
        self.subsystem = subsystem
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the CORAL wheels to eject speed.
        """
        self.subsystem.setSpeed(-0.8)  # 80% speed for ejection
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        self.subsystem.stop() 