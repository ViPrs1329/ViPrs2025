"""
Commands for the ALGAE subsystem.
"""
import commands2
<<<<<<< HEAD
from subsystems.algaesubsystem import AlgaeSubsystem
from constants import ALGAE_ARM_WORKING_ANGLE
=======
from robot.subsystems.algaesubsystem import AlgaeSubsystem
from robot.constants import ALGAE_ARM_WORKING_ANGLE
>>>>>>> 0f35bd26675a1644dc9f6f438f5c9e4297dc0f25

class SetArmPosition(commands2.Command):
    """
    Command to set the ALGAE arm to a specific position.
    """
    
    def __init__(self, subsystem: AlgaeSubsystem, position: float):
        """
        Creates a new SetArmPosition command.
        
        Parameters
        ----------
        subsystem : AlgaeSubsystem
            The ALGAE subsystem to use
        position : float
            The position to set in degrees
        """
        super().__init__()
        
        self.subsystem = subsystem
        self.position = position
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the arm to the specified position.
        """
        self.subsystem.setArmPosition(self.position)
        
    def isFinished(self) -> bool:
        """
        Returns whether the command has finished.
        
        Returns
        -------
        bool
            True if the arm has reached its target position
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

class IntakeAlgae(commands2.Command):
    """
    Command to intake ALGAE.
    """
    
    def __init__(self, subsystem: AlgaeSubsystem):
        """
        Creates a new IntakeAlgae command.
        
        Parameters
        ----------
        subsystem : AlgaeSubsystem
            The ALGAE subsystem to use
        """
        super().__init__()
        
        self.subsystem = subsystem
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the arm to working position and intake wheels to intake speed.
        """
        self.subsystem.setArmPosition(ALGAE_ARM_WORKING_ANGLE)
        self.subsystem.setIntakeSpeed(0.8)  # 80% speed for intake
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        self.subsystem.stop()

class EjectAlgae(commands2.Command):
    """
    Command to eject ALGAE.
    """
    
    def __init__(self, subsystem: AlgaeSubsystem):
        """
        Creates a new EjectAlgae command.
        
        Parameters
        ----------
        subsystem : AlgaeSubsystem
            The ALGAE subsystem to use
        """
        super().__init__()
        
        self.subsystem = subsystem
        
        self.addRequirements(subsystem)
        
    def execute(self) -> None:
        """
        Sets the intake wheels to eject speed.
        """
        self.subsystem.setIntakeSpeed(-0.8)  # 80% speed for ejection
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        
        Parameters
        ----------
        interrupted : bool
            Whether the command was interrupted
        """
        self.subsystem.stop() 