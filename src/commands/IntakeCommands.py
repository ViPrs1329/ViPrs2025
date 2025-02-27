# commands/IntakeCommands.py
import commands2
from subsystems.EndEffector import EndEffector
from constants import endEffectorConsts

class IntakeCoralCommand(commands2.CommandBase):
    def __init__(self, endEffector: EndEffector, intake_speed=None):
        """Initialize the IntakeCoralCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            intake_speed (float, optional): Speed for intake. Defaults to value in constants.
        """
        super().__init__()
        self.setName("IntakeCoralCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)
        self.intake_speed = intake_speed if intake_speed is not None else endEffectorConsts.CORAL_INTAKE_SPEED
        self.is_finished = False
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print("Starting coral intake")
        self.is_finished = False
        
    def execute(self):
        """Called repeatedly during command execution.
        
        Returns:
            bool: Whether the intake operation is complete.
        """
        # Run the intake method, which returns True when finished
        self.is_finished = self.endEffector.intakeCoral(self.intake_speed)
        return self.is_finished
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if coral is properly positioned.
        """
        return self.is_finished
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Ending coral intake (interrupted: {interrupted})")
        self.endEffector.stopCoralIntake()