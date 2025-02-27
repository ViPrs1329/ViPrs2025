# commands/IntakeCommands.py
import commands2
from subsystems.EndEffector import EndEffector
from constants import endEffectorConsts
import wpilib

class IntakeCoralCommand(commands2.CommandBase):
    """Command to intake coral using the end effector."""
    
    def __init__(self, endEffector: EndEffector, intake_speed=None):
        """Initialize the IntakeCoralCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            intake_speed (float, optional): Speed for intake. Defaults to value in constants.
        """
        super().__init__()
        self.setName("IntakeCoralCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)  # Pass subsystem directly, not in a list
        self.intake_speed = intake_speed if intake_speed is not None else endEffectorConsts.CORAL_INTAKE_SPEED
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print("Starting coral intake")
        wpilib.SmartDashboard.putString("Intake Status", "Starting")
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Run the intake method, which returns True when finished
        is_finished = self.endEffector.intakeCoral(self.intake_speed)
        
        # Update dashboard
        if self.endEffector.isCoralDetected():
            wpilib.SmartDashboard.putString("Intake Status", "Coral Detected")
        else:
            wpilib.SmartDashboard.putString("Intake Status", "Running")
            
        return is_finished
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if coral is properly positioned.
        """
        return self.endEffector.isCoralPositioned()
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Ending coral intake (interrupted: {interrupted})")
        self.endEffector.stopCoralIntake()
        
        if interrupted:
            wpilib.SmartDashboard.putString("Intake Status", "Interrupted")
        else:
            wpilib.SmartDashboard.putString("Intake Status", "Completed")

class EjectCoralCommand(commands2.CommandBase):
    """Command to eject coral from the end effector."""
    
    def __init__(self, endEffector: EndEffector, eject_speed=0.5, timeout=1.0):
        """Initialize the EjectCoralCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            eject_speed (float, optional): Speed for ejection. Default 0.5.
            timeout (float, optional): Timeout in seconds. Default 1.0.
        """
        super().__init__()
        self.setName("EjectCoralCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)  # Pass subsystem directly, not in a list
        self.eject_speed = eject_speed
        self.timeout = timeout
        self.timer = wpilib.Timer()
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print("Starting coral ejection")
        wpilib.SmartDashboard.putString("Intake Status", "Ejecting")
        self.timer.reset()
        self.timer.start()
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Run motors in reverse to eject coral
        self.endEffector.setCoralIntakeLeftSpeed(-self.eject_speed)
        self.endEffector.setCoralIntakeRightSpeed(-self.eject_speed)
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if timed out.
        """
        return self.timer.hasElapsed(self.timeout)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Ending coral ejection (interrupted: {interrupted})")
        self.endEffector.stopCoralIntake()
        wpilib.SmartDashboard.putString("Intake Status", "Idle")