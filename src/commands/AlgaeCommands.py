# commands/AlgaeCommands.py
import commands2
from subsystems.EndEffectorSubsystem import EndEffector
import wpilib

class SetAlgaePositionCommand(commands2.CommandBase):
    """Command to set the algae intake to a specific position."""
    
    def __init__(self, endEffector: EndEffector, target_position: float):
        """Initialize the SetAlgaePositionCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            target_position (float): Target position for algae mechanism in degrees or encoder ticks.
        """
        super().__init__()
        self.setName("SetAlgaePositionCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)
        self.target_position = target_position
        
        # Default tolerance for position control
        self.position_tolerance = 0.05  # Adjust based on your needs
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Setting algae position to: {self.target_position}")
        wpilib.SmartDashboard.putString("Algae Status", "Moving")
        wpilib.SmartDashboard.putNumber("Algae Target Position", self.target_position)
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Get current position from the encoder
        current_position = self.endEffector.getAlgaePosition()
        
        # Calculate position error
        position_error = self.target_position - current_position
        
        # Determine direction based on error
        if abs(position_error) < self.position_tolerance:
            # Within tolerance, stop the motor
            self.endEffector.setAlgaeRotationSpeed(0)
        else:
            # Not at target position, move the motor in the appropriate direction
            direction = 1 if position_error > 0 else -1
            
            # Use a proportional control approach for smoother positioning
            speed = min(abs(position_error) * 2, 0.7) * direction  # Adjust scaling factor as needed
            
            # Set the motor speed
            self.endEffector.setAlgaeRotationSpeed(speed)
        
        # Update dashboard with current position
        wpilib.SmartDashboard.putNumber("Algae Current Position", current_position)
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if algae is at the target position.
        """
        current_position = self.endEffector.getAlgaePosition()
        return abs(self.target_position - current_position) < self.position_tolerance
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Algae positioning ended (interrupted: {interrupted})")
        
        # Stop the motor
        self.endEffector.setAlgaeRotationSpeed(0)
        
        if interrupted:
            wpilib.SmartDashboard.putString("Algae Status", "Interrupted")
        else:
            wpilib.SmartDashboard.putString("Algae Status", "At Position")

# Convenience commands for specific positions
class AlgaeRetractedCommand(commands2.CommandBase):
    """Command to move the algae intake to fully retracted position."""
    
    def __init__(self, endEffector: EndEffector):
        """Initialize the AlgaeRetractedCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
        """
        super().__init__()
        self.setName("AlgaeRetractedCommand")
        # For example, 0 degrees is retracted position
        self.command = SetAlgaePositionCommand(endEffector, 0.0)
        self.addRequirements(endEffector)
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.command.initialize()
        
    def execute(self):
        """Called repeatedly during command execution."""
        self.command.execute()
        
    def isFinished(self):
        """Return whether the command has finished."""
        return self.command.isFinished()
        
    def end(self, interrupted):
        """Called when the command ends."""
        self.command.end(interrupted)

class AlgaeTopPickupCommand(commands2.CommandBase):
    """Command to move the algae intake to top pickup position."""
    
    def __init__(self, endEffector: EndEffector):
        """Initialize the AlgaeTopPickupCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
        """
        super().__init__()
        self.setName("AlgaeTopPickupCommand")
        # For example, 90 degrees is top pickup position
        self.command = SetAlgaePositionCommand(endEffector, 90.0)
        self.addRequirements(endEffector)
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.command.initialize()
        
    def execute(self):
        """Called repeatedly during command execution."""
        self.command.execute()
        
    def isFinished(self):
        """Return whether the command has finished."""
        return self.command.isFinished()
        
    def end(self, interrupted):
        """Called when the command ends."""
        self.command.end(interrupted)

class AlgaeBottomPickupCommand(commands2.CommandBase):
    """Command to move the algae intake to bottom pickup position."""
    
    def __init__(self, endEffector: EndEffector):
        """Initialize the AlgaeBottomPickupCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
        """
        super().__init__()
        self.setName("AlgaeBottomPickupCommand")
        # For example, -90 degrees is bottom pickup position
        self.command = SetAlgaePositionCommand(endEffector, -90.0)
        self.addRequirements(endEffector)
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.command.initialize()
        
    def execute(self):
        """Called repeatedly during command execution."""
        self.command.execute()
        
    def isFinished(self):
        """Return whether the command has finished."""
        return self.command.isFinished()
        
    def end(self, interrupted):
        """Called when the command ends."""
        self.command.end(interrupted)

class AlgaeIntakeCommand(commands2.CommandBase):
    """Command to run the algae intake."""
    
    def __init__(self, endEffector: EndEffector, speed=0.7):
        """Initialize the AlgaeIntakeCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            speed (float, optional): Intake speed. Defaults to 0.7.
        """
        super().__init__()
        self.setName("AlgaeIntakeCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)
        self.speed = speed
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Starting algae intake at speed: {self.speed}")
        wpilib.SmartDashboard.putString("Algae Intake Status", "Running")
        
    def execute(self):
        """Called repeatedly during command execution."""
        self.endEffector.setAlgaeIntakeSpeed(self.speed)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Algae intake ended (interrupted: {interrupted})")
        self.endEffector.setAlgaeIntakeSpeed(0)
        wpilib.SmartDashboard.putString("Algae Intake Status", "Stopped")
        
    def isFinished(self):
        """The command runs until interrupted."""
        return False

class AlgaeEjectCommand(commands2.CommandBase):
    """Command to eject from the algae intake."""
    
    def __init__(self, endEffector: EndEffector, speed=0.7):
        """Initialize the AlgaeEjectCommand.
        
        Args:
            endEffector (EndEffector): The EndEffector subsystem.
            speed (float, optional): Eject speed. Defaults to 0.7.
        """
        super().__init__()
        self.setName("AlgaeEjectCommand")
        self.endEffector = endEffector
        self.addRequirements(endEffector)
        self.speed = speed
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Starting algae ejection at speed: {self.speed}")
        wpilib.SmartDashboard.putString("Algae Intake Status", "Ejecting")
        
    def execute(self):
        """Called repeatedly during command execution."""
        self.endEffector.setAlgaeIntakeSpeed(-self.speed)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Algae ejection ended (interrupted: {interrupted})")
        self.endEffector.setAlgaeIntakeSpeed(0)
        wpilib.SmartDashboard.putString("Algae Intake Status", "Stopped")
        
    def isFinished(self):
        """The command runs until interrupted."""
        return False