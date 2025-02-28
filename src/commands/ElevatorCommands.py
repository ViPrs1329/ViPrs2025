# commands/ElevatorCommands.py
import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
from constants import elevatorConsts

class MoveElevatorToPositionCommand(commands2.CommandBase):
    """Command to move the elevator to a preset position."""
    
    def __init__(self, elevator: Elevator, target_position: float, tolerance=0.5):
        """Initialize the MoveElevatorToPositionCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
            target_position (float): Target position in encoder ticks or meters.
            tolerance (float, optional): Position tolerance. Defaults to 0.5.
        """
        super().__init__()
        self.setName("MoveElevatorToPositionCommand")
        self.elevator = elevator
        self.addRequirements(elevator)
        self.target_position = target_position
        self.tolerance = tolerance
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Moving elevator to position: {self.target_position}")
        wpilib.SmartDashboard.putString("Elevator Status", "Moving")
        wpilib.SmartDashboard.putNumber("Elevator Target", self.target_position)
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Use the elevator's moveToPosition method
        self.elevator.moveToPosition(self.target_position)
        
        # Update dashboard with current position
        current_position = self.elevator.getCurrentPosition()
        wpilib.SmartDashboard.putNumber("Elevator Position", current_position)
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the elevator is at the target position.
        """
        return self.elevator.isAtPosition(self.target_position, self.tolerance)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Elevator move ended (interrupted: {interrupted})")
        
        if interrupted:
            # Hold position if interrupted
            self.elevator.holdPosition()
            wpilib.SmartDashboard.putString("Elevator Status", "Interrupted")
        else:
            # Hold position at target
            self.elevator.holdPosition()
            wpilib.SmartDashboard.putString("Elevator Status", "At Position")


class ElevatorPresetCommand(commands2.CommandBase):
    """Command to move the elevator to a preset height."""
    
    def __init__(self, elevator: Elevator, preset_name: str):
        """Initialize the ElevatorPresetCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
            preset_name (str): Name of the preset position (e.g., "low", "medium", "high").
        """
        super().__init__()
        self.setName(f"ElevatorPreset-{preset_name}")
        self.elevator = elevator
        self.addRequirements(elevator)
        self.preset_name = preset_name
        
        # Get the target position from the constants
        # This allows the constants to be updated without changing command code
        self.target_position = self._get_preset_position(preset_name)
        
    def _get_preset_position(self, preset_name: str) -> float:
        """Get the position value for a preset name.
        
        Args:
            preset_name (str): Name of the preset position.
            
        Returns:
            float: The position value in encoder ticks or meters.
        """
        # Use getattr to look up the position in the constants
        # Default to 0 if the preset doesn't exist
        preset_constant_name = f"{preset_name.upper()}_POSITION"
        return getattr(elevatorConsts, preset_constant_name, 0.0)
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Moving elevator to preset: {self.preset_name} (position: {self.target_position})")
        wpilib.SmartDashboard.putString("Elevator Status", f"Moving to {self.preset_name}")
        wpilib.SmartDashboard.putNumber("Elevator Target", self.target_position)
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Use the elevator's moveToPosition method
        self.elevator.moveToPosition(self.target_position)
        
        # Update dashboard with current position
        current_position = self.elevator.getCurrentPosition()
        wpilib.SmartDashboard.putNumber("Elevator Position", current_position)
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the elevator is at the target position.
        """
        return self.elevator.isAtPosition(self.target_position)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        print(f"Elevator preset move ended (interrupted: {interrupted})")
        
        if interrupted:
            # Hold position if interrupted
            self.elevator.holdPosition()
            wpilib.SmartDashboard.putString("Elevator Status", "Interrupted")
        else:
            # Hold position at target
            self.elevator.holdPosition()
            wpilib.SmartDashboard.putString("Elevator Status", f"At {self.preset_name}")

# Convenience commands for specific presets
class ElevatorLowPositionCommand(commands2.CommandBase):
    """Command to move the elevator to the low position."""
    
    def __init__(self, elevator: Elevator):
        """Initialize the ElevatorLowPositionCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
        """
        super().__init__()
        self.setName("ElevatorLowPosition")
        self.command = ElevatorPresetCommand(elevator, "low")
        self.addRequirements(elevator)
        
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

class ElevatorMediumPositionCommand(commands2.CommandBase):
    """Command to move the elevator to the medium position."""
    
    def __init__(self, elevator: Elevator):
        """Initialize the ElevatorMediumPositionCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
        """
        super().__init__()
        self.setName("ElevatorMediumPosition")
        self.command = ElevatorPresetCommand(elevator, "medium")
        self.addRequirements(elevator)
        
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

class ElevatorHighPositionCommand(commands2.CommandBase):
    """Command to move the elevator to the high position."""
    
    def __init__(self, elevator: Elevator):
        """Initialize the ElevatorHighPositionCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
        """
        super().__init__()
        self.setName("ElevatorHighPosition")
        self.command = ElevatorPresetCommand(elevator, "high")
        self.addRequirements(elevator)
        
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

class ElevatorHomePositionCommand(commands2.CommandBase):
    """Command to move the elevator to the home/stowed position."""
    
    def __init__(self, elevator: Elevator):
        """Initialize the ElevatorHomePositionCommand.
        
        Args:
            elevator (Elevator): The Elevator subsystem.
        """
        super().__init__()
        self.setName("ElevatorHomePosition")
        self.command = ElevatorPresetCommand(elevator, "home")
        self.addRequirements(elevator)
        
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