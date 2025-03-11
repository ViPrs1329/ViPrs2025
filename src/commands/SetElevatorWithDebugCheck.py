import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
import constants

class SetElevatorWithDebugCheck(commands2.Command):
    """
    Extended version of the SetElevator command that checks if debug mode is enabled
    before executing. If debug mode is enabled, the command will not run.
    """
    def __init__(self, direction: str, elevator: Elevator, endEffector: EndEffector, debug_mode_ref):
        """
        Initialize the command.
        
        Args:
            direction: "up" or "down" to specify elevator movement direction
            elevator: The elevator subsystem
            endEffector: The end effector subsystem
            debug_mode_ref: A list containing a single boolean value for debug mode state
        """
        super().__init__()
        self.direction = direction  # "up" or "down"
        self.elevator = elevator
        self.endEffector = endEffector
        self.debug_mode_ref = debug_mode_ref
        
        # Add requirements for command scheduling
        self.addRequirements(elevator, endEffector)
        
        # Constants for elevator levels
        self.MIN_LEVEL = 1
        self.MAX_LEVEL = 4

    def initialize(self):
        """Called when the command is initially scheduled."""
        # Check if debug mode is enabled - if so, don't execute
        if self.debug_mode_ref[0]:
            print("Command ignored: Elevator in debug mode")
            return
            
        # Set the next elevator level based on direction
        if self.direction == "up":
            self.elevator.currentLevel = min(self.elevator.currentLevel + 1, self.MAX_LEVEL)
        elif self.direction == "down":
            self.elevator.currentLevel = max(self.elevator.currentLevel - 1, self.MIN_LEVEL)
        else:
            raise ValueError("Direction must be 'up' or 'down'")
            
        # Get the target level (zero-indexed for array access)
        level_index = self.elevator.currentLevel - 1
        
        # Calculate elevator position and arm angle for the target level
        target_height = constants.reefConsts.reefLevels[level_index][1] + constants.elevatorConsts.verticalOffset
        target_position = constants.convert.in2rot(target_height)
        
        # Move the elevator and arm to the appropriate positions
        self.elevator.gotoPosition(target_position)
        self.endEffector.setAlgaeArmAngle(constants.intakeConsts.algaeArmAngles[level_index])
        
        # Log the level change
        print(f"Elevator moving to level {self.elevator.currentLevel}")

    def execute(self):
        # No execution needed - all work done in initialize
        pass

    def end(self, interrupted: bool):
        if interrupted:
            print(f"Elevator command interrupted at level {self.elevator.currentLevel}")
    
    def isFinished(self) -> bool:
        # This is a one-shot command, finish immediately even if in debug mode
        return True