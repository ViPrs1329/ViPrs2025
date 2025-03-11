import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
import constants

class SetElevator(commands2.Command):
    def __init__(self, direction: str, elevator: Elevator, endEffector: EndEffector):
        super().__init__()
        self.direction = direction  # "up" or "down"
        self.elevator = elevator
        self.endEffector = endEffector
        
        # Add requirements for command scheduling
        self.addRequirements(elevator, endEffector)
        
        # Constants for elevator levels
        self.MIN_LEVEL = 1
        self.MAX_LEVEL = 4

    def initialize(self):
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
        # This is a one-shot command
        return True