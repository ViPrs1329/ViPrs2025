import commands2
from subsystems.ElevatorSubsystem import Elevator
from wpilib import XboxController
import constants

class JoystickElevatorControl(commands2.Command):
    """
    Command to control the elevator using a joystick for debugging purposes.
    This allows for manual control of the elevator position using an Xbox controller's right stick.
    """
    def __init__(self, elevator: Elevator, controller: XboxController, scale_factor: float = 0.5):
        """
        Initialize the joystick elevator control command.
        
        Args:
            elevator: The elevator subsystem to control
            controller: The Xbox controller to use for input
            scale_factor: How much to scale the joystick input by (default: 0.5)
        """
        super().__init__()
        self.elevator = elevator
        self.controller = controller
        self.scale_factor = scale_factor
        self.addRequirements(elevator)
        
        # Save the original position when starting joystick control
        self.starting_position = 0
        self.current_position = 0
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.starting_position = self.elevator.getElevatorPosition()
        self.current_position = self.starting_position
        print(f"Starting elevator joystick control at position: {self.starting_position}")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        # Get joystick input (using right stick Y axis)
        # Negative because pushing up should move elevator up
        joystick_input = -self.controller.getRightY()
        
        # Apply deadzone
        if abs(joystick_input) < 0.1:
            joystick_input = 0
        
        # Update current position based on joystick input
        # The scale factor determines sensitivity
        self.current_position += joystick_input * self.scale_factor
        
        # Constrain the position to safe limits (you'll need to define these)
        max_height = 40  # This should be the maximum safe rotations value
        min_height = 0   # This should be the minimum safe rotations value
        self.current_position = max(min_height, min(self.current_position, max_height))
        
        # Set the elevator to go to the current position
        self.elevator.gotoPosition(self.current_position)
        
        # Print out debugging information
        if joystick_input != 0:
            print(f"Elevator position: {self.current_position:.2f} rotations " +
                  f"({2 * constants.convert.rot2in(self.current_position):.2f} inches)")
            
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        print(f"Ending elevator joystick control at position: {self.current_position}")
        # Leave the elevator at its current position
        
    def isFinished(self):
        """Returns true when the command should end."""
        return False  # Run until interrupted