import commands2
from subsystems.ElevatorSubsystem import Elevator
from wpilib import XboxController
import constants
import ntcore

class JoystickElevatorControl(commands2.Command):
    """
    Command to control the elevator using a joystick for debugging purposes.
    This allows for manual control of the elevator position using an Xbox controller's right stick.
    """
    def __init__(self, elevator: Elevator, controller: XboxController, scale_factor: float = 0.5, 
                 current_threshold: float = 30.0, rumble_threshold: float = 25.0):
        """
        Initialize the joystick elevator control command.
        
        Args:
            elevator: The elevator subsystem to control
            controller: The Xbox controller to use for input
            scale_factor: How much to scale the joystick input by (default: 0.5)
            current_threshold: The current threshold in amps that indicates the elevator has reached a limit (default: 30.0)
            rumble_threshold: The current threshold in amps to start controller rumble as a warning (default: 25.0)
        """
        super().__init__()
        self.elevator = elevator
        self.controller = controller
        self.scale_factor = scale_factor
        self.current_threshold = current_threshold
        self.rumble_threshold = rumble_threshold
        self.addRequirements(elevator)
        
        # Save the original position when starting joystick control
        self.starting_position = 0
        self.current_position = 0
        
        # For current monitoring
        self.last_current_left = 0
        self.last_current_right = 0
        self.peak_current = 0
        self.at_limit = False
        
        # Create NetworkTable entries for monitoring
        inst = ntcore.NetworkTableInstance.getDefault()
        self.elevator_table = inst.getTable("ElevatorDebug")
        self.left_current_entry = self.elevator_table.getDoubleTopic("left_motor_current").publish()
        self.right_current_entry = self.elevator_table.getDoubleTopic("right_motor_current").publish()
        self.peak_current_entry = self.elevator_table.getDoubleTopic("peak_current").publish()
        self.at_limit_entry = self.elevator_table.getBooleanTopic("at_current_limit").publish()
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.starting_position = self.elevator.getElevatorPosition()
        self.current_position = self.starting_position
        print(f"Starting elevator joystick control at position: {self.starting_position}")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        # Get current measurements from both motors
        left_current = self.elevator.LEM.getOutputCurrent()
        right_current = self.elevator.REM.getOutputCurrent()
        
        # Update peak current if necessary
        current_max = max(left_current, right_current)
        if current_max > self.peak_current:
            self.peak_current = current_max
        
        # Publish current values to NetworkTables
        self.left_current_entry.set(left_current)
        self.right_current_entry.set(right_current)
        self.peak_current_entry.set(self.peak_current)
        
        # Check if we're at a current limit
        self.at_limit = current_max >= self.current_threshold
        self.at_limit_entry.set(self.at_limit)
        
        # Get joystick input (using right stick Y axis)
        # Negative because pushing up should move elevator up
        joystick_input = -self.controller.getRightY()
        
        # Apply deadzone
        if abs(joystick_input) < 0.1:
            joystick_input = 0
        
        # If we're at the current limit, only allow motion in the opposite direction
        if self.at_limit:
            # Determine direction of movement (positive = up, negative = down)
            movement_direction = 1 if joystick_input > 0 else -1
            # Determine which way we hit the limit (looking at current trend)
            limit_direction = 1 if self.current_position > self.starting_position else -1
            
            # Only allow movement away from the limit
            if movement_direction == limit_direction:
                print(f"⚠️ CURRENT LIMIT REACHED: {current_max:.1f} amps - Movement restricted")
                joystick_input = 0
        
        # Apply rumble feedback as warning when approaching current threshold
        if current_max >= self.rumble_threshold:
            # Calculate rumble intensity based on how close we are to the threshold
            intensity = min(1.0, (current_max - self.rumble_threshold) / 
                          (self.current_threshold - self.rumble_threshold))
            self.controller.setRumble(self.controller.RumbleType.kBothRumble, intensity)
        else:
            self.controller.setRumble(self.controller.RumbleType.kBothRumble, 0)
        
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
        if joystick_input != 0 or current_max > 5.0:  # Also print when significant current is detected
            print(f"Elevator: pos={self.current_position:.2f} rot " +
                  f"({2 * constants.convert.rot2in(self.current_position):.2f} in) | " +
                  f"Current: L={left_current:.1f}A R={right_current:.1f}A | " +
                  f"Peak: {self.peak_current:.1f}A")
            
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Turn off controller rumble
        self.controller.setRumble(self.controller.RumbleType.kBothRumble, 0)
        
        print(f"Ending elevator joystick control at position: {self.current_position}")
        print(f"Peak current during operation: {self.peak_current:.1f} amps")
        
        # Leave the elevator at its current position
        
    def isFinished(self):
        """Returns true when the command should end."""
        return False  # Run until interrupted