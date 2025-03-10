import commands2
from subsystems.ElevatorSubsystem import Elevator
from wpilib import XboxController
import ntcore
import constants
import time

class ElevatorLimitFinder(commands2.Command):
    """
    Command to help safely find the elevator's upper and lower limits
    by slowly moving the elevator while monitoring current draw.
    """
    def __init__(self, elevator: Elevator, controller: XboxController, 
                current_threshold: float = 30.0, max_position: float = 100.0):
        """
        Initialize the elevator limit finder command.
        
        Args:
            elevator: The elevator subsystem to control
            controller: The Xbox controller to use for feedback
            current_threshold: Current threshold in amps that indicates a limit (default: 30.0)
            max_position: Maximum rotation position to try (safety limit)
        """
        super().__init__()
        self.elevator = elevator
        self.controller = controller
        self.current_threshold = current_threshold
        self.max_position = max_position
        self.addRequirements(elevator)
        
        # Testing state
        self.finding_upper_limit = True  # Start by finding upper limit
        self.test_complete = False
        self.start_position = 0
        self.current_test_position = 0
        self.found_upper_limit = None
        self.found_lower_limit = None
        self.upper_current = 0
        self.lower_current = 0
        self.step_size = 0.5  # How much to increase/decrease by each step
        self.last_step_time = 0
        self.step_delay = 1.0  # Time between steps in seconds
        
        # Create NetworkTable entries for monitoring
        inst = ntcore.NetworkTableInstance.getDefault()
        self.limit_table = inst.getTable("ElevatorLimits")
        self.upper_limit_entry = self.limit_table.getDoubleTopic("upper_limit_rotation").publish()
        self.lower_limit_entry = self.limit_table.getDoubleTopic("lower_limit_rotation").publish()
        self.upper_limit_inches_entry = self.limit_table.getDoubleTopic("upper_limit_inches").publish()
        self.lower_limit_inches_entry = self.limit_table.getDoubleTopic("lower_limit_inches").publish()
        self.current_test_pos_entry = self.limit_table.getDoubleTopic("test_position").publish()
        self.test_phase_entry = self.limit_table.getStringTopic("test_phase").publish()
        self.current_left_entry = self.limit_table.getDoubleTopic("left_current").publish()
        self.current_right_entry = self.limit_table.getDoubleTopic("right_current").publish()
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.start_position = self.elevator.getElevatorPosition()
        self.current_test_position = self.start_position
        self.finding_upper_limit = True
        self.test_complete = False
        self.found_upper_limit = None
        self.found_lower_limit = None
        self.upper_current = 0
        self.lower_current = 0
        self.last_step_time = time.time()
        
        print("=== Elevator Limit Finder Started ===")
        print(f"Starting position: {self.start_position:.2f} rotations")
        print("Finding upper limit first...")
        self.test_phase_entry.set("Finding Upper Limit")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        # Get current measurements
        left_current = self.elevator.LEM.getOutputCurrent()
        right_current = self.elevator.REM.getOutputCurrent()
        current_max = max(left_current, right_current)
        
        # Update NetworkTables
        self.current_left_entry.set(left_current)
        self.current_right_entry.set(right_current)
        self.current_test_pos_entry.set(self.current_test_position)
        
        # If we've completed testing both directions
        if self.test_complete:
            # Just display the results
            return
            
        # Check if it's time for the next step
        current_time = time.time()
        if current_time - self.last_step_time < self.step_delay:
            return
            
        # Check if we've hit a current limit
        if current_max >= self.current_threshold:
            if self.finding_upper_limit:
                self.found_upper_limit = self.current_test_position
                self.upper_current = current_max
                self.upper_limit_entry.set(self.found_upper_limit)
                self.upper_limit_inches_entry.set(2 * constants.convert.rot2in(self.found_upper_limit))
                
                print(f"⚡ Upper limit found at {self.found_upper_limit:.2f} rotations")
                print(f"  Current: {current_max:.1f} amps")
                print(f"  Height: {2 * constants.convert.rot2in(self.found_upper_limit):.2f} inches")
                print("Now finding lower limit...")
                
                # Switch to finding lower limit
                self.finding_upper_limit = False
                self.current_test_position = self.start_position
                self.elevator.gotoPosition(self.current_test_position)
                self.last_step_time = current_time
                self.test_phase_entry.set("Finding Lower Limit")
                
                # Give haptic feedback
                self.controller.setRumble(self.controller.RumbleType.kBothRumble, 1.0)
                return
                
            else:  # Finding lower limit
                self.found_lower_limit = self.current_test_position
                self.lower_current = current_max
                self.lower_limit_entry.set(self.found_lower_limit)
                self.lower_limit_inches_entry.set(2 * constants.convert.rot2in(self.found_lower_limit))
                
                print(f"⚡ Lower limit found at {self.found_lower_limit:.2f} rotations")
                print(f"  Current: {current_max:.1f} amps")
                print(f"  Height: {2 * constants.convert.rot2in(self.found_lower_limit):.2f} inches")
                print("=== Testing Complete ===")
                
                # Complete the test
                self.test_complete = True
                self.test_phase_entry.set("Test Complete")
                
                # Return to a safe middle position
                safe_position = (self.found_upper_limit + self.found_lower_limit) / 2
                self.elevator.gotoPosition(safe_position)
                
                # Give haptic feedback
                self.controller.setRumble(self.controller.RumbleType.kBothRumble, 1.0)
                return
                
        # If we haven't found a limit yet, continue stepping
        if self.finding_upper_limit:
            # Step upward, but respect safety limit
            self.current_test_position += self.step_size
            if self.current_test_position > self.max_position:
                print(f"⚠️ Reached safety limit of {self.max_position} rotations without finding upper limit")
                self.finding_upper_limit = False
                self.current_test_position = self.start_position
                self.test_phase_entry.set("Finding Lower Limit")
        else:
            # Step downward, but don't go below zero
            self.current_test_position -= self.step_size
            if self.current_test_position < 0:
                self.current_test_position = 0
                print("⚠️ Reached 0 rotation position without finding significant current increase")
                self.found_lower_limit = 0
                self.lower_limit_entry.set(self.found_lower_limit)
                self.lower_limit_inches_entry.set(0)
                self.test_complete = True
                self.test_phase_entry.set("Test Complete")
                
                # Return to a safe position
                safe_position = self.found_upper_limit / 2 if self.found_upper_limit is not None else self.start_position
                self.elevator.gotoPosition(safe_position)
                return
        
        # Set the elevator to the new test position
        self.elevator.gotoPosition(self.current_test_position)
        
        # Print status update
        print(f"Testing position: {self.current_test_position:.2f} rotations | " +
              f"Current: L={left_current:.1f}A R={right_current:.1f}A")
              
        # Update the last step time
        self.last_step_time = current_time
        
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Turn off controller rumble
        self.controller.setRumble(self.controller.RumbleType.kBothRumble, 0)
        
        if interrupted:
            print("⚠️ Elevator limit finding was interrupted!")
            # Return to starting position
            self.elevator.gotoPosition(self.start_position)
        
        if self.found_upper_limit is not None and self.found_lower_limit is not None:
            print("\n=== Elevator Limit Summary ===")
            print(f"Upper limit: {self.found_upper_limit:.2f} rotations " +
                  f"({2 * constants.convert.rot2in(self.found_upper_limit):.2f} inches)")
            print(f"Lower limit: {self.found_lower_limit:.2f} rotations " +
                  f"({2 * constants.convert.rot2in(self.found_lower_limit):.2f} inches)")
            print(f"Safe range: {self.found_upper_limit - self.found_lower_limit:.2f} rotations " +
                  f"({2 * constants.convert.rot2in(self.found_upper_limit - self.found_lower_limit):.2f} inches)")
            print("================================")
        
    def isFinished(self):
        """Returns true when the command should end."""
        return self.test_complete