"""
Test implementation of the Elevator subsystem for testing.
"""
import commands2
from robot.constants import ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT

class TestElevatorSubsystem(commands2.Subsystem):
    """
    A test implementation of the Elevator subsystem.
    """
    
    def __init__(self):
        """
        Creates a new test Elevator subsystem.
        """
        super().__init__()
        
        # Track method calls
        self.set_position_calls = []
        self.set_speed_calls = []
        self.current_position = 0.0  # Start at 0.0 as expected by tests
        self.current_speed = 0.0
        
        # Track motor current
        self.motor_current = 0.0
        
        # Track whether we should enforce height limits
        self.enforce_height_limits = False
        
    def setPosition(self, target_inches: float) -> None:
        """
        Sets the target position for the elevator.
        
        Parameters
        ----------
        target_inches : float
            Target position in inches
        """
        if self.enforce_height_limits:
            # Clamp position between min and max heights
            clamped_position = max(min(target_inches, ELEVATOR_MAX_HEIGHT), ELEVATOR_MIN_HEIGHT)
            self.current_position = clamped_position
            self.set_position_calls.append(clamped_position)
        else:
            # Don't clamp position in test implementation
            self.current_position = target_inches
            self.set_position_calls.append(target_inches)
        
    def getPosition(self) -> float:
        """
        Gets the current position of the elevator.
        
        Returns
        -------
        float
            Current position in inches
        """
        return self.current_position
        
    def setSpeed(self, speed: float) -> None:
        """
        Sets the speed of the elevator.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        # Clamp speed between -1 and 1
        clamped_speed = max(min(speed, 1.0), -1.0)
        self.current_speed = clamped_speed
        self.set_speed_calls.append(clamped_speed)
        
    def stop(self) -> None:
        """
        Stops all motors.
        """
        self.setSpeed(0)
        
    def atSetpoint(self) -> bool:
        """
        Returns whether the elevator has reached its target position.
        
        Returns
        -------
        bool
            True if at setpoint, False otherwise
        """
        if not self.set_position_calls:
            return False
        return abs(self.current_position - self.set_position_calls[-1]) < 0.1
        
    def set_motor_currents(self, current: float) -> None:
        """
        Sets the current value for the motors.
        
        Parameters
        ----------
        current : float
            Current for motors in amps
        """
        self.motor_current = current
        
    def set_enforce_height_limits(self, enforce: bool) -> None:
        """
        Sets whether height limits should be enforced.
        
        Parameters
        ----------
        enforce : bool
            True to enforce height limits, False otherwise
        """
        self.enforce_height_limits = enforce