"""
Test implementation of the CORAL subsystem for testing.
"""
import commands2

class TestCoralSubsystem(commands2.Subsystem):
    """
    A test implementation of the CORAL subsystem.
    """
    
    def __init__(self):
        """
        Creates a new test CORAL subsystem.
        """
        super().__init__()
        
        # Track method calls
        self.set_left_speed_calls = []
        self.set_right_speed_calls = []
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        
        # Track motor currents
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        
    def setLeftSpeed(self, speed: float) -> None:
        """
        Sets the speed of the left CORAL wheel.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        # Clamp speed between -1 and 1
        clamped_speed = max(min(speed, 1.0), -1.0)
        self.current_left_speed = clamped_speed
        self.set_left_speed_calls.append(clamped_speed)
        
    def setRightSpeed(self, speed: float) -> None:
        """
        Sets the speed of the right CORAL wheel.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        # Clamp speed between -1 and 1
        clamped_speed = max(min(speed, 1.0), -1.0)
        self.current_right_speed = clamped_speed
        self.set_right_speed_calls.append(clamped_speed)
        
    def stop(self) -> None:
        """
        Stops all motors.
        """
        self.setLeftSpeed(0)
        self.setRightSpeed(0)
        
    def set_motor_currents(self, left_current: float, right_current: float) -> None:
        """
        Sets the current values for the motors.
        
        Parameters
        ----------
        left_current : float
            Current for left motor in amps
        right_current : float
            Current for right motor in amps
        """
        self.left_motor_current = left_current
        self.right_motor_current = right_current
        
    def getCurrent(self) -> float:
        """
        Gets the average current of both motors.
        
        Returns
        -------
        float
            Average current in amps
        """
        return (self.left_motor_current + self.right_motor_current) / 2.0
        
    def isStalled(self) -> bool:
        """
        Returns whether the motors are stalled based on current draw.
        
        Returns
        -------
        bool
            True if stalled, False otherwise
        """
        return self.getCurrent() > 30.0  # Stall threshold of 30 amps 