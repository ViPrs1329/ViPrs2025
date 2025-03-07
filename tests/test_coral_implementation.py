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
        self.set_speed_calls = []
        self.current_speed = 0.0
        
        # Track motor currents
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        
    def setSpeed(self, speed: float) -> None:
        """
        Sets the speed of the CORAL wheels.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        self.current_speed = speed
        self.set_speed_calls.append(speed)
        
    def getCurrent(self) -> float:
        """
        Gets the current draw of the CORAL motors.
        
        Returns
        -------
        float
            Current draw in amps
        """
        return (self.left_motor_current + self.right_motor_current) / 2
        
    def stop(self) -> None:
        """
        Stops the CORAL wheels.
        """
        self.setSpeed(0)
        
    def isStalled(self) -> bool:
        """
        Checks if the CORAL wheels are stalled based on current draw.
        
        Returns
        -------
        bool
            True if stalled, False otherwise
        """
        return self.getCurrent() > 32  # 80% of 40A limit
        
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