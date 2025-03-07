"""
Test-specific implementation of the ElevatorSubsystem.
"""
import commands2
from typing import Optional

class TestElevatorSubsystem(commands2.Subsystem):
    """
    A test-specific implementation of the elevator subsystem that doesn't use any hardware.
    """
    
    # Scoring heights in inches
    SCORING_HEIGHTS = {
        "BASE": 25.36,
        "L1": 18.0,
        "L2": 31.875,
        "L3": 47.625,
        "L4": 72.0
    }
    
    def __init__(self):
        """
        Creates a new test elevator subsystem with simulated components.
        """
        super().__init__()
        
        # Simulated state
        self.current_position = self.SCORING_HEIGHTS["BASE"]
        self.target_position = self.SCORING_HEIGHTS["BASE"]
        self.current_speed = 0.0
        self.motor1_current = 10.0
        self.motor2_current = 10.0
        
        # Track method calls for testing
        self.set_position_calls = []
        self.set_speed_calls = []
        self.at_setpoint_return_value = False
        
    def periodic(self):
        """
        This method is called periodically by the scheduler.
        """
        pass
        
    def setPosition(self, target_inches: float) -> None:
        """
        Sets the target position for the elevator.
        
        Parameters
        ----------
        target_inches : float
            Target position in inches
        """
        self.target_position = target_inches
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
    
    def getCurrent(self) -> float:
        """
        Gets the current draw of the elevator motors.
        
        Returns
        -------
        float
            Current draw in amps
        """
        return (self.motor1_current + self.motor2_current) / 2
    
    def atSetpoint(self) -> bool:
        """
        Returns whether the elevator has reached its target position.
        
        Returns
        -------
        bool
            True if at setpoint, False otherwise
        """
        return self.at_setpoint_return_value
    
    def stop(self) -> None:
        """
        Stops the elevator.
        """
        self.current_speed = 0.0
        self.set_speed_calls.append(0.0)
        
    def setSpeed(self, speed: float) -> None:
        """
        Sets the speed of the elevator motors directly.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        self.current_speed = speed
        self.set_speed_calls.append(speed)
        
    # Test-specific methods
    def set_motor_currents(self, motor1_current: float, motor2_current: float) -> None:
        """
        Sets the simulated motor currents for testing.
        """
        self.motor1_current = motor1_current
        self.motor2_current = motor2_current
        
    def set_current_position(self, position: float) -> None:
        """
        Sets the simulated current position for testing.
        """
        self.current_position = position
        
    def set_at_setpoint(self, at_setpoint: bool) -> None:
        """
        Sets the return value for atSetpoint() for testing.
        """
        self.at_setpoint_return_value = at_setpoint