"""
Test implementation of the ALGAE subsystem for testing.
"""
import commands2

class TestAlgaeSubsystem(commands2.Subsystem):
    """
    A test implementation of the ALGAE subsystem.
    """
    
    def __init__(self):
        """
        Creates a new test ALGAE subsystem.
        """
        super().__init__()
        
        # Track method calls
        self.set_arm_position_calls = []
        self.set_intake_speed_calls = []
        self.current_arm_position = 0.0
        self.current_intake_speed = 0.0
        
        # Track motor currents
        self.arm_motor_current = 0.0
        self.intake_motor_current = 0.0
        
    def setArmPosition(self, target_degrees: float) -> None:
        """
        Sets the target position for the ALGAE arm.
        
        Parameters
        ----------
        target_degrees : float
            Target position in degrees
        """
        self.current_arm_position = target_degrees
        self.set_arm_position_calls.append(target_degrees)
        
    def getArmPosition(self) -> float:
        """
        Gets the current position of the ALGAE arm.
        
        Returns
        -------
        float
            Current position in degrees
        """
        return self.current_arm_position
        
    def setIntakeSpeed(self, speed: float) -> None:
        """
        Sets the speed of the ALGAE intake wheels.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        # Clamp speed between -1 and 1
        clamped_speed = max(min(speed, 1.0), -1.0)
        self.current_intake_speed = clamped_speed
        self.set_intake_speed_calls.append(clamped_speed)
        
    def stop(self) -> None:
        """
        Stops all motors.
        """
        self.setIntakeSpeed(0)
        self.setArmPosition(self.current_arm_position)  # Keep current position
        
    def atSetpoint(self) -> bool:
        """
        Returns whether the arm has reached its target position.
        
        Returns
        -------
        bool
            True if at setpoint, False otherwise
        """
        return abs(self.current_arm_position - self.set_arm_position_calls[-1]) < 1.0
        
    def isAtBottom(self) -> bool:
        """
        Returns whether the arm is at the bottom position.
        
        Returns
        -------
        bool
            True if at bottom position, False otherwise
        """
        return abs(self.current_arm_position) < 1.0
        
    def set_motor_currents(self, arm_current: float, intake_current: float) -> None:
        """
        Sets the current values for the motors.
        
        Parameters
        ----------
        arm_current : float
            Current for arm motor in amps
        intake_current : float
            Current for intake motor in amps
        """
        self.arm_motor_current = arm_current
        self.intake_motor_current = intake_current 