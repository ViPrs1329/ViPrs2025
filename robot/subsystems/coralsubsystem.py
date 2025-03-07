"""
CORAL subsystem for the 2025 Reefscape robot.
Handles the CORAL-specific portion of the End Effector.
"""
import wpilib
import commands2
import rev
from wpimath.controller import PIDController

from constants import (
    CORAL_LEFT_MOTOR_ID,
    CORAL_RIGHT_MOTOR_ID,
    CORAL_P,
    CORAL_I,
    CORAL_D,
    NEO_CURRENT_LIMIT
)

class CoralSubsystem(commands2.Subsystem):
    """
    A subsystem that controls the CORAL handling portion of the End Effector.
    """
    
    def __init__(self):
        """
        Creates a new CORAL subsystem.
        """
        super().__init__()
        
        # Constants for reset and persist modes
        reset_mode = rev.SparkBase.ResetMode.kNoResetSafeParameters
        persist_mode = rev.SparkBase.PersistMode.kNoPersistParameters

        # Create motors
        self.left_motor = rev.SparkMax(CORAL_LEFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        self.right_motor = rev.SparkMax(CORAL_RIGHT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        
        # Configure motors
        # self.left_motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        # self.right_motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        left_conf = rev.SparkBaseConfig()
        right_conf = rev.SparkBaseConfig()
        left_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        right_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        left_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)
        right_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)

        self.left_motor.configure(left_conf, reset_mode, persist_mode)
        self.right_motor.configure(right_conf, reset_mode, persist_mode)
        
        # Set current limits
        # self.left_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        # self.right_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        
        # Create encoders
        self.left_encoder = self.left_motor.getEncoder()
        self.right_encoder = self.right_motor.getEncoder()
        
        # Create PID controller for speed control
        self.pid = PIDController(
            CORAL_P,
            CORAL_I,
            CORAL_D
        )
        
        # Set PID tolerance
        self.pid.setTolerance(0.1)  # 0.1 RPM tolerance
        
        # Add to SmartDashboard
        wpilib.SmartDashboard.putData("CORAL PID", self.pid)
        
    def periodic(self):
        """
        This method is called periodically by the scheduler.
        """
        # Update SmartDashboard
        wpilib.SmartDashboard.putNumber("CORAL Left Speed", self.left_encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("CORAL Right Speed", self.right_encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("CORAL Current", self.getCurrent())
        
    def setSpeed(self, speed: float) -> None:
        """
        Sets the speed of the CORAL wheels.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
            Positive values intake CORAL
            Negative values eject CORAL
        """
        self.left_motor.set(speed)
        self.right_motor.set(speed)
        
    def getCurrent(self) -> float:
        """
        Gets the current draw of the CORAL motors.
        
        Returns
        -------
        float
            Current draw in amps
        """
        return (self.left_motor.getOutputCurrent() + self.right_motor.getOutputCurrent()) / 2
        
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
        return self.getCurrent() > NEO_CURRENT_LIMIT * 0.8  # 80% of current limit 