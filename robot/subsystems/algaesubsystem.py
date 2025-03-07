"""
ALGAE subsystem for the 2025 Reefscape robot.
Handles the ALGAE-specific portion of the End Effector.
"""
import wpilib
import commands2
import rev
from wpimath.controller import PIDController

from constants import (
    ALGAE_ARM_MOTOR_ID,
    ALGAE_INTAKE_MOTOR_ID,
    ALGAE_ARM_REST_ANGLE,
    ALGAE_ARM_WORKING_ANGLE,
    ALGAE_ARM_P,
    ALGAE_ARM_I,
    ALGAE_ARM_D,
    NEO_CURRENT_LIMIT
)

class AlgaeSubsystem(commands2.Subsystem):
    """
    A subsystem that controls the ALGAE handling portion of the End Effector.
    """
    
    def __init__(self):
        """
        Creates a new ALGAE subsystem.
        """
        super().__init__()
        
        # Constants for reset and persist modes
        reset_mode = rev.SparkBase.ResetMode.kNoResetSafeParameters
        persist_mode = rev.SparkBase.PersistMode.kNoPersistParameters

        # Create motors
        self.arm_motor = rev.SparkMax(ALGAE_ARM_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        self.intake_motor = rev.SparkMax(ALGAE_INTAKE_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        
        # Configure motors
        # self.arm_motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        # self.intake_motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        arm_conf = rev.SparkBaseConfig()
        intake_conf = rev.SparkBaseConfig()
        arm_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        intake_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        arm_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)
        intake_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)

        self.arm_motor.configure(arm_conf, reset_mode, persist_mode)
        self.intake_motor.configure(intake_conf, reset_mode, persist_mode)
        
        # Set current limits
        # self.arm_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        # self.intake_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        
        # Create absolute encoder for arm
        self.arm_encoder = self.arm_motor.getAbsoluteEncoder()
        
        # Configure encoder
        # self.arm_encoder.setPositionConversionFactor(1.0)  # Convert to degrees
        # self.arm_encoder.setVelocityConversionFactor(1.0)  # Convert to degrees per second
        
        # Create PID controller for arm position
        self.pid = PIDController(
            ALGAE_ARM_P,
            ALGAE_ARM_I,
            ALGAE_ARM_D
        )
        
        # Set PID tolerance
        self.pid.setTolerance(1.0)  # 1 degree tolerance
        
        # Set initial position
        self.setArmPosition(ALGAE_ARM_REST_ANGLE)
        
        # Add to SmartDashboard
        wpilib.SmartDashboard.putData("ALGAE Arm PID", self.pid)
        
    def periodic(self):
        """
        This method is called periodically by the scheduler.
        """
        # Update SmartDashboard
        wpilib.SmartDashboard.putNumber("ALGAE Arm Position", self.getArmPosition())
        wpilib.SmartDashboard.putNumber("ALGAE Arm Current", self.arm_motor.getOutputCurrent())
        wpilib.SmartDashboard.putNumber("ALGAE Intake Current", self.intake_motor.getOutputCurrent())
        
    def setArmPosition(self, target_degrees: float) -> None:
        """
        Sets the target position for the ALGAE arm.
        
        Parameters
        ----------
        target_degrees : float
            Target position in degrees
        """
        self.pid.setSetpoint(target_degrees)
        
    def getArmPosition(self) -> float:
        """
        Gets the current position of the ALGAE arm.
        
        Returns
        -------
        float
            Current position in degrees
        """
        return self.arm_encoder.getPosition()
    
    def setIntakeSpeed(self, speed: float) -> None:
        """
        Sets the speed of the ALGAE intake wheels.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
            Positive values intake ALGAE
            Negative values eject ALGAE
        """
        self.intake_motor.set(speed)
        
    def stop(self) -> None:
        """
        Stops all motors.
        """
        self.arm_motor.set(0)
        self.intake_motor.set(0)
        
    def atSetpoint(self) -> bool:
        """
        Returns whether the arm has reached its target position.
        
        Returns
        -------
        bool
            True if at setpoint, False otherwise
        """
        return self.pid.atSetpoint()
        
    def isAtBottom(self) -> bool:
        """
        Returns whether the arm is at the bottom position.
        
        Returns
        -------
        bool
            True if at bottom position, False otherwise
        """
        return abs(self.getArmPosition() - ALGAE_ARM_REST_ANGLE) < 1.0 