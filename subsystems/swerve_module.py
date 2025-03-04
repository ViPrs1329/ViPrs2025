import math
import rev
import wpilib
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

class SwerveModule:
    """
    A single swerve drive module consisting of a drive motor and a turning motor.
    """
    
    def __init__(
        self,
        drive_motor_id: int,
        turn_motor_id: int,
        cancoder_id: int,
        drive_motor_inverted: bool,
        turn_motor_inverted: bool,
        absolute_encoder_offset: float,
        module_name: str
    ):
        """
        Initialize the swerve module.
        
        :param drive_motor_id: CAN ID of the drive NEO motor
        :param turn_motor_id: CAN ID of the turning NEO motor
        :param cancoder_id: CAN ID of the CANcoder
        :param drive_motor_inverted: Whether to invert the drive motor
        :param turn_motor_inverted: Whether to invert the turn motor
        :param absolute_encoder_offset: Offset of the absolute encoder in rotations
        :param module_name: Name of this module for logging/debugging
        """
        # Create motor controllers
        self.drive_motor = rev.SparkMax(drive_motor_id, rev.SparkMax.MotorType.kBrushless)
        self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkMax.MotorType.kBrushless)
        
        # Create CANcoder
        self.cancoder = CANcoder(cancoder_id)
        
        # Configure motor controllers
        # Note: restoreFactoryDefaults is not needed in 2025, motors start in a known state
        
        self.drive_motor.setInverted(drive_motor_inverted)
        self.turn_motor.setInverted(turn_motor_inverted)
        
        # Get encoders
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getEncoder()
        
        # Configure PID controllers
        self.drive_pid = self.drive_motor.getPIDController()
        self.turn_pid = self.turn_motor.getPIDController()
        
        # PID coefficients for drive motor
        self.drive_pid.setP(0.1)
        self.drive_pid.setI(0)
        self.drive_pid.setD(0)
        self.drive_pid.setFF(0.2)
        
        # PID coefficients for turn motor
        self.turn_pid.setP(1.0)
        self.turn_pid.setI(0)
        self.turn_pid.setD(0)
        
        # Save configuration to motor controllers
        self.drive_motor.burnFlash()
        self.turn_motor.burnFlash()
        
        self.name = module_name
        self.absolute_encoder_offset = absolute_encoder_offset
        
        # Initialize current state
        self.current_state = SwerveModuleState(0, Rotation2d(0))
        
        # Reset the turn encoder based on absolute position
        self.reset_to_absolute()
        
        # Add to SmartDashboard
        self.init_smartdashboard()
    
    def init_smartdashboard(self):
        """Initialize SmartDashboard entries for this module."""
        prefix = f"SwerveModule/{self.name}"
        wpilib.SmartDashboard.putNumber(f"{prefix}/Drive Speed", 0)
        wpilib.SmartDashboard.putNumber(f"{prefix}/Turn Angle", 0)
        
    def get_state(self) -> SwerveModuleState:
        """
        Get the current state of the module.
        
        :return: Current SwerveModuleState
        """
        velocity = self.drive_encoder.getVelocity()
        angle = Rotation2d(self.turn_encoder.getPosition())
        return SwerveModuleState(velocity, angle)
    
    def get_position(self) -> SwerveModulePosition:
        """
        Get the current position of the module.
        
        :return: Current SwerveModulePosition
        """
        position = self.drive_encoder.getPosition()
        angle = Rotation2d(self.turn_encoder.getPosition())
        return SwerveModulePosition(position, angle)
    
    def set_desired_state(self, desired_state: SwerveModuleState, open_loop: bool = False):
        """
        Set the desired state of the module.
        
        :param desired_state: Desired SwerveModuleState
        :param open_loop: Whether to use open-loop control
        """
        # Optimize the state to avoid spinning more than 90 degrees
        optimized_state = SwerveModuleState.optimize(
            desired_state, Rotation2d(self.turn_encoder.getPosition())
        )
        
        # Set the optimized turn angle
        self.turn_pid.setReference(
            optimized_state.angle.radians(),
            rev.SparkMax.ControlType.kPosition
        )
        
        # Set the drive speed
        if open_loop:
            self.drive_motor.set(optimized_state.speed)
        else:
            self.drive_pid.setReference(
                optimized_state.speed,
                rev.SparkMax.ControlType.kVelocity
            )
        
        # Update SmartDashboard
        prefix = f"SwerveModule/{self.name}"
        wpilib.SmartDashboard.putNumber(f"{prefix}/Drive Speed", optimized_state.speed)
        wpilib.SmartDashboard.putNumber(f"{prefix}/Turn Angle", optimized_state.angle.degrees())
        
        self.current_state = optimized_state
    
    def reset_encoders(self):
        """Reset the encoders to zero."""
        self.drive_encoder.setPosition(0)
        self.turn_encoder.setPosition(0)
    
    def reset_to_absolute(self):
        """Reset the turn encoder based on the CANcoder's absolute position."""
        absolute_position = self.cancoder.get_position().value
        adjusted_position = absolute_position - self.absolute_encoder_offset
        self.turn_encoder.setPosition(adjusted_position * 2 * math.pi)  # Convert to radians

    def close(self):
        """Clean up resources when the module is no longer needed."""
        self.drive_motor.close()
        self.turn_motor.close()
        self.cancoder.close() 