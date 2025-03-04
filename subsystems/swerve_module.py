import math
from commands2 import SubsystemBase
from rev import SparkMax, SparkLowLevel, SparkBase
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants.constants import DriveConstants
from utils.caching import CachingSubsystemBase

class SwerveModule(CachingSubsystemBase):
    """
    A single swerve drive module consisting of a drive motor and a turning motor.
    """
    
    class Cache(CachingSubsystemBase.Cache):
        """Cache specific to swerve module"""
        def __init__(self):
            super().__init__()
            # Initialize with default values
            self.set_cached("drive_position", 0.0)
            self.set_cached("drive_velocity", 0.0)
            self.set_cached("turn_position", 0.0)
            self.set_cached("turn_velocity", 0.0)
            self.set_cached("drive_current", 0.0)
            self.set_cached("turn_current", 0.0)
            self.set_cached("cancoder_position", 0.0)
            
            # Setpoints
            self.set_setpoint("drive_setpoint", 0.0)
            self.set_setpoint("turn_setpoint", 0.0)
            self.set_setpoint("open_loop", False)
    
    def __init__(
        self,
        drive_motor_id: int,
        turn_motor_id: int,
        cancoder_id: int,
        drive_inverted: bool,
        turn_inverted: bool,
        encoder_offset: float,
        module_name: str
    ):
        """
        Initialize the swerve module.
        
        :param drive_motor_id: CAN ID of the drive NEO motor
        :param turn_motor_id: CAN ID of the turning NEO motor
        :param cancoder_id: CAN ID of the CANcoder
        :param drive_inverted: Whether to invert the drive motor
        :param turn_inverted: Whether to invert the turn motor
        :param encoder_offset: Offset of the absolute encoder in rotations
        :param module_name: Name of this module for logging/debugging
        """
        super().__init__()
        
        # Initialize motors
        self.drive_motor = SparkMax(drive_motor_id, SparkLowLevel.MotorType.kBrushless)
        self.turn_motor = SparkMax(turn_motor_id, SparkLowLevel.MotorType.kBrushless)
        
        # Create CANcoder
        self.cancoder = CANcoder(cancoder_id)
        
        # Configure motor controllers
        self.drive_motor.setInverted(drive_inverted)
        self.turn_motor.setInverted(turn_inverted)
        
        # Get encoders
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getEncoder()
        
        # Configure PID controllers
        self.drive_pid_controller = self.drive_motor.getPIDController()
        self.turn_pid_controller = self.turn_motor.getPIDController()
        
        # PID coefficients for drive motor
        self.drive_pid_controller.setP(0.1)
        self.drive_pid_controller.setI(0)
        self.drive_pid_controller.setD(0)
        self.drive_pid_controller.setFF(0.2)
        
        # PID coefficients for turn motor
        self.turn_pid_controller.setP(1.0)
        self.turn_pid_controller.setI(0)
        self.turn_pid_controller.setD(0)
        
        # Save configuration to motor controllers
        self.drive_motor.burnFlash()
        self.turn_motor.burnFlash()
        
        self.name = module_name
        self.encoder_offset = encoder_offset
        
        # Reset the turn encoder based on absolute position
        self.reset_to_absolute()
    
    def cache_sensors(self) -> None:
        """Cache all sensor values."""
        # Cache drive motor values
        self.cache.set_cached("drive_position", self.drive_encoder.getPosition())
        self.cache.set_cached("drive_velocity", self.drive_encoder.getVelocity())
        self.cache.set_cached("drive_current", self.drive_motor.getOutputCurrent())
        
        # Cache turn motor values
        self.cache.set_cached("turn_position", self.turn_encoder.getPosition())
        self.cache.set_cached("turn_velocity", self.turn_encoder.getVelocity())
        self.cache.set_cached("turn_current", self.turn_motor.getOutputCurrent())
        
        # Cache CANcoder value
        self.cache.set_cached("cancoder_position", self.cancoder.get_position().value)
    
    def update_hardware(self) -> None:
        """Update hardware with cached setpoints."""
        # Update turn motor
        self.turn_pid_controller.setReference(
            self.cache.get_setpoint("turn_setpoint"),
            SparkLowLevel.ControlType.kPosition
        )
        
        # Update drive motor
        if self.cache.get_setpoint("open_loop"):
            self.drive_motor.set(self.cache.get_setpoint("drive_setpoint"))
        else:
            self.drive_pid_controller.setReference(
                self.cache.get_setpoint("drive_setpoint"),
                SparkLowLevel.ControlType.kVelocity
            )
    
    def periodic_logic(self) -> None:
        """Update SmartDashboard with cached values."""
        prefix = f"SwerveModule/{self.name}"
        wpilib.SmartDashboard.putNumber(f"{prefix}/Drive Speed", 
                                      self.cache.get_cached("drive_velocity"))
        wpilib.SmartDashboard.putNumber(f"{prefix}/Turn Angle", 
                                      math.degrees(self.cache.get_cached("turn_position")))
        wpilib.SmartDashboard.putNumber(f"{prefix}/Drive Current", 
                                      self.cache.get_cached("drive_current"))
        wpilib.SmartDashboard.putNumber(f"{prefix}/Turn Current", 
                                      self.cache.get_cached("turn_current"))
    
    def get_state(self) -> SwerveModuleState:
        """
        Get the current state of the module.
        
        :return: Current SwerveModuleState
        """
        velocity = self.cache.get_cached("drive_velocity")
        angle = Rotation2d(self.cache.get_cached("turn_position"))
        return SwerveModuleState(velocity, angle)
    
    def get_position(self) -> SwerveModulePosition:
        """
        Get the current position of the module.
        
        :return: Current SwerveModulePosition
        """
        position = self.cache.get_cached("drive_position")
        angle = Rotation2d(self.cache.get_cached("turn_position"))
        return SwerveModulePosition(position, angle)
    
    def set_desired_state(self, desired_state: SwerveModuleState, open_loop: bool = False):
        """
        Set the desired state of the module.
        
        :param desired_state: Desired SwerveModuleState
        :param open_loop: Whether to use open-loop control
        """
        # Optimize the state to avoid spinning more than 90 degrees
        optimized_state = SwerveModuleState.optimize(
            desired_state, 
            Rotation2d(self.cache.get_cached("turn_position"))
        )
        
        # Set the setpoints
        self.cache.set_setpoint("turn_setpoint", optimized_state.angle.radians())
        self.cache.set_setpoint("drive_setpoint", optimized_state.speed)
        self.cache.set_setpoint("open_loop", open_loop)
    
    def reset_encoders(self):
        """Reset the encoders to zero."""
        self.drive_encoder.setPosition(0)
        self.turn_encoder.setPosition(0)
        self.cache_sensors()  # Update cached values
    
    def reset_to_absolute(self):
        """Reset the turn encoder based on the CANcoder's absolute position."""
        absolute_position = self.cancoder.get_position().value
        adjusted_position = absolute_position - self.encoder_offset
        self.turn_encoder.setPosition(adjusted_position * 2 * math.pi)  # Convert to radians
        self.cache_sensors()  # Update cached values

    def close(self):
        """Clean up resources when the module is no longer needed."""
        self.drive_motor.close()
        self.turn_motor.close()
        self.cancoder.close() 