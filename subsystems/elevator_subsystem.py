from commands2 import SubsystemBase
from rev import SparkFlex, SparkLowLevel, SparkAbsoluteEncoder, SparkBase
from wpilib import SmartDashboard
from constants.constants import ElevatorConstants
from utils.caching import CachingSubsystemBase

class ElevatorSubsystem(CachingSubsystemBase):
    """Subsystem for controlling the elevator mechanism."""
    
    class Cache(CachingSubsystemBase.Cache):
        """Cache specific to elevator subsystem"""
        def __init__(self):
            super().__init__()
            # Initialize with default values
            self.set_cached("position", 0.0)
            self.set_cached("velocity", 0.0)
            self.set_cached("left_current", 0.0)
            self.set_cached("right_current", 0.0)
            self.set_setpoint("target_position", ElevatorConstants.BASE_HEIGHT)
            
    def __init__(self) -> None:
        """Creates a new ElevatorSubsystem."""
        super().__init__()
        
        # Initialize motors
        self.left_motor = SparkFlex(
            ElevatorConstants.LEFT_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        self.right_motor = SparkFlex(
            ElevatorConstants.RIGHT_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        
        # Reset motor controllers to factory defaults
        self.left_motor.restoreFactoryDefaults()
        self.right_motor.restoreFactoryDefaults()
        
        # Set right motor to follow left motor (inverted)
        self.right_motor.follow(self.left_motor, True)
        
        # Get the absolute encoder
        self.absolute_encoder = self.left_motor.getAbsoluteEncoder(
            SparkAbsoluteEncoder.Type.kDutyCycle
        )
        
        # Configure absolute encoder
        self.absolute_encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
        self.absolute_encoder.setVelocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
        
        # Get and configure PID controller
        self.pid_controller = self.left_motor.getPIDController()
        self.pid_controller.setFeedbackDevice(self.absolute_encoder)
        
        # Set PID coefficients
        self.pid_controller.setP(ElevatorConstants.kP)
        self.pid_controller.setI(ElevatorConstants.kI)
        self.pid_controller.setD(ElevatorConstants.kD)
        self.pid_controller.setFF(ElevatorConstants.kFF)
        
        # Set motion profile constraints
        self.pid_controller.setSmartMotionMaxVelocity(ElevatorConstants.MAX_VELOCITY, 0)
        self.pid_controller.setSmartMotionMaxAccel(ElevatorConstants.MAX_ACCELERATION, 0)
        
        # Configure current limits
        self.left_motor.setSmartCurrentLimit(
            ElevatorConstants.CURRENT_LIMIT,
            ElevatorConstants.TRIGGER_THRESHOLD_CURRENT,
            ElevatorConstants.TRIGGER_THRESHOLD_TIME
        )
        self.right_motor.setSmartCurrentLimit(
            ElevatorConstants.CURRENT_LIMIT,
            ElevatorConstants.TRIGGER_THRESHOLD_CURRENT,
            ElevatorConstants.TRIGGER_THRESHOLD_TIME
        )
        
        # Set soft limits
        self.left_motor.setSoftLimit(
            SparkBase.SoftLimitDirection.kForward,
            ElevatorConstants.MAX_HEIGHT
        )
        self.left_motor.setSoftLimit(
            SparkBase.SoftLimitDirection.kReverse,
            ElevatorConstants.MIN_HEIGHT
        )
        self.left_motor.enableSoftLimit(SparkBase.SoftLimitDirection.kForward, True)
        self.left_motor.enableSoftLimit(SparkBase.SoftLimitDirection.kReverse, True)
        
        # Save configurations
        self.left_motor.burnFlash()
        self.right_motor.burnFlash()
    
    def setPosition(self, position: float) -> None:
        """
        Sets the target position of the elevator.
        
        :param position: Target position in meters
        """
        # Clamp position to soft limits
        position = min(max(position, ElevatorConstants.MIN_HEIGHT), 
                      ElevatorConstants.MAX_HEIGHT)
        self.cache.set_setpoint("target_position", position)
    
    def getCurrentPosition(self) -> float:
        """Returns the current position of the elevator in meters."""
        return self.cache.get_cached("position")
    
    def getCurrentVelocity(self) -> float:
        """Returns the current velocity of the elevator in meters per second."""
        return self.cache.get_cached("velocity")
    
    def isAtPosition(self) -> bool:
        """Returns whether the elevator is at the target position."""
        current_pos = self.getCurrentPosition()
        current_vel = self.getCurrentVelocity()
        target_pos = self.cache.get_setpoint("target_position")
        
        return (abs(current_pos - target_pos) < ElevatorConstants.POSITION_TOLERANCE
                and abs(current_vel) < ElevatorConstants.VELOCITY_TOLERANCE)
    
    def goToBase(self) -> None:
        """Moves the elevator to the base position."""
        self.setPosition(ElevatorConstants.BASE_HEIGHT)
    
    def goToL1(self) -> None:
        """Moves the elevator to L1 position."""
        self.setPosition(ElevatorConstants.L1_HEIGHT)
    
    def goToL2(self) -> None:
        """Moves the elevator to L2 position."""
        self.setPosition(ElevatorConstants.L2_HEIGHT)
    
    def goToL3(self) -> None:
        """Moves the elevator to L3 position."""
        self.setPosition(ElevatorConstants.L3_HEIGHT)
    
    def goToL4(self) -> None:
        """Moves the elevator to L4 position."""
        self.setPosition(ElevatorConstants.L4_HEIGHT)
    
    def cache_sensors(self) -> None:
        """Cache all sensor values."""
        self.cache.set_cached("position", self.absolute_encoder.getPosition())
        self.cache.set_cached("velocity", self.absolute_encoder.getVelocity())
        self.cache.set_cached("left_current", self.left_motor.getOutputCurrent())
        self.cache.set_cached("right_current", self.right_motor.getOutputCurrent())
    
    def update_hardware(self) -> None:
        """Update hardware with cached setpoints."""
        target_position = self.cache.get_setpoint("target_position")
        self.pid_controller.setReference(
            target_position, 
            SparkLowLevel.ControlType.kSmartMotion
        )
    
    def periodic_logic(self) -> None:
        """Update SmartDashboard with cached values."""
        SmartDashboard.putNumber("Elevator Position (m)", self.getCurrentPosition())
        SmartDashboard.putNumber("Elevator Velocity (m/s)", self.getCurrentVelocity())
        SmartDashboard.putNumber("Elevator Target Position (m)", 
                                self.cache.get_setpoint("target_position"))
        SmartDashboard.putBoolean("Elevator At Position", self.isAtPosition())
        SmartDashboard.putNumber("Left Motor Current", 
                                self.cache.get_cached("left_current"))
        SmartDashboard.putNumber("Right Motor Current", 
                                self.cache.get_cached("right_current")) 