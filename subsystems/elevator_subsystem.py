from commands2 import SubsystemBase
from rev import CANSparkMax, CANSparkLowLevel, SparkAbsoluteEncoder, SparkPIDController
from wpilib import SmartDashboard
from constants.constants import ElevatorConstants

class ElevatorSubsystem(SubsystemBase):
    def __init__(self) -> None:
        """Creates a new ElevatorSubsystem."""
        super().__init__()
        
        # Initialize motors
        self.left_motor = CANSparkMax(
            ElevatorConstants.LEFT_MOTOR_ID, 
            CANSparkLowLevel.MotorType.kBrushless
        )
        self.right_motor = CANSparkMax(
            ElevatorConstants.RIGHT_MOTOR_ID, 
            CANSparkLowLevel.MotorType.kBrushless
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
            CANSparkMax.SoftLimitDirection.kForward,
            ElevatorConstants.MAX_HEIGHT
        )
        self.left_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse,
            ElevatorConstants.MIN_HEIGHT
        )
        self.left_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, True)
        self.left_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, True)
        
        # Save configurations
        self.left_motor.burnFlash()
        self.right_motor.burnFlash()
        
        # Initialize target position
        self.target_position = ElevatorConstants.BASE_HEIGHT
    
    def setPosition(self, position: float) -> None:
        """
        Sets the target position of the elevator.
        
        :param position: Target position in meters
        """
        # Clamp position to soft limits
        position = min(max(position, ElevatorConstants.MIN_HEIGHT), 
                      ElevatorConstants.MAX_HEIGHT)
        self.target_position = position
        self.pid_controller.setReference(
            position, 
            CANSparkMax.ControlType.kSmartMotion
        )
    
    def getCurrentPosition(self) -> float:
        """Returns the current position of the elevator in meters."""
        return self.absolute_encoder.getPosition()
    
    def getCurrentVelocity(self) -> float:
        """Returns the current velocity of the elevator in meters per second."""
        return self.absolute_encoder.getVelocity()
    
    def isAtPosition(self) -> bool:
        """Returns whether the elevator is at the target position."""
        current_pos = self.getCurrentPosition()
        current_vel = self.getCurrentVelocity()
        
        return (abs(current_pos - self.target_position) < ElevatorConstants.POSITION_TOLERANCE
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
    
    def periodic(self) -> None:
        """Periodic function that runs every scheduler loop."""
        # Update SmartDashboard with elevator data
        SmartDashboard.putNumber("Elevator Position (m)", self.getCurrentPosition())
        SmartDashboard.putNumber("Elevator Velocity (m/s)", self.getCurrentVelocity())
        SmartDashboard.putNumber("Elevator Target Position (m)", self.target_position)
        SmartDashboard.putBoolean("Elevator At Position", self.isAtPosition())
        SmartDashboard.putNumber("Left Motor Current", self.left_motor.getOutputCurrent())
        SmartDashboard.putNumber("Right Motor Current", self.right_motor.getOutputCurrent()) 