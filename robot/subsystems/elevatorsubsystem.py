"""
Elevator subsystem for the 2025 Reefscape robot.
"""
import wpilib
import commands2
import rev
from wpimath.controller import PIDController
from wpimath.units import inchesToMeters

from robot.constants import (
    LEFT_ELEVATOR_MOTOR_ID,
    RIGHT_ELEVATOR_MOTOR_ID,
    ELEVATOR_MIN_HEIGHT,
    ELEVATOR_L1_HEIGHT,
    ELEVATOR_L2_HEIGHT,
    ELEVATOR_L3_HEIGHT,
    ELEVATOR_L4_HEIGHT,
    ELEVATOR_MAX_HEIGHT,
    ELEVATOR_P,
    ELEVATOR_I,
    ELEVATOR_D,
    NEO_VORTEX_CURRENT_LIMIT
)

class ElevatorSubsystem(commands2.Subsystem):
    """
    A subsystem that controls the cascading elevator.
    """
    
    # Scoring heights in inches
    SCORING_HEIGHTS = {
        "BASE": ELEVATOR_MIN_HEIGHT,
        "L1": ELEVATOR_L1_HEIGHT,
        "L2": ELEVATOR_L2_HEIGHT,
        "L3": ELEVATOR_L3_HEIGHT,
        "L4": ELEVATOR_L4_HEIGHT
    }
    
    def __init__(self):
        """
        Creates a new elevator subsystem.
        """
        super().__init__()
        
        # Create motors
        self.motor1 = rev.SparkFlex(LEFT_ELEVATOR_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor2 = rev.SparkFlex(RIGHT_ELEVATOR_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)
        
        # Configure motors
        self.motor1.setIdleMode(rev.SparkFlex.IdleMode.kBrake)
        self.motor2.setIdleMode(rev.SparkFlex.IdleMode.kBrake)
        
        # Set current limits
        self.motor1.setSmartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT)
        self.motor2.setSmartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT)
        
        # Create absolute encoder
        self.encoder = self.motor1.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        
        # Configure encoder
        self.encoder.setPositionConversionFactor(1.0)  # Convert to inches
        self.encoder.setVelocityConversionFactor(1.0)  # Convert to inches per second
        
        # Create PID controller
        self.pid = PIDController(
            ELEVATOR_P,
            ELEVATOR_I,
            ELEVATOR_D
        )
        
        # Set PID tolerance
        self.pid.setTolerance(0.1)  # 0.1 inch tolerance
        
        # Set initial position
        self.setPosition(self.SCORING_HEIGHTS["BASE"])
        
        # Add to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator PID", self.pid)
        
    def periodic(self):
        """
        This method is called periodically by the scheduler.
        """
        # Update SmartDashboard
        wpilib.SmartDashboard.putNumber("Elevator Position", self.getPosition())
        wpilib.SmartDashboard.putNumber("Elevator Current", self.getCurrent())
        
    def setPosition(self, target_inches: float) -> None:
        """
        Sets the target position for the elevator.
        
        Parameters
        ----------
        target_inches : float
            Target position in inches
        """
        # Convert to meters for PID controller
        target_meters = inchesToMeters(target_inches)
        self.pid.setSetpoint(target_meters)
        
    def getPosition(self) -> float:
        """
        Gets the current position of the elevator.
        
        Returns
        -------
        float
            Current position in inches
        """
        return self.encoder.getPosition()
    
    def getCurrent(self) -> float:
        """
        Gets the current draw of the elevator motors.
        
        Returns
        -------
        float
            Current draw in amps
        """
        return (self.motor1.getOutputCurrent() + self.motor2.getOutputCurrent()) / 2
    
    def atSetpoint(self) -> bool:
        """
        Returns whether the elevator has reached its target position.
        
        Returns
        -------
        bool
            True if at setpoint, False otherwise
        """
        return self.pid.atSetpoint()
    
    def stop(self) -> None:
        """
        Stops the elevator.
        """
        self.motor1.set(0)
        self.motor2.set(0)
        
    def setSpeed(self, speed: float) -> None:
        """
        Sets the speed of the elevator motors directly.
        
        Parameters
        ----------
        speed : float
            Speed value between -1 and 1
        """
        self.motor1.set(speed)
        self.motor2.set(speed) 