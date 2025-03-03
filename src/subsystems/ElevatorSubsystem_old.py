# ElevatorSubsystem.py
import rev
import math
import commands2
import wpilib
from wpimath import controller

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from constants import CANIDs, elevatorConsts
from team254.LazySparkMax import LazySparkMax, LazySparkFlex
from team254.SparkMaxFactory import SparkMaxFactory

class Elevator(commands2.Subsystem):
    class Cache:
        def __init__(self):
            # Cached sensor values
            self.leftPosition = 0.0
            self.rightPosition = 0.0
            self.leftAbsolutePosition = 0.0
            self.leftVelocity = 0.0
            self.rightVelocity = 0.0
            self.leftCurrent = 0.0
            self.rightCurrent = 0.0
            self.call_counters = {"current": 0, "temperature": 0}
            
    def __init__(self) -> None:
        super().__init__()

        # Initialize cache
        self.cache = self.Cache()

        # Check if we're in simulation mode
        self.is_simulation = wpilib.RobotBase.isSimulation()

        # Create motor configurations
        elevator_config = SparkMaxFactory.Configuration()
        elevator_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        elevator_config.current_limit = elevatorConsts.currentLimit
        elevator_config.voltage_comp_enabled = True
        
        # PID values from constants
        elevator_config.kP = elevatorConsts.kP if hasattr(elevatorConsts, 'kP') else 0.1
        elevator_config.kI = elevatorConsts.kI if hasattr(elevatorConsts, 'kI') else 0.0
        elevator_config.kD = elevatorConsts.kD if hasattr(elevatorConsts, 'kD') else 0.0
        elevator_config.kF = elevatorConsts.kF if hasattr(elevatorConsts, 'kF') else 0.0
        
        # Initialize motors using factory
        self.LEM = SparkMaxFactory.createSparkFlex(CANIDs.ElevatorLeftID, elevator_config)
        
        # Create follower configuration
        follower_config = SparkMaxFactory.Configuration()
        # TODO: CANSparkMax doesn't exist any more. Check here: https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
        follower_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        follower_config.current_limit = elevatorConsts.currentLimit
        follower_config.follow_leader = self.LEM
        follower_config.follow_invert = True  # Invert if needed for your mechanical setup
        
        self.REM = SparkMaxFactory.createSparkFlex(CANIDs.ElevatorRightID, follower_config)
        
        # Get encoder for position feedback
        # TODO: getEncoder() isn't being recognized, most likely because of the LazySpark implementation. Check here https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
        self.leftEncoder = self.LEM.getEncoder()
        self.rightEncoder = self.REM.getEncoder()

        # Try to get the absolute encoder attached to the left SparkFlex
        try:
            # TODO: Can't find rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle , check here https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
            self.leftAbsoluteEncoder = self.LEM.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)

            # Configure the absolute encoder
            # Set the zero offset based on your mechanical setup
            offset = elevatorConsts.ABSOLUTE_ENCODER_OFFSET if hasattr(elevatorConsts, 'ABSOLUTE_ENCODER_OFFSET') else 0.0
            self.leftAbsoluteEncoder.setZeroOffset(offset)

            # Set whether the absolute encoder is inverted
            inverted = elevatorConsts.ABSOLUTE_ENCODER_INVERTED if hasattr(elevatorConsts, 'ABSOLUTE_ENCODER_INVERTED') else False
            self.leftAbsoluteEncoder.setInverted(inverted)

            # Set position conversion factor (convert rotations to meaningful units)
            conversion_factor = elevatorConsts.POSITION_CONVERSION_FACTOR if hasattr(elevatorConsts, 'POSITION_CONVERSION_FACTOR') else 0.1
            
            try:
                self.leftAbsoluteEncoder.setPositionConversionFactor(conversion_factor)
                # TODO: setPositionConversionFactor doesn't seem to exist. Check here https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
                self.leftEncoder.setPositionConversionFactor(conversion_factor)
                self.rightEncoder.setPositionConversionFactor(conversion_factor)
            except Exception as e:
                print(f"Warning: Could not set position conversion factor: {e}")
            
            # Use absolute encoder for position feedback
            self.use_absolute = True
        except Exception as e:
            print(f"Warning: Could not initialize absolute encoder: {e}")
            # Fall back to relative encoder
            self.use_absolute = False
            
            # For simulation, provide a simulated absolute encoder that just returns the relative encoder value
            if self.is_simulation:
                print("Creating simulated absolute encoder")
                from team254.LazySparkMax import SimSparkMaxAbsoluteEncoder
                self.leftAbsoluteEncoder = SimSparkMaxAbsoluteEncoder(self.LEM)
        
        # Create PID controller for position control using WPILib PID
        self.kP = elevatorConsts.kP if hasattr(elevatorConsts, 'kP') else 0.1
        self.kI = elevatorConsts.kI if hasattr(elevatorConsts, 'kI') else 0.0
        self.kD = elevatorConsts.kD if hasattr(elevatorConsts, 'kD') else 0.0
        self.kF = elevatorConsts.kF if hasattr(elevatorConsts, 'kF') else 0.0
        
        self.pid_controller = controller.PIDController(self.kP, self.kI, self.kD)
        
        # Try to configure built-in PID controller to use absolute encoder
        try:
            self.leftPID = self.LEM.getPIDController()
            
            if self.use_absolute:
                # TODO: setFeedbackDevice() doesn't seem to exist. Check here: https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
                self.leftPID.setFeedbackDevice(self.leftAbsoluteEncoder)
        except Exception as e:
            print(f"Warning: Could not configure built-in PID controller: {e}")
        
        # Create soft limit variables with defaults
        self.min_height = elevatorConsts.MIN_HEIGHT if hasattr(elevatorConsts, 'MIN_HEIGHT') else 0.0
        self.max_height = elevatorConsts.MAX_HEIGHT if hasattr(elevatorConsts, 'MAX_HEIGHT') else 100.0
            
        # Default position tolerance
        self.position_tolerance = elevatorConsts.POSITION_TOLERANCE if hasattr(elevatorConsts, 'POSITION_TOLERANCE') else 1.0
            
        # Initialize current target position
        self.target_position = 0.0
        
        # Initialize dashboard values
        wpilib.SmartDashboard.putNumber("Elevator Position", 0.0)
        wpilib.SmartDashboard.putNumber("Elevator Target", 0.0)
        wpilib.SmartDashboard.putString("Elevator Status", "Initialized")
        
        # Cache sensor values initially
        self.cacheSensors()

    def cacheSensors(self):
        """Cache sensor values to reduce CAN bus traffic"""
        try:
            # Always cache position and velocity values
            self.cache.leftPosition = self.leftEncoder.getPosition()
            self.cache.rightPosition = self.rightEncoder.getPosition()
            
            if hasattr(self, 'leftAbsoluteEncoder'):
                self.cache.leftAbsolutePosition = self.leftAbsoluteEncoder.getPosition()
            else:
                self.cache.leftAbsolutePosition = self.cache.leftPosition
                
            self.cache.leftVelocity = self.leftEncoder.getVelocity()
            self.cache.rightVelocity = self.rightEncoder.getVelocity()
            
            # Cache current readings less frequently
            if self.cache.call_counters["current"] == 0:
                self.cache.leftCurrent = self.LEM.getOutputCurrent()
                self.cache.rightCurrent = self.REM.getOutputCurrent()
            self.cache.call_counters["current"] = (self.cache.call_counters["current"] + 1) % 10
        except Exception as e:
            if not self.is_simulation:
                print(f"Error caching sensor values: {e}")

    def periodic(self):
        """Called periodically during all robot modes."""
        # Update sensor cache
        self.cacheSensors()
        
        # Update dashboard with current position
        wpilib.SmartDashboard.putNumber("Elevator Position", self.getCurrentPosition())
        wpilib.SmartDashboard.putNumber("Elevator Current", self.cache.leftCurrent)

    def getCurrentPosition(self):
        """Get the current position of the elevator.
        
        Returns:
            float: Current position in meters based on absolute encoder.
        """
        # Use cached value instead of direct sensor read
        if self.use_absolute and hasattr(self, 'leftAbsoluteEncoder'):
            return self.cache.leftAbsolutePosition
        else:
            return self.cache.leftPosition

    def moveToPosition(self, position):
        """Move the elevator to the specified position.
        
        Args:
            position (float): Target position in encoder counts or converted units.
        """
        # Clamp position to soft limits
        position = min(max(position, self.min_height), self.max_height)
        
        # Store target position
        self.target_position = position
        
        try:
            # Option 1: Use WPILib PID controller (software PID)
            current_position = self.getCurrentPosition()
            pid_output = self.pid_controller.calculate(current_position, position)
            
            # Add feedforward if available
            if self.kF != 0:
                pid_output += self.kF * math.copysign(1.0, position - current_position)
            
            # Limit output to valid motor input range
            pid_output = min(max(pid_output, -1.0), 1.0)
            
            # Set motor output
            self.LEM.set(pid_output)
            
            # Option 2: Use built-in SparkMax PID controller (more efficient)
            # Only use if it's properly configured
            """
            if hasattr(self, 'leftPID') and self.use_absolute:
                try:
                    self.leftPID.setReference(position, rev.CANSparkMax.ControlType.kPosition)
                except Exception as e:
                    print(f"Warning: Could not use built-in PID, falling back to software PID: {e}")
                    self.LEM.set(pid_output)
            else:
                self.LEM.set(pid_output)
            """
            
        except Exception as e:
            print(f"Error moving elevator to position: {e}")
            self.stopMotors()

    def isAtPosition(self, position, tolerance=None):
        """Check if the elevator is at the target position within tolerance.
        
        Args:
            position (float): Target position to check against.
            tolerance (float, optional): Position tolerance. Defaults to self.position_tolerance.
            
        Returns:
            bool: True if the elevator is at the target position within tolerance.
        """
        if tolerance is None:
            tolerance = self.position_tolerance
            
        current_position = self.getCurrentPosition()
        return abs(current_position - position) <= tolerance

    def holdPosition(self):
        """Hold the elevator at its current position."""
        current_position = self.getCurrentPosition()
        self.moveToPosition(current_position)

    def stopMotors(self):
        """Stop all elevator motors."""
        try:
            self.LEM.set(0)
            # No need to set REM since it's a follower
        except Exception as e:
            print(f"Error stopping elevator motors: {e}")

    def setManualSpeed(self, speed):
        """Set the elevator speed manually.
        
        Args:
            speed (float): Speed to set (-1.0 to 1.0).
        """
        # Check if the elevator is at a soft limit
        current_position = self.getCurrentPosition()
        
        # Prevent movement past soft limits
        if (current_position <= self.min_height and speed < 0) or \
           (current_position >= self.max_height and speed > 0):
            speed = 0
        
        try:
            self.LEM.set(speed)
        except Exception as e:
            print(f"Error setting elevator speed: {e}")
            self.stopMotors()

    def resetEncoder(self, position=0.0):
        """Reset the elevator encoder to a specific position.
        
        Args:
            position (float, optional): Position to reset to. Defaults to 0.0.
        """
        try:
            # TODO: setPosition() doesn't seem to exist. Check here: https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
            self.leftEncoder.setPosition(position)
        except Exception as e:
            print(f"Error resetting elevator encoder: {e}")