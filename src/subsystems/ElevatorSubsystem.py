# ElevatorSubsystem.py
import rev
import math
import commands2
import wpilib
from wpimath import controller

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

import constants

class Elevator(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Check if we're in simulation mode
        self.is_simulation = wpilib.RobotBase.isSimulation()

        # Motor initialization
        self.LEM = rev.SparkFlex(constants.CANIDs.ElevatorLeftID, rev.SparkFlex.MotorType.kBrushless)
        self.REM = rev.SparkFlex(constants.CANIDs.ElevatorRightID, rev.SparkFlex.MotorType.kBrushless)

        # Set configurations
        self.LEMConfig = rev.SparkBaseConfig()
        self.LEMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.LEMConfig.smartCurrentLimit(constants.elevatorConsts.currentLimit)
        self.REMConfig = rev.SparkBaseConfig()
        self.REMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.REMConfig.smartCurrentLimit(constants.elevatorConsts.currentLimit)

        # Configure motors
        self.LEM.configure(self.LEMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.REM.configure(self.REMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        
        # Set the right motor to follow the left, with inverted direction
        self.REM.follow(self.LEM, invert=True)
        
        # Get encoder for position feedback
        self.leftEncoder = self.LEM.getEncoder()
        self.rightEncoder = self.REM.getEncoder()
        
        # Create PID controller for position control
        # These gains will need to be tuned for your specific elevator
        # The gains below are placeholders
        if hasattr(constants.elevatorConsts, 'kP'):
            self.kP = constants.elevatorConsts.kP
            self.kI = constants.elevatorConsts.kI
            self.kD = constants.elevatorConsts.kD
            self.kF = constants.elevatorConsts.kF
        else:
            # Default PID values if not in constants
            self.kP = 0.1
            self.kI = 0.0
            self.kD = 0.0
            self.kF = 0.0
        
        self.pid_controller = controller.PIDController(self.kP, self.kI, self.kD)
        
        # Create soft limit variables with defaults
        if hasattr(constants.elevatorConsts, 'MIN_HEIGHT'):
            self.min_height = constants.elevatorConsts.MIN_HEIGHT
        else:
            self.min_height = 0.0
            
        if hasattr(constants.elevatorConsts, 'MAX_HEIGHT'):
            self.max_height = constants.elevatorConsts.MAX_HEIGHT
        else:
            self.max_height = 100.0  # Default max height
            
        # Default position tolerance
        if hasattr(constants.elevatorConsts, 'POSITION_TOLERANCE'):
            self.position_tolerance = constants.elevatorConsts.POSITION_TOLERANCE
        else:
            self.position_tolerance = 1.0  # Default tolerance
            
        # Initialize current target position
        self.target_position = 0.0
        
        # Initialize dashboard values
        wpilib.SmartDashboard.putNumber("Elevator Position", 0.0)
        wpilib.SmartDashboard.putNumber("Elevator Target", 0.0)
        wpilib.SmartDashboard.putString("Elevator Status", "Initialized")

    def periodic(self):
        """Called periodically during all robot modes."""
        # Update dashboard with current position
        current_position = self.getCurrentPosition()
        wpilib.SmartDashboard.putNumber("Elevator Position", current_position)
        
        # Add any other periodic tasks

    def getCurrentPosition(self):
        """Get the current position of the elevator.
        
        Returns:
            float: Current position in encoder counts or converted units.
        """
        try:
            return self.encoder.getPosition()
        except Exception as e:
            print(f"Error getting elevator position: {e}")
            return 0.0

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
            # Get current position
            current_position = self.getCurrentPosition()
            
            # Calculate PID output
            pid_output = self.pid_controller.calculate(current_position, position)
            
            # Add feedforward if available
            if hasattr(self, 'kF') and self.kF != 0:
                pid_output += self.kF * math.copysign(1.0, position - current_position)
            
            # Limit output to valid motor input range
            pid_output = min(max(pid_output, -1.0), 1.0)
            
            # Set motor output
            self.LEM.set(pid_output)
            
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
            self.encoder.setPosition(position)
        except Exception as e:
            print(f"Error resetting elevator encoder: {e}")