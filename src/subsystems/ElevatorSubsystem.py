# src/subsystems/ElevatorSubsystem.py
import rev
import math
import commands2
import wpilib
from wpimath import controller

from constants import CANIDs, elevatorConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory
from subsystems.BaseSubsystem import BaseSubsystem

class Elevator(BaseSubsystem):
    """
    Elevator subsystem for a two-stage cascade elevator mechanism.
    
    This class manages the elevator's motors, sensors, and control logic
    to handle precise positioning and manual control.
    """
    
    class Cache:
        """Cache for storing sensor values to reduce CAN bus traffic."""
        def __init__(self):
            # Cached sensor values
            self.left_position = 0.0
            self.right_position = 0.0
            self.absolute_position = 0.0
            self.left_velocity = 0.0
            self.right_velocity = 0.0
            
            # Motor current values (read less frequently)
            self.left_current = 0.0
            self.right_current = 0.0
            
            # Motor temperature values (read least frequently)
            self.left_temp = 0.0
            self.right_temp = 0.0
            
            # Counter for less frequent reads
            self.current_counter = 0
            self.temp_counter = 0
    
    def __init__(self):
        """Initialize the elevator subsystem."""
        super().__init__("Elevator")
        
        try:
            # Initialize cache
            self.cache = self.Cache()
            
            # Create motor configurations
            left_config = SparkMaxFactory.Configuration()
            left_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            left_config.current_limit = elevatorConsts.currentLimit
            left_config.voltage_comp_enabled = True
            left_config.voltage_comp_saturation = 12.0
            
            # PID values for position control
            left_config.kP = elevatorConsts.kP
            left_config.kI = elevatorConsts.kI
            left_config.kD = elevatorConsts.kD
            left_config.kF = elevatorConsts.kF
            
            # Create follower configuration (right side follows left but inverted)
            right_config = SparkMaxFactory.Configuration()
            right_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            right_config.current_limit = elevatorConsts.currentLimit
            right_config.voltage_comp_enabled = True
            right_config.voltage_comp_saturation = 12.0
            right_config.inverted = True  # Invert follower
            
            # Create motors
            self.left_motor = SparkMaxFactory.createSparkMax(CANIDs.ElevatorLeftID, left_config)
            self.right_motor = SparkMaxFactory.createSparkMax(CANIDs.ElevatorRightID, right_config)
            
            # Set up the right motor to follow the left
            self.right_motor.follow(self.left_motor, True)  # Follow with inversion
            
            # Get encoders
            self.left_encoder = self.left_motor.getEncoder()
            self.right_encoder = self.right_motor.getEncoder()
            
            # Try to get absolute encoder if available
            try:
                self.absolute_encoder = self.left_motor.getAbsoluteEncoder(
                    rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
                )
                
                # Configure absolute encoder
                self.absolute_encoder.setPositionConversionFactor(
                    elevatorConsts.POSITION_CONVERSION_FACTOR
                )
                self.absolute_encoder.setZeroOffset(
                    elevatorConsts.ABSOLUTE_ENCODER_OFFSET if hasattr(elevatorConsts, 'ABSOLUTE_ENCODER_OFFSET') else 0.0
                )
                
                # Use absolute encoder for feedback if available
                self.use_absolute_encoder = True
                print("Using absolute encoder for elevator position feedback")
                
                # Configure built-in PID to use absolute encoder
                self.motor_pid_controller = self.left_motor.getPIDController()
                self.motor_pid_controller.setFeedbackDevice(self.absolute_encoder)
                
                # Set PID values
                self.motor_pid_controller.setP(elevatorConsts.kP)
                self.motor_pid_controller.setI(elevatorConsts.kI)
                self.motor_pid_controller.setD(elevatorConsts.kD)
                self.motor_pid_controller.setFF(elevatorConsts.kF)
                
            except Exception as e:
                # Fall back to relative encoder if absolute not available
                print(f"Warning: Could not initialize absolute encoder, falling back to relative: {e}")
                self.use_absolute_encoder = False
                
                # Set up conversion factor for relative encoder
                self.left_encoder.setPositionConversionFactor(
                    elevatorConsts.POSITION_CONVERSION_FACTOR
                )
                self.right_encoder.setPositionConversionFactor(
                    elevatorConsts.POSITION_CONVERSION_FACTOR
                )
                
                # Create WPILib PID controller as fallback
                self.position_pid_controller = controller.PIDController(
                    elevatorConsts.kP, elevatorConsts.kI, elevatorConsts.kD
                )
            
            # Soft limits for safety
            self.min_height = elevatorConsts.MIN_HEIGHT
            self.max_height = elevatorConsts.MAX_HEIGHT
            
            # Position tolerance
            self.position_tolerance = elevatorConsts.POSITION_TOLERANCE
            
            # Current target position
            self.target_position = 0.0
            
            # Initialize default position
            self.resetPosition()
            
            # Cache initial sensor values
            self.cacheSensors()
            
            # Initialize dashboard values
            wpilib.SmartDashboard.putNumber("Elevator/Position", 0.0)
            wpilib.SmartDashboard.putNumber("Elevator/Target", 0.0)
            wpilib.SmartDashboard.putString("Elevator/Status", "Ready")
            
        except Exception as e:
            self.handleError("__init__", e)
    
    def cacheSensors(self):
        """Cache sensor values to reduce CAN bus traffic."""
        try:
            # Always cache positions
            self.cache.left_position = self.left_encoder.getPosition()
            self.cache.right_position = self.right_encoder.getPosition()
            
            if self.use_absolute_encoder:
                self.cache.absolute_position = self.absolute_encoder.getPosition()
            else:
                # When no absolute encoder, use left encoder as the source of truth
                self.cache.absolute_position = self.cache.left_position
            
            # Always cache velocities
            self.cache.left_velocity = self.left_encoder.getVelocity()
            self.cache.right_velocity = self.right_encoder.getVelocity()
            
            # Cache current every 5 iterations
            if self.cache.current_counter == 0:
                self.cache.left_current = self.left_motor.getOutputCurrent()
                self.cache.right_current = self.right_motor.getOutputCurrent()
                
            self.cache.current_counter = (self.cache.current_counter + 1) % 5
            
            # Cache temperature every 20 iterations
            if self.cache.temp_counter == 0:
                self.cache.left_temp = self.left_motor.getMotorTemperature()
                self.cache.right_temp = self.right_motor.getMotorTemperature()
                
            self.cache.temp_counter = (self.cache.temp_counter + 1) % 20
            
        except Exception as e:
            self.handleError("cacheSensors", e)
    
    def subsystemPeriodic(self):
        """Periodic code for the elevator subsystem."""
        try:
            # Update dashboard with current position and status
            wpilib.SmartDashboard.putNumber("Elevator/Position", self.getCurrentPosition())
            wpilib.SmartDashboard.putNumber("Elevator/Current", self.cache.left_current)
            wpilib.SmartDashboard.putNumber("Elevator/Temperature", self.cache.left_temp)
            
            # Check for safety issues
            if self.checkSafety():
                # If safety issue detected, stop motors
                self.stopMotors()
            
        except Exception as e:
            self.handleError("subsystemPeriodic", e)
    
    def checkSafety(self):
        """
        Check for safety issues such as overcurrent or overtemperature.
        
        Returns:
            bool: True if safety issue detected
        """
        # Current threshold
        if (self.cache.left_current > elevatorConsts.CURRENT_LIMIT_THRESHOLD or 
            self.cache.right_current > elevatorConsts.CURRENT_LIMIT_THRESHOLD):
            wpilib.SmartDashboard.putString("Elevator/Status", "Overcurrent")
            return True
        
        # Temperature threshold
        if (self.cache.left_temp > elevatorConsts.TEMP_LIMIT_THRESHOLD or 
            self.cache.right_temp > elevatorConsts.TEMP_LIMIT_THRESHOLD):
            wpilib.SmartDashboard.putString("Elevator/Status", "Overtemperature")
            return True
        
        # Current position outside soft limits
        current_pos = self.getCurrentPosition()
        if current_pos < self.min_height or current_pos > self.max_height:
            wpilib.SmartDashboard.putString("Elevator/Status", "Outside Limits")
            return True
        
        return False
    
    def getCurrentPosition(self):
        """Get the current position of the elevator."""
        if self.use_absolute_encoder:
            return self.cache.absolute_position
        else:
            return self.cache.left_position
    
    def resetPosition(self, position=0.0):
        """Reset the elevator position to a specific value."""
        try:
            if self.use_absolute_encoder:
                # Can't "reset" an absolute encoder, but we can adjust the offset
                # Not typically needed as the absolute encoder has a fixed frame of reference
                pass
            else:
                # Reset the relative encoder
                self.left_encoder.setPosition(position)
                self.right_encoder.setPosition(position)
            
            print(f"Elevator position reset to {position}")
            
        except Exception as e:
            self.handleError("resetPosition", e)
    
    def moveToPosition(self, position):
        """
        Move the elevator to a specified position.
        
        Args:
            position (float): Target position in elevator units
        """
        try:
            # Clamp position to soft limits
            position = max(min(position, self.max_height), self.min_height)
            
            # Update target position
            self.target_position = position
            wpilib.SmartDashboard.putNumber("Elevator/Target", position)
            
            # If using built-in PID with absolute encoder
            if self.use_absolute_encoder and hasattr(self, 'motor_pid_controller'):
                self.motor_pid_controller.setReference(
                    position, 
                    rev.CANSparkMax.ControlType.kPosition
                )
            else:
                # Using WPILib PID controller
                current_position = self.getCurrentPosition()
                output = self.position_pid_controller.calculate(
                    current_position, position
                )
                
                # Apply gravity feedforward if configured
                if hasattr(elevatorConsts, 'kG'):
                    output += elevatorConsts.kG
                
                # Clamp output to valid range
                output = max(min(output, 1.0), -1.0)
                
                # Set motor output
                self.left_motor.set(output)
                
            wpilib.SmartDashboard.putString("Elevator/Status", "Moving")
            
        except Exception as e:
            self.handleError("moveToPosition", e)
            self.stopMotors()
    
    def isAtPosition(self, position=None, tolerance=None):
        """
        Check if the elevator is at the target position.
        
        Args:
            position (float, optional): Position to check against. Defaults to current target.
            tolerance (float, optional): Position tolerance. Defaults to configured tolerance.
            
        Returns:
            bool: True if at position
        """
        if position is None:
            position = self.target_position
            
        if tolerance is None:
            tolerance = self.position_tolerance
            
        current_position = self.getCurrentPosition()
        return abs(current_position - position) <= tolerance
    
    def holdPosition(self):
        """Hold the elevator at its current position."""
        current_position = self.getCurrentPosition()
        self.moveToPosition(current_position)
        wpilib.SmartDashboard.putString("Elevator/Status", "Holding")
    
    def setManualSpeed(self, speed):
        """
        Set the elevator speed manually.
        
        Args:
            speed (float): Speed to set (-1.0 to 1.0)
        """
        try:
            # Check if at soft limits
            current_position = self.getCurrentPosition()
            
            # Prevent movement beyond soft limits
            if (current_position <= self.min_height and speed < 0) or \
               (current_position >= self.max_height and speed > 0):
                speed = 0
                wpilib.SmartDashboard.putString("Elevator/Status", "At Limit")
            else:
                wpilib.SmartDashboard.putString("Elevator/Status", "Manual Control")
            
            # Apply gravity feedforward if configured
            if hasattr(elevatorConsts, 'kG') and speed != 0:
                speed += math.copysign(elevatorConsts.kG, speed)
                
                # Re-clamp to valid range
                speed = max(min(speed, 1.0), -1.0)
            
            # Set motor output
            self.left_motor.set(speed)
            
        except Exception as e:
            self.handleError("setManualSpeed", e)
            self.stopMotors()
    
    def stopMotors(self):
        """Stop all elevator motors."""
        try:
            self.left_motor.set(0)
            # Right motor follows left, so no need to stop it separately
            wpilib.SmartDashboard.putString("Elevator/Status", "Stopped")
        except Exception as e:
            self.handleError("stopMotors", e)
    
    def subsystemSimulationPeriodic(self):
        """Periodic simulation code for the elevator."""
        if self.is_simulation:
            # In simulation, we would update position based on motor speeds
            # This could involve simple physics simulation for the elevator
            pass