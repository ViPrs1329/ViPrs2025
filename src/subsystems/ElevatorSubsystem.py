# src/subsystems/ElevatorSubsystem.py
import rev
import math
import commands2
import wpilib
import wpimath
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
        
        # Initialize default values first to avoid AttributeError if initialization fails
        self.left_encoder = None
        self.right_encoder = None
        self.use_absolute_encoder = False
        self.absolute_encoder = None
        self.min_height = elevatorConsts.MIN_HEIGHT
        self.max_height = elevatorConsts.MAX_HEIGHT
        self.position_tolerance = elevatorConsts.POSITION_TOLERANCE
        self.target_position = 0.0
        self.left_motor = None
        self.right_motor = None
        
        try:
            # Initialize cache
            self.cache = self.Cache()
            
            # Create motor configurations
            left_config = SparkMaxFactory.Configuration()
            if wpilib.RobotBase.isSimulation():
                left_config.idle_mode = rev.SparkMax.IdleMode.kBrake
            else:
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
            if wpilib.RobotBase.isSimulation():
                right_config.idle_mode = rev.SparkMax.IdleMode.kBrake
            else:
                right_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            right_config.current_limit = elevatorConsts.currentLimit
            right_config.voltage_comp_enabled = True
            right_config.voltage_comp_saturation = 12.0
            right_config.inverted = True  # Invert follower
            
            # Create motors
            self.left_motor = SparkMaxFactory.createSparkMax(CANIDs.ElevatorLeftID, left_config)
            self.right_motor = SparkMaxFactory.createSparkMax(CANIDs.ElevatorRightID, right_config)
            
            # Set up the right motor to follow the left
            if self.right_motor and self.left_motor:
                try:
                    self.right_motor.follow(self.left_motor, True)  # Follow with inversion
                except Exception as e:
                    print(f"Warning: Could not set up follower mode: {e}")
            
            # Initialize simulation-specific attributes
            self.is_simulation = wpilib.RobotBase.isSimulation()
            
            # Initialize encoder attributes - create these before accessing them
            if self.left_motor:
                try:
                    if self.is_simulation:
                        # In simulation, try to use the simulation helper if available
                        from team254.LazySparkMax import SimSparkMaxAbsoluteEncoder
                        self.left_encoder = SimSparkMaxAbsoluteEncoder(self.left_motor)
                        self.right_encoder = SimSparkMaxAbsoluteEncoder(self.right_motor) if self.right_motor else None
                    else:
                        # On real hardware, get the actual encoders
                        self.left_encoder = self.left_motor.getEncoder()
                        self.right_encoder = self.right_motor.getEncoder() if self.right_motor else None
                        
                        # Try to get absolute encoder if available
                        try:
                            self.absolute_encoder = self.left_motor.getAbsoluteEncoder(
                                rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
                            )
                            self.use_absolute_encoder = True
                        except Exception as e:
                            print(f"Warning: Could not initialize absolute encoder, falling back to relative: {e}")
                            self.use_absolute_encoder = False
                            self.absolute_encoder = None
                except Exception as e:
                    print(f"Warning: Could not initialize encoders: {e}")
            
            # Set up conversion factors for relative encoders
            if not self.is_simulation and self.left_encoder:
                try:
                    self.left_encoder.setPositionConversionFactor(
                        elevatorConsts.POSITION_CONVERSION_FACTOR
                    )
                    if self.right_encoder:
                        self.right_encoder.setPositionConversionFactor(
                            elevatorConsts.POSITION_CONVERSION_FACTOR
                        )
                except Exception as e:
                    print(f"Warning: Could not set conversion factors: {e}")
            
            # Configure built-in PID to use absolute encoder
            if self.use_absolute_encoder and self.left_motor:
                try:
                    self.motor_pid_controller = self.left_motor.getPIDController()
                    self.motor_pid_controller.setFeedbackDevice(self.absolute_encoder)
                    
                    # Set PID values
                    self.motor_pid_controller.setP(elevatorConsts.kP)
                    self.motor_pid_controller.setI(elevatorConsts.kI)
                    self.motor_pid_controller.setD(elevatorConsts.kD)
                    self.motor_pid_controller.setFF(elevatorConsts.kF)
                except Exception as e:
                    print(f"Warning: Could not configure PID controller: {e}")
            
            # Initialize WPILib PID controller for position control
            self.position_pid_controller = controller.PIDController(
                elevatorConsts.kP,
                elevatorConsts.kI,
                elevatorConsts.kD
            )
            
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
            # Make sure we still have valid defaults even if initialization fails
            if not hasattr(self, 'left_encoder') or self.left_encoder is None:
                print("Warning: Elevator encoders not initialized properly, using dummy values")
                # Create dummy encoder implementation for simulation
                class DummyEncoder:
                    def getPosition(self): return 0.0
                    def getVelocity(self): return 0.0
                    def setPosition(self, pos): pass
                
                self.left_encoder = DummyEncoder()
                self.right_encoder = DummyEncoder()
    
    def cacheSensors(self):
        """Cache sensor values to reduce CAN bus traffic."""
        try:
            # Always cache positions if encoders are available
            if hasattr(self, 'left_encoder') and self.left_encoder:
                self.cache.left_position = self.left_encoder.getPosition()
            
            if hasattr(self, 'right_encoder') and self.right_encoder:
                self.cache.right_position = self.right_encoder.getPosition()
            
            if hasattr(self, 'use_absolute_encoder') and self.use_absolute_encoder and hasattr(self, 'absolute_encoder') and self.absolute_encoder:
                self.cache.absolute_position = self.absolute_encoder.getPosition()
            else:
                # When no absolute encoder, use left encoder as the source of truth
                self.cache.absolute_position = self.cache.left_position
            
            # Always cache velocities if encoders are available
            if hasattr(self, 'left_encoder') and self.left_encoder:
                self.cache.left_velocity = self.left_encoder.getVelocity()
            
            if hasattr(self, 'right_encoder') and self.right_encoder:
                self.cache.right_velocity = self.right_encoder.getVelocity()
            
            # Cache current every 5 iterations if motors are available
            if self.cache.current_counter == 0:
                if hasattr(self, 'left_motor') and self.left_motor:
                    self.cache.left_current = self.left_motor.getOutputCurrent()
                
                if hasattr(self, 'right_motor') and self.right_motor:
                    self.cache.right_current = self.right_motor.getOutputCurrent()
                
            self.cache.current_counter = (self.cache.current_counter + 1) % 5
            
            # Cache temperature every 20 iterations if motors are available
            if self.cache.temp_counter == 0:
                if hasattr(self, 'left_motor') and self.left_motor:
                    self.cache.left_temp = self.left_motor.getMotorTemperature()
                
                if hasattr(self, 'right_motor') and self.right_motor:
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
        # Make sure we check if attributes exist
        if hasattr(self, 'use_absolute_encoder') and self.use_absolute_encoder and hasattr(self, 'absolute_encoder') and self.absolute_encoder:
            return self.cache.absolute_position
        else:
            return self.cache.left_position
    
    def resetPosition(self, position=0.0):
        """Reset the elevator position to a specific value."""
        try:
            if hasattr(self, 'use_absolute_encoder') and self.use_absolute_encoder and hasattr(self, 'absolute_encoder') and self.absolute_encoder:
                # Can't "reset" an absolute encoder, but we can adjust the offset
                # Not typically needed as the absolute encoder has a fixed frame of reference
                pass
            else:
                # Reset the relative encoder if available
                if hasattr(self, 'left_encoder') and self.left_encoder:
                    self.left_encoder.setPosition(position)
                
                if hasattr(self, 'right_encoder') and self.right_encoder:
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
            if hasattr(self, 'use_absolute_encoder') and self.use_absolute_encoder and hasattr(self, 'motor_pid_controller') and self.motor_pid_controller:
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
                
                # Set motor output if available
                if hasattr(self, 'left_motor') and self.left_motor:
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
        try:
            # Get current position
            current_position = self.getCurrentPosition()
            
            # Move to current position (this will maintain position)
            self.moveToPosition(current_position)
            
        except Exception as e:
            self.handleError("holdPosition", e)
            self.stopMotors()
    
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
            
            # Set motor output if available
            if hasattr(self, 'left_motor') and self.left_motor:
                self.left_motor.set(speed)
            
        except Exception as e:
            self.handleError("setManualSpeed", e)
            self.stopMotors()
    
    def stopMotors(self):
        """Stop all elevator motors."""
        try:
            if hasattr(self, 'left_motor') and self.left_motor:
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