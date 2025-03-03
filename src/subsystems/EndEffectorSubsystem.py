# src/subsystems/EndEffectorSubsystem.py
import rev
import math
import commands2
import wpilib
from wpimath import controller

from constants import CANIDs, endEffectorConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory
from subsystems.BaseSubsystem import BaseSubsystem
from phoenix6.hardware import CANcoder

class EndEffector(BaseSubsystem):
    """
    End effector subsystem for manipulating game pieces.
    
    This subsystem handles both Coral and Algae game pieces with 
    specific mechanisms for each.
    """
    
    class Cache:
        """Cache for storing sensor values to reduce CAN bus traffic."""
        def __init__(self):
            # Coral mechanism
            self.coral_entry_distance = 8000  # mm
            self.coral_entry_status = 0
            self.coral_stop_distance = 8000  # mm
            self.coral_stop_status = 0
            self.coral_left_current = 0.0
            self.coral_right_current = 0.0
            
            # Algae mechanism
            self.algae_position = 0.0
            self.algae_velocity = 0.0
            self.algae_rotation_current = 0.0
            self.algae_intake_current = 0.0
            
            # Motor temperature values (read least frequently)
            self.motor_temps = {
                "CoralLeft": 30.0,
                "CoralRight": 30.0,
                "AlgaeRotation": 30.0,
                "AlgaeIntake": 30.0
            }
            
            # Counters for less frequent reads
            self.current_counter = 0
            self.temp_counter = 0
    
    def __init__(self):
        """Initialize the end effector subsystem."""
        super().__init__("EndEffector")
        
        try:
            # Initialize cache
            self.cache = self.Cache()
            
            # ============ Coral Mechanism ============
            # Create motor configurations
            coral_left_config = SparkMaxFactory.Configuration()
            coral_left_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            coral_left_config.current_limit = endEffectorConsts.coralCurrentLimit
            coral_left_config.voltage_comp_enabled = True
            coral_left_config.inverted = False
            
            coral_right_config = SparkMaxFactory.Configuration()
            coral_right_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            coral_right_config.current_limit = endEffectorConsts.coralCurrentLimit
            coral_right_config.voltage_comp_enabled = True
            coral_right_config.inverted = True  # Typically inverted for opposing wheels
            
            # Create motors
            self.coral_left_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EECoralLeftID, coral_left_config
            )
            self.coral_right_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EECoralRightID, coral_right_config
            )
            
            # Initialize LaserCAN sensors
            try:
                from phoenix6.hardware import CANrange
                self.coral_entry_sensor = CANrange(CANIDs.EECoralInSensorID)
                self.coral_stop_sensor = CANrange(CANIDs.EECoralStopSensorID)
            except Exception as e:
                print(f"Warning: Could not initialize LaserCAN sensors: {e}")
                if self.is_simulation:
                    # Create simulation sensors
                    self.coral_entry_sensor = self._create_sim_sensor()
                    self.coral_stop_sensor = self._create_sim_sensor()
                    print("Using simulated distance sensors")
            
            # ============ Algae Mechanism ============
            # Create motor configurations
            algae_rotation_config = SparkMaxFactory.Configuration()
            algae_rotation_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            algae_rotation_config.current_limit = endEffectorConsts.algaeRotCurrentLimit
            algae_rotation_config.voltage_comp_enabled = True
            
            # PID values for rotation control
            algae_rotation_config.kP = endEffectorConsts.ALGAE_KP
            algae_rotation_config.kI = endEffectorConsts.ALGAE_KI
            algae_rotation_config.kD = endEffectorConsts.ALGAE_KD
            algae_rotation_config.kF = endEffectorConsts.ALGAE_KF
            
            algae_intake_config = SparkMaxFactory.Configuration()
            algae_intake_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            algae_intake_config.current_limit = endEffectorConsts.algaeIntakeCurrentLimit
            algae_intake_config.voltage_comp_enabled = True
            
            # Create motors
            self.algae_rotation_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EEAlgaeArmRotationID, algae_rotation_config
            )
            self.algae_intake_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EEAlgaeIntakeID, algae_intake_config
            )
            
            # Get encoder for algae rotation
            self.algae_rotation_encoder = self.algae_rotation_motor.getEncoder()
            
            # Try to get absolute encoder for algae rotation
            try:
                self.algae_abs_encoder = self.algae_rotation_motor.getAbsoluteEncoder(
                    rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
                )
                
                # Configure absolute encoder
                self.algae_abs_encoder.setPositionConversionFactor(360.0)  # Convert to degrees
                self.algae_abs_encoder.setZeroOffset(
                    endEffectorConsts.ALGAE_ENCODER_OFFSET if hasattr(endEffectorConsts, 'ALGAE_ENCODER_OFFSET') else 0.0
                )
                
                # Use absolute encoder for PID control
                self.algae_pid_controller = self.algae_rotation_motor.getPIDController()
                self.algae_pid_controller.setFeedbackDevice(self.algae_abs_encoder)
                
                # Set PID values
                self.algae_pid_controller.setP(endEffectorConsts.ALGAE_KP)
                self.algae_pid_controller.setI(endEffectorConsts.ALGAE_KI)
                self.algae_pid_controller.setD(endEffectorConsts.ALGAE_KD)
                self.algae_pid_controller.setFF(endEffectorConsts.ALGAE_KF)
                
                self.use_abs_encoder = True
                print("Using absolute encoder for algae rotation")
                
            except Exception as e:
                print(f"Warning: Could not initialize absolute encoder for algae rotation: {e}")
                self.use_abs_encoder = False
                
                # Fall back to relative encoder
                self.algae_rotation_encoder.setPositionConversionFactor(360.0 / 49.0)  # Convert to degrees with 49:1 gearbox
                
                # Create WPILib PID controller as fallback
                self.algae_pid_controller = controller.PIDController(
                    endEffectorConsts.ALGAE_KP,
                    endEffectorConsts.ALGAE_KI, 
                    endEffectorConsts.ALGAE_KD
                )
            
            # Define preset positions
            self.ALGAE_RETRACTED_POS = endEffectorConsts.ALGAE_RETRACTED_POS
            self.ALGAE_TOP_PICKUP_POS = endEffectorConsts.ALGAE_TOP_PICKUP_POS
            self.ALGAE_BOTTOM_PICKUP_POS = endEffectorConsts.ALGAE_BOTTOM_PICKUP_POS
            
            # Position tolerance
            self.ALGAE_POSITION_TOLERANCE = endEffectorConsts.ALGAE_POSITION_TOLERANCE
            
            # Cache initial sensor values
            self.cacheSensors()
            
            # Initialize dashboard values
            wpilib.SmartDashboard.putBoolean("EndEffector/CoralDetected", False)
            wpilib.SmartDashboard.putBoolean("EndEffector/CoralPositioned", False)
            wpilib.SmartDashboard.putNumber("EndEffector/AlgaePosition", 0.0)
            wpilib.SmartDashboard.putString("EndEffector/Status", "Ready")
            
        except Exception as e:
            self.handleError("__init__", e)
    
    def _create_sim_sensor(self):
        """Create a simulated distance sensor."""
        class SimDistanceSensor:
            def __init__(self):
                self.distance = 8000  # 8000mm (nothing detected)
                self.status = 0  # Good status
                
            def get_measurement(self):
                return self.distance, self.status
                
            def set_simulated_distance(self, distance, status=0):
                self.distance = distance
                self.status = status
                
            def is_object_detected(self, threshold=100):
                return self.distance < threshold
        
        return SimDistanceSensor()
    
    def cacheSensors(self):
        """Cache sensor values to reduce CAN bus traffic."""
        try:
            # Cache LaserCAN sensor readings
            try:
                measurement = self.coral_entry_sensor.get_measurement()
                if measurement:
                    self.cache.coral_entry_distance, self.cache.coral_entry_status = measurement
                    
                measurement = self.coral_stop_sensor.get_measurement()
                if measurement:
                    self.cache.coral_stop_distance, self.cache.coral_stop_status = measurement
            except Exception as e:
                if not self.is_simulation:
                    print(f"Warning: Error reading distance sensors: {e}")
            
            # Cache algae encoder position
            if self.use_abs_encoder:
                self.cache.algae_position = self.algae_abs_encoder.getPosition()
            else:
                self.cache.algae_position = self.algae_rotation_encoder.getPosition()
                
            self.cache.algae_velocity = self.algae_rotation_encoder.getVelocity()
            
            # Cache motor currents (less frequently)
            if self.cache.current_counter == 0:
                self.cache.coral_left_current = self.coral_left_motor.getOutputCurrent()
                self.cache.coral_right_current = self.coral_right_motor.getOutputCurrent()
                self.cache.algae_rotation_current = self.algae_rotation_motor.getOutputCurrent()
                self.cache.algae_intake_current = self.algae_intake_motor.getOutputCurrent()
            
            self.cache.current_counter = (self.cache.current_counter + 1) % 5
            
            # Cache motor temperatures (least frequently)
            if self.cache.temp_counter == 0:
                self.cache.motor_temps["CoralLeft"] = self.coral_left_motor.getMotorTemperature()
                self.cache.motor_temps["CoralRight"] = self.coral_right_motor.getMotorTemperature()
                self.cache.motor_temps["AlgaeRotation"] = self.algae_rotation_motor.getMotorTemperature()
                self.cache.motor_temps["AlgaeIntake"] = self.algae_intake_motor.getMotorTemperature()
            
            self.cache.temp_counter = (self.cache.temp_counter + 1) % 20
            
        except Exception as e:
            self.handleError("cacheSensors", e)
    
    def subsystemPeriodic(self):
        """Periodic code for the end effector subsystem."""
        try:
            # Check for safety issues
            safety_issues = []
            
            if self.checkMotorCurrents():
                safety_issues.append("Overcurrent")
            
            if self.detectJam():
                safety_issues.append("Jam")
            
            if not self.checkAlgaeRotationLimits():
                safety_issues.append("Rotation Limit")
            
            if safety_issues:
                # If safety issue detected, stop motors
                self.stopMotors()
                issues_str = ", ".join(safety_issues)
                wpilib.SmartDashboard.putString("EndEffector/Status", f"Safety: {issues_str}")
            
            # Update dashboard with current state
            wpilib.SmartDashboard.putBoolean("EndEffector/CoralDetected", self.isCoralDetected())
            wpilib.SmartDashboard.putBoolean("EndEffector/CoralPositioned", self.isCoralPositioned())
            wpilib.SmartDashboard.putNumber("EndEffector/AlgaePosition", self.getAlgaePosition())
            wpilib.SmartDashboard.putNumber("EndEffector/CoralEntry", self.cache.coral_entry_distance)
            wpilib.SmartDashboard.putNumber("EndEffector/CoralStop", self.cache.coral_stop_distance)
            
        except Exception as e:
            self.handleError("subsystemPeriodic", e)
    
    # ============ Coral Control Methods ============
    
    def isCoralDetected(self):
        """
        Check if coral is detected at the entrance.
        
        Returns:
            bool: True if coral is detected
        """
        return (self.cache.coral_entry_status == 0 and 
                self.cache.coral_entry_distance < endEffectorConsts.CORAL_DETECTION_THRESHOLD)
    
    def isCoralPositioned(self):
        """
        Check if coral is correctly positioned inside the mechanism.
        
        Returns:
            bool: True if coral is properly positioned
        """
        return (self.cache.coral_stop_status == 0 and 
                self.cache.coral_stop_distance < endEffectorConsts.CORAL_STOP_THRESHOLD)
    
    def setCoralIntakeLeftSpeed(self, speed):
        """Set the speed of the left coral intake motor."""
        try:
            self.coral_left_motor.set(speed)
        except Exception as e:
            self.handleError("setCoralIntakeLeftSpeed", e)
    
    def setCoralIntakeRightSpeed(self, speed):
        """Set the speed of the right coral intake motor."""
        try:
            self.coral_right_motor.set(speed)
        except Exception as e:
            self.handleError("setCoralIntakeRightSpeed", e)
    
    def stopCoralIntake(self):
        """Stop both coral intake motors."""
        self.setCoralIntakeLeftSpeed(0)
        self.setCoralIntakeRightSpeed(0)
    
    def intakeCoral(self, speed=None):
        """
        Run coral intake until the piece is properly positioned.
        
        Args:
            speed (float, optional): Speed to run intake. Defaults to CORAL_INTAKE_SPEED.
            
        Returns:
            bool: True if coral is positioned (intake complete)
        """
        if speed is None:
            speed = endEffectorConsts.CORAL_INTAKE_SPEED
        
        if not self.isCoralPositioned():
            self.setCoralIntakeLeftSpeed(speed)
            self.setCoralIntakeRightSpeed(speed)
            wpilib.SmartDashboard.putString("EndEffector/Status", "Intaking Coral")
            return False  # Not finished
        else:
            self.stopCoralIntake()
            wpilib.SmartDashboard.putString("EndEffector/Status", "Coral Positioned")
            return True  # Finished
    
    def ejectCoral(self, speed=0.7):
        """
        Eject coral from the mechanism.
        
        Args:
            speed (float, optional): Speed to run ejection. Defaults to 0.7.
        """
        try:
            # Use negative speed to eject
            self.setCoralIntakeLeftSpeed(-speed)
            self.setCoralIntakeRightSpeed(-speed)
            wpilib.SmartDashboard.putString("EndEffector/Status", "Ejecting Coral")
        except Exception as e:
            self.handleError("ejectCoral", e)
    
    # ============ Algae Control Methods ============
    
    def getAlgaePosition(self):
        """
        Get the current position of the algae mechanism.
        
        Returns:
            float: Current position in degrees
        """
        return self.cache.algae_position
    
    def setAlgaeRotationSpeed(self, speed):
        """
        Set the speed of the algae rotation motor.
        
        Args:
            speed (float): Speed to set (-1.0 to 1.0)
        """
        try:
            # Check rotation limits
            current_position = self.getAlgaePosition()
            
            # Prevent movement beyond limits
            if (current_position <= endEffectorConsts.ALGAE_MIN_ANGLE and speed < 0) or \
               (current_position >= endEffectorConsts.ALGAE_MAX_ANGLE and speed > 0):
                speed = 0
                wpilib.SmartDashboard.putString("EndEffector/Status", "Algae At Limit")
            else:
                if speed != 0:
                    wpilib.SmartDashboard.putString("EndEffector/Status", "Moving Algae")
            
            # Set motor output
            self.algae_rotation_motor.set(speed)
            
        except Exception as e:
            self.handleError("setAlgaeRotationSpeed", e)
            self.algae_rotation_motor.set(0)
    
    def setAlgaeIntakeSpeed(self, speed):
        """
        Set the speed of the algae intake motor.
        
        Args:
            speed (float): Speed to set (-1.0 to 1.0)
        """
        try:
            self.algae_intake_motor.set(speed)
            if speed > 0:
                wpilib.SmartDashboard.putString("EndEffector/Status", "Intaking Algae")
            elif speed < 0:
                wpilib.SmartDashboard.putString("EndEffector/Status", "Ejecting Algae")
        except Exception as e:
            self.handleError("setAlgaeIntakeSpeed", e)
    
    def moveAlgaeToPosition(self, position):
        """
        Move the algae mechanism to a specific position.
        
        Args:
            position (float): Target position in degrees
        """
        try:
            # Clamp position to valid range
            position = max(
                min(position, endEffectorConsts.ALGAE_MAX_ANGLE),
                endEffectorConsts.ALGAE_MIN_ANGLE
            )
            
            wpilib.SmartDashboard.putString("EndEffector/Status", "Positioning Algae")
            
            # Using built-in PID controller with absolute encoder
            if self.use_abs_encoder and hasattr(self, 'algae_pid_controller') and hasattr(self.algae_pid_controller, 'setReference'):
                self.algae_pid_controller.setReference(
                    position,
                    rev.CANSparkMax.ControlType.kPosition
                )
            else:
                # Using WPILib PID controller
                current_position = self.getAlgaePosition()
                output = self.algae_pid_controller.calculate(current_position, position)
                
                # Limit output to valid range
                output = max(min(output, 0.7), -0.7)  # Lower max speed for safety
                
                # Set motor output
                self.algae_rotation_motor.set(output)
            
        except Exception as e:
            self.handleError("moveAlgaeToPosition", e)
            self.algae_rotation_motor.set(0)
    
    def isAlgaeAtPosition(self, target_position, tolerance=None):
        """
        Check if the algae mechanism is at the target position.
        
        Args:
            target_position (float): Target position to check
            tolerance (float, optional): Position tolerance. Defaults to configured tolerance.
            
        Returns:
            bool: True if at position
        """
        if tolerance is None:
            tolerance = self.ALGAE_POSITION_TOLERANCE
            
        current_position = self.getAlgaePosition()
        return abs(current_position - target_position) <= tolerance
    
    def moveAlgaeToRetracted(self):
        """Move algae mechanism to retracted position."""
        self.moveAlgaeToPosition(self.ALGAE_RETRACTED_POS)
    
    def moveAlgaeToTopPickup(self):
        """Move algae mechanism to top pickup position."""
        self.moveAlgaeToPosition(self.ALGAE_TOP_PICKUP_POS)
    
    def moveAlgaeToBottomPickup(self):
        """Move algae mechanism to bottom pickup position."""
        self.moveAlgaeToPosition(self.ALGAE_BOTTOM_PICKUP_POS)
    
    # ============ Safety Check Methods ============
    
    def checkMotorCurrents(self):
        """
        Check if any motor current exceeds safe limits.
        
        Returns:
            bool: True if overcurrent detected
        """
        # Define critical current threshold
        critical_threshold = endEffectorConsts.CURRENT_CRITICAL_THRESHOLD
        
        # Check each motor's current
        if self.cache.coral_left_current > critical_threshold:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Coral Left Overcurrent")
            return True
            
        if self.cache.coral_right_current > critical_threshold:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Coral Right Overcurrent")
            return True
            
        if self.cache.algae_rotation_current > critical_threshold:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Algae Rotation Overcurrent")
            return True
            
        if self.cache.algae_intake_current > critical_threshold:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Algae Intake Overcurrent")
            return True
            
        return False
    
    def detectJam(self):
        """
        Detect if a jam has occurred in either mechanism.
        
        Returns:
            bool: True if jam detected
        """
        # Detect coral jam - high current but no movement in position
        if self.isCoralDetected() and not self.isCoralPositioned():
            coral_current_avg = (self.cache.coral_left_current + self.cache.coral_right_current) / 2
            if coral_current_avg > endEffectorConsts.CORAL_JAM_CURRENT_THRESHOLD:
                wpilib.SmartDashboard.putString("EndEffector/Warning", "Coral Jam Detected")
                return True
        
        # Detect algae jam - high current but low velocity
        if abs(self.cache.algae_intake_current) > endEffectorConsts.ALGAE_JAM_CURRENT_THRESHOLD and \
           abs(self.cache.algae_velocity) < 1.0:  # Near-zero velocity
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Algae Jam Detected")
            return True
            
        return False
    
    def checkAlgaeRotationLimits(self):
        """
        Check if algae rotation is within safe limits.
        
        Returns:
            bool: True if within limits
        """
        current_position = self.getAlgaePosition()
        
        # Check if outside limits
        if current_position < endEffectorConsts.ALGAE_MIN_ANGLE:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Algae Below Min Angle")
            return False
            
        if current_position > endEffectorConsts.ALGAE_MAX_ANGLE:
            wpilib.SmartDashboard.putString("EndEffector/Warning", "Algae Above Max Angle")
            return False
            
        return True
    
    # ============ Stop Methods ============
    
    def stopMotors(self):
        """Stop all motors in the end effector."""
        try:
            # Stop coral intake motors
            self.coral_left_motor.set(0)
            self.coral_right_motor.set(0)
            
            # Stop algae motors
            self.algae_rotation_motor.set(0)
            self.algae_intake_motor.set(0)
            
            # Update dashboard status
            wpilib.SmartDashboard.putString("EndEffector/Status", "All Motors Stopped")
            
        except Exception as e:
            self.handleError("stopMotors", e)
    
    def subsystemSimulationPeriodic(self):
        """Periodic simulation code for the end effector."""
        if self.is_simulation:
            # In simulation, update mechanism states based on motor outputs
            # For example, we could simulate coral movement by adjusting sensor values
            pass