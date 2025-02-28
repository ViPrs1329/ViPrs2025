# EndEffector.py
import rev
import commands2
import wpilib
from constants import CANIDs, endEffectorConsts
import grapple.LaserCAN as LC
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory

class EndEffector(commands2.Subsystem):
    class Cache:
        def __init__(self):
            # Cached sensor values
            self.coral_entry_distance = 8000
            self.coral_entry_status = 0
            self.coral_stop_distance = 8000
            self.coral_stop_status = 0
            
            # Algae mechanism
            self.algae_rotation_position = 0.0
            self.algae_rotation_velocity = 0.0
            
            # Motor current values (read less frequently)
            self.algae_rotation_current = 0.0
            self.algae_intake_current = 0.0
            self.coral_left_current = 0.0
            self.coral_right_current = 0.0
            
            # Counter for less frequent reads
            self.current_counter = 0
    
    def __init__(self) -> None:
        super().__init__()

        # Initialize cache
        self.cache = self.Cache()

        # Check if we're in simulation mode
        self.is_simulation = wpilib.RobotBase.isSimulation()
        
        # Create configurations for motors
        try:
            # 1. Configure Algae Rotation Motor
            algae_rotation_config = SparkMaxFactory.Configuration()
            algae_rotation_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            algae_rotation_config.current_limit = endEffectorConsts.algaeRotCurrentLimit
            algae_rotation_config.voltage_comp_enabled = True
            algae_rotation_config.inverted = False
            
            # PID values for algae rotation
            algae_rotation_config.kP = 0.1  # Adjust based on your mechanism
            algae_rotation_config.kI = 0.0
            algae_rotation_config.kD = 0.005
            algae_rotation_config.kF = 0.0
            
            # 2. Configure Algae Intake Motor
            algae_intake_config = SparkMaxFactory.Configuration()
            algae_intake_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            algae_intake_config.current_limit = endEffectorConsts.algaeIntakeCurrentLimit
            algae_intake_config.voltage_comp_enabled = True
            algae_intake_config.inverted = False
            
            # 3. Configure Coral Left Motor
            coral_left_config = SparkMaxFactory.Configuration()
            coral_left_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            coral_left_config.current_limit = endEffectorConsts.coralCurrentLimit
            coral_left_config.voltage_comp_enabled = True
            coral_left_config.inverted = False
            
            # 4. Configure Coral Right Motor
            coral_right_config = SparkMaxFactory.Configuration()
            coral_right_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast
            coral_right_config.current_limit = endEffectorConsts.coralCurrentLimit
            coral_right_config.voltage_comp_enabled = True
            coral_right_config.inverted = True  # Note: Inverted
            
            # Create motors using factory
            self.algae_rotation_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EEAlgaeArmRotationID, algae_rotation_config
            )
            
            self.algae_intake_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EEAlgaeIntakeID, algae_intake_config
            )
            
            self.coral_intake_left_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EECoralLeftID, coral_left_config
            )
            
            self.coral_intake_right_motor = SparkMaxFactory.createSparkMax(
                CANIDs.EECoralRightID, coral_right_config
            )
            
            # Get encoders for the algae rotation motor
            self.algae_rotation_encoder = self.algae_rotation_motor.getEncoder()
            
            # Get the absolute encoder for algae rotation (if available)
            try:
                self.algae_abs_encoder = self.algae_rotation_motor.getAbsoluteEncoder(
                    rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
                )
                
                # Configure the absolute encoder
                # Set position conversion factor (convert rotations to degrees)
                self.algae_abs_encoder.setPositionConversionFactor(360.0)
                
                # Set zero offset based on mechanical setup
                offset = endEffectorConsts.ALGAE_ENCODER_OFFSET if hasattr(endEffectorConsts, 'ALGAE_ENCODER_OFFSET') else 0.0
                self.algae_abs_encoder.setZeroOffset(offset)
                
                # Use the absolute encoder for the PID controller
                self.algae_pid_controller = self.algae_rotation_motor.getPIDController()
                self.algae_pid_controller.setFeedbackDevice(self.algae_abs_encoder)
                
                # Set PID values
                self.algae_pid_controller.setP(algae_rotation_config.kP)
                self.algae_pid_controller.setI(algae_rotation_config.kI)
                self.algae_pid_controller.setD(algae_rotation_config.kD)
                self.algae_pid_controller.setFF(algae_rotation_config.kF)
                
                self.has_abs_encoder = True
                print("Algae absolute encoder configured successfully")
            except Exception as e:
                print(f"Failed to initialize algae absolute encoder: {e}")
                # Fall back to relative encoder
                self.has_abs_encoder = False
                
                # Configure the relative encoder
                self.algae_rotation_encoder.setPositionConversionFactor(360.0 / 49.0)  # Adjust for 49:1 gearbox
                
                # Create a WPILib PID controller as fallback
                from wpimath.controller import PIDController
                self.algae_pid_controller = PIDController(
                    algae_rotation_config.kP,
                    algae_rotation_config.kI,
                    algae_rotation_config.kD
                )
            
            # Define the algae positions (in degrees)
            self.ALGAE_RETRACTED_POS = 0.0
            self.ALGAE_TOP_PICKUP_POS = 90.0
            self.ALGAE_BOTTOM_PICKUP_POS = -90.0
            
            # Position tolerance (in degrees)
            self.ALGAE_POSITION_TOLERANCE = 5.0
                
        except Exception as e:
            print(f"Error initializing REV motors: {e}")
            if self.is_simulation:
                print("Running in simulation mode - errors are expected")
            else:
                raise  # Re-raise if not in simulation

        # Initialize LaserCAN sensors
        try:
            self.coral_intake_LC = LC.LaserCAN(CANIDs.EECoralInSensorID)
            self.coral_stop_LC = LC.LaserCAN(CANIDs.EECoralStopSensorID)
        except Exception as e:
            print(f"Error initializing LaserCAN sensors: {e}")
            if self.is_simulation:
                print("Running in simulation mode - initializing simulated LaserCAN sensors")
                # Create simulated LaserCAN sensors
                self.coral_intake_LC = self._create_sim_laser()
                self.coral_stop_LC = self._create_sim_laser()
            else:
                raise  # Re-raise if not in simulation
    
    def cacheSensors(self):
        """Cache sensor values to reduce bus traffic."""
        try:
            # Always cache LaserCAN readings
            measurement = self.coral_intake_LC.get_measurement()
            if measurement:
                self.cache.coral_entry_distance, self.cache.coral_entry_status = measurement
                
            measurement = self.coral_stop_LC.get_measurement()
            if measurement:
                self.cache.coral_stop_distance, self.cache.coral_stop_status = measurement
            
            # Cache algae position and velocity
            if hasattr(self, 'algae_rotation_encoder'):
                if self.has_abs_encoder:
                    self.cache.algae_rotation_position = self.algae_abs_encoder.getPosition()
                else:
                    self.cache.algae_rotation_position = self.algae_rotation_encoder.getPosition()
                
                self.cache.algae_rotation_velocity = self.algae_rotation_encoder.getVelocity()
                
            # Cache motor currents less frequently
            if self.cache.current_counter == 0:
                self.cache.algae_rotation_current = self.algae_rotation_motor.getOutputCurrent()
                self.cache.algae_intake_current = self.algae_intake_motor.getOutputCurrent()
                self.cache.coral_left_current = self.coral_intake_left_motor.getOutputCurrent()
                self.cache.coral_right_current = self.coral_intake_right_motor.getOutputCurrent()
                
            self.cache.current_counter = (self.cache.current_counter + 1) % 10
                
        except Exception as e:
            if not self.is_simulation:
                print(f"Error caching sensor values: {e}")
    
    def periodic(self):
        """Called periodically during all robot modes."""
        # Update sensor cache
        self.cacheSensors()
        
        # Update dashboard telemetry
        wpilib.SmartDashboard.putBoolean("Coral Detected", self.isCoralDetected())
        wpilib.SmartDashboard.putBoolean("Coral Positioned", self.isCoralPositioned())
        wpilib.SmartDashboard.putNumber("Algae Position", self.getAlgaePosition())
    
    def _create_sim_laser(self):
        """Create a simulated LaserCAN with minimal interface for simulation."""
        class SimLaser:
            def __init__(self):
                self.distance = 8000
                self.status = 0
                
            def get_measurement(self):
                return (self.distance, self.status)
                
            def set_simulated_distance(self, distance, status=0):
                self.distance = distance
                self.status = status
                
            def is_object_detected(self, threshold_mm=100):
                return self.distance < threshold_mm
        
        return SimLaser()

    # Algae rotation control
    def setAlgaeRotationSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake rotation motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        try:
            self.algae_rotation_motor.set(speed)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error setting algae rotation speed: {e}")
    
    def getAlgaePosition(self) -> float:
        """Get the current position of the algae mechanism.
        
        Returns:
            float: Current position in degrees.
        """
        return self.cache.algae_rotation_position
    
    def moveAlgaeToPosition(self, position: float) -> None:
        """Move the algae mechanism to a specific position.
        
        Args:
            position (float): Target position in degrees.
        """
        try:
            if self.has_abs_encoder:
                # Use the built-in PID controller
                self.algae_pid_controller.setReference(
                    position, 
                    rev.CANSparkMax.ControlType.kPosition
                )
            else:
                # Use the WPILib PID controller
                current_position = self.getAlgaePosition()
                output = self.algae_pid_controller.calculate(current_position, position)
                
                # Limit the output
                output = max(min(output, 0.5), -0.5)
                
                # Set the motor output
                self.setAlgaeRotationSpeed(output)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error moving algae to position: {e}")
    
    def isAlgaeAtPosition(self, target_position: float, tolerance: float = None) -> bool:
        """Check if the algae mechanism is at the target position.
        
        Args:
            target_position (float): Target position in degrees.
            tolerance (float, optional): Position tolerance. Defaults to ALGAE_POSITION_TOLERANCE.
            
        Returns:
            bool: True if the mechanism is at the target position.
        """
        if tolerance is None:
            tolerance = self.ALGAE_POSITION_TOLERANCE
            
        current_position = self.getAlgaePosition()
        return abs(current_position - target_position) <= tolerance

    def setAlgaeIntakeSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        try:
            self.algae_intake_motor.set(speed)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error setting algae intake speed: {e}")

    def setCoralIntakeLeftSpeed(self, speed: float) -> None:
        """Sets the speed of the left coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        try:
            self.coral_intake_left_motor.set(speed)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error setting left coral intake speed: {e}")

    def setCoralIntakeRightSpeed(self, speed: float) -> None:
        """Sets the speed of the right coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        try:
            self.coral_intake_right_motor.set(speed)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error setting right coral intake speed: {e}")

    def stopAllMotors(self) -> None:
        """Stops all motors in the end effector subsystem."""
        try:
            self.algae_rotation_motor.set(0)
            self.algae_intake_motor.set(0)
            self.coral_intake_left_motor.set(0)
            self.coral_intake_right_motor.set(0)
        except Exception as e:
            if not self.is_simulation:
                print(f"Error stopping motors: {e}")

    def isCoralDetected(self) -> bool:
        """Check if coral is detected at entrance of intake."""
        # Use cached values instead of direct sensor reads
        return (self.cache.coral_entry_status == 0 and 
                self.cache.coral_entry_distance < endEffectorConsts.CORAL_DETECTION_THRESHOLD)
    
    def isCoralPositioned(self) -> bool:
        """Check if coral has reached correct position inside intake."""
        # Use cached values instead of direct sensor reads
        return (self.cache.coral_stop_status == 0 and 
                self.cache.coral_stop_distance < endEffectorConsts.CORAL_STOP_THRESHOLD)

    def intakeCoral(self, speed=None):
        """Intake coral at given speed until properly positioned.
        
        Args:
            speed (float, optional): The speed to run the intake. Defaults to CORAL_INTAKE_SPEED.
            
        Returns:
            bool: True if finished (coral positioned), False otherwise
        """
        if speed is None:
            speed = endEffectorConsts.CORAL_INTAKE_SPEED
            
        if not self.isCoralPositioned():
            self.setCoralIntakeLeftSpeed(speed)
            self.setCoralIntakeRightSpeed(speed)
            return False  # Not finished
        else:
            self.stopCoralIntake()
            return True  # Finished
        
    def stopCoralIntake(self):
        """Stop the coral intake motors."""
        self.setCoralIntakeLeftSpeed(0)
        self.setCoralIntakeRightSpeed(0)
        
    def getCoralEntryDistance(self):
        """Get the current distance reading from the coral entry sensor."""
        return self.cache.coral_entry_distance
    
    def getCoralStopDistance(self):
        """Get the current distance reading from the coral stop sensor."""
        return self.cache.coral_stop_distance