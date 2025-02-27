import rev
import commands2
import wpilib
from constants import CANIDs, endEffectorConsts
import grapple.LaserCAN as LC

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Check if we're in simulation mode
        self.is_simulation = wpilib.RobotBase.isSimulation()
        
        # 1. Initialize motors with appropriate error handling
        try:
            # Algae Intake Rotation Motor
            self.algae_rotation_motor = rev.SparkMax(
                CANIDs.EEAlgaeArmRotationID, rev.SparkMax.MotorType.kBrushless
            )
            self.algae_rotation_motor.setInverted(False)
            
            if hasattr(rev, 'SparkBaseConfig'):  # Check for new API
                self.algae_rotation_motor_config = rev.SparkBaseConfig()
                self.algae_rotation_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                self.algae_rotation_motor_config.smartCurrentLimit(endEffectorConsts.algaeRotCurrentLimit)
                self.algae_rotation_motor.configure(self.algae_rotation_motor_config, 
                                                   rev.SparkBase.ResetMode.kResetSafeParameters, 
                                                   rev.SparkBase.PersistMode.kPersistParameters)
            else:  # Fallback for simulation or older API
                self.algae_rotation_motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
                self.algae_rotation_motor.setSmartCurrentLimit(endEffectorConsts.algaeRotCurrentLimit)
            
            # 2. Algae Intake Motor
            self.algae_intake_motor = rev.SparkMax(
                CANIDs.EEAlgaeIntakeID, rev.SparkMax.MotorType.kBrushless
            )
            self.algae_intake_motor.setInverted(False)
            
            if hasattr(rev, 'SparkBaseConfig'):
                self.algae_intake_motor_config = rev.SparkBaseConfig()
                self.algae_intake_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
                self.algae_intake_motor_config.smartCurrentLimit(endEffectorConsts.algaeIntakeCurrentLimit)
                self.algae_intake_motor.configure(self.algae_intake_motor_config,
                                                 rev.SparkBase.ResetMode.kResetSafeParameters,
                                                 rev.SparkBase.PersistMode.kPersistParameters)
            else:
                self.algae_intake_motor.setIdleMode(rev.SparkMax.IdleMode.kCoast)
                self.algae_intake_motor.setSmartCurrentLimit(endEffectorConsts.algaeIntakeCurrentLimit)
            
            # 3. Coral Intake Left Motor
            self.coral_intake_left_motor = rev.SparkMax(
                CANIDs.EECoralLeftID, rev.SparkMax.MotorType.kBrushless
            )
            self.coral_intake_left_motor.setInverted(False)
            
            if hasattr(rev, 'SparkBaseConfig'):
                self.coral_intake_left_motor_config = rev.SparkBaseConfig()
                self.coral_intake_left_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
                self.coral_intake_left_motor_config.smartCurrentLimit(endEffectorConsts.coralCurrentLimit)
                self.coral_intake_left_motor.configure(self.coral_intake_left_motor_config,
                                                     rev.SparkBase.ResetMode.kResetSafeParameters,
                                                     rev.SparkBase.PersistMode.kPersistParameters)
            else:
                self.coral_intake_left_motor.setIdleMode(rev.SparkMax.IdleMode.kCoast)
                self.coral_intake_left_motor.setSmartCurrentLimit(endEffectorConsts.coralCurrentLimit)
            
            # 4. Coral Intake Right Motor
            self.coral_intake_right_motor = rev.SparkMax(
                CANIDs.EECoralRightID, rev.SparkMax.MotorType.kBrushless
            )
            self.coral_intake_right_motor.setInverted(True)  # Inverted
            
            if hasattr(rev, 'SparkBaseConfig'):
                self.coral_intake_right_motor_config = rev.SparkBaseConfig()
                self.coral_intake_right_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
                self.coral_intake_right_motor_config.smartCurrentLimit(endEffectorConsts.coralCurrentLimit)
                self.coral_intake_right_motor.configure(self.coral_intake_right_motor_config,
                                                     rev.SparkBase.ResetMode.kResetSafeParameters,
                                                     rev.SparkBase.PersistMode.kPersistParameters)
            else:
                self.coral_intake_right_motor.setIdleMode(rev.SparkMax.IdleMode.kCoast)
                self.coral_intake_right_motor.setSmartCurrentLimit(endEffectorConsts.coralCurrentLimit)
                
        except Exception as e:
            print(f"Error initializing REV motors: {e}")
            if self.is_simulation:
                print("Running in simulation mode - errors are expected")
            else:
                raise  # Re-raise if not in simulation

        # 5. Initialize LaserCAN sensors
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
        try:
            measurement = self.coral_intake_LC.get_measurement()
            if measurement:
                distance, status = measurement
                # Use a threshold distance defined in constants.py
                return status == 0 and distance < endEffectorConsts.CORAL_DETECTION_THRESHOLD
        except Exception as e:
            if not self.is_simulation:
                print(f"Error checking coral detection: {e}")
        return False
    
    def isCoralPositioned(self) -> bool:
        """Check if coral has reached correct position inside intake."""
        try:
            measurement = self.coral_stop_LC.get_measurement()
            if measurement:
                distance, status = measurement
                return status == 0 and distance < endEffectorConsts.CORAL_STOP_THRESHOLD
        except Exception as e:
            if not self.is_simulation:
                print(f"Error checking coral position: {e}")
        return False

    def intakeCoral(self, speed=endEffectorConsts.CORAL_INTAKE_SPEED):
        """Intake coral at given speed until properly positioned.
        
        Returns:
            bool: True if finished (coral positioned), False otherwise
        """
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