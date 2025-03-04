import rev
import wpilib
from libgrapplefrc import LaserCan, CanBridge
from commands2 import SubsystemBase
from constants.constants import AlgaeManipulatorConstants, CoralManipulatorConstants

class EndEffector(SubsystemBase):
    """
    The End Effector subsystem handles both the Algae and Coral manipulators.
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize CanBridge for LaserCan
        CanBridge.runWebsocketInBackground(7171)
        
        # Create Algae manipulator motors
        self.algae_rotation_motor = rev.SparkMax(
            AlgaeManipulatorConstants.ROTATION_MOTOR,
            rev.SparkMax.MotorType.kBrushless
        )
        self.algae_intake_motor = rev.SparkMax(
            AlgaeManipulatorConstants.INTAKE_MOTOR,
            rev.SparkMax.MotorType.kBrushless
        )
        
        # Create Coral manipulator motors
        self.coral_left_motor = rev.SparkMax(
            CoralManipulatorConstants.LEFT_MOTOR,
            rev.SparkMax.MotorType.kBrushless
        )
        self.coral_right_motor = rev.SparkMax(
            CoralManipulatorConstants.RIGHT_MOTOR,
            rev.SparkMax.MotorType.kBrushless
        )
        
        # Configure Algae motors
        self.algae_rotation_motor.setInverted(False)  # Adjust if needed
        self.algae_intake_motor.setInverted(False)   # Adjust if needed
        
        # Configure Coral motors (opposite directions for intake)
        self.coral_left_motor.setInverted(False)     # Adjust if needed
        self.coral_right_motor.setInverted(True)     # Adjust if needed
        
        # Create and configure Coral sensors
        self.coral_intake_sensor = LaserCan(CoralManipulatorConstants.LEFT_SENSOR)
        self.coral_outlet_sensor = LaserCan(CoralManipulatorConstants.RIGHT_SENSOR)
        
        try:
            # Configure both sensors
            for sensor in [self.coral_intake_sensor, self.coral_outlet_sensor]:
                sensor.setRangingMode(LaserCan.RangingMode.SHORT)
                sensor.setRegionOfInterest(LaserCan.RegionOfInterest(8, 8, 16, 16))
                sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS)
        except Exception as e:
            print(f"LaserCan configuration failed: {e}")
        
        # Initialize state variables
        self.coral_intaking = False
        self.coral_fully_intaken = False
        
        # Add to SmartDashboard
        self.init_smartdashboard()
    
    def init_smartdashboard(self):
        """Initialize SmartDashboard entries."""
        # Algae state
        wpilib.SmartDashboard.putNumber("Algae/Arm Position", 0)
        wpilib.SmartDashboard.putNumber("Algae/Intake Speed", 0)
        
        # Coral state
        wpilib.SmartDashboard.putBoolean("Coral/Intaking", False)
        wpilib.SmartDashboard.putBoolean("Coral/Fully Intaken", False)
        wpilib.SmartDashboard.putNumber("Coral/Intake Distance (mm)", -1)
        wpilib.SmartDashboard.putNumber("Coral/Outlet Distance (mm)", -1)
    
    def periodic(self):
        """Update SmartDashboard with current state."""
        # Update Algae state
        wpilib.SmartDashboard.putNumber(
            "Algae/Arm Position",
            self.algae_rotation_motor.getEncoder().getPosition()
        )
        wpilib.SmartDashboard.putNumber(
            "Algae/Intake Speed",
            self.algae_intake_motor.getEncoder().getVelocity()
        )
        
        # Update Coral state and sensor readings
        wpilib.SmartDashboard.putBoolean("Coral/Intaking", self.coral_intaking)
        wpilib.SmartDashboard.putBoolean("Coral/Fully Intaken", self.coral_fully_intaken)
        
        # Get and update intake sensor measurement
        intake_measurement = self.coral_intake_sensor.getMeasurement()
        if intake_measurement and intake_measurement.status == 0:
            wpilib.SmartDashboard.putNumber("Coral/Intake Distance (mm)", intake_measurement.distance_mm)
        else:
            wpilib.SmartDashboard.putNumber("Coral/Intake Distance (mm)", -1)
            
        # Get and update outlet sensor measurement
        outlet_measurement = self.coral_outlet_sensor.getMeasurement()
        if outlet_measurement and outlet_measurement.status == 0:
            wpilib.SmartDashboard.putNumber("Coral/Outlet Distance (mm)", outlet_measurement.distance_mm)
        else:
            wpilib.SmartDashboard.putNumber("Coral/Outlet Distance (mm)", -1)
    
    # Algae Manipulator Methods
    def set_algae_arm_position(self, position: float):
        """
        Set the Algae arm position.
        
        :param position: Target position in radians
        """
        self.algae_rotation_motor.getPIDController().setReference(
            position,
            rev.SparkMax.ControlType.kPosition
        )
    
    def set_algae_intake_speed(self, speed: float):
        """
        Set the Algae intake speed.
        
        :param speed: Speed from -1 to 1
        """
        self.algae_intake_motor.set(speed)
    
    def get_algae_arm_position(self) -> float:
        """
        Get the current Algae arm position.
        
        :return: Current position in radians
        """
        return self.algae_rotation_motor.getEncoder().getPosition()
    
    # Coral Manipulator Methods
    def set_coral_intake_speed(self, speed: float):
        """
        Set the Coral intake speed.
        
        :param speed: Speed from -1 to 1
        """
        self.coral_left_motor.set(speed)
        self.coral_right_motor.set(speed)
    
    def get_coral_intake_sensor(self) -> bool:
        """
        Get the state of the Coral intake sensor.
        
        :return: True if Coral is detected at intake (within detection threshold)
        """
        measurement = self.coral_intake_sensor.getMeasurement()
        if measurement and measurement.status == 0:
            # Return True if object is detected within threshold distance (adjust as needed)
            return measurement.distance_mm < CoralManipulatorConstants.DETECTION_THRESHOLD_MM
        return False
    
    def get_coral_outlet_sensor(self) -> bool:
        """
        Get the state of the Coral outlet sensor.
        
        :return: True if Coral is detected at outlet (within detection threshold)
        """
        measurement = self.coral_outlet_sensor.getMeasurement()
        if measurement and measurement.status == 0:
            # Return True if object is detected within threshold distance (adjust as needed)
            return measurement.distance_mm < CoralManipulatorConstants.DETECTION_THRESHOLD_MM
        return False
    
    def is_coral_fully_intaken(self) -> bool:
        """
        Check if Coral is fully intaken.
        
        :return: True if Coral is fully intaken
        """
        return self.coral_fully_intaken
    
    def set_coral_intaking(self, intaking: bool):
        """
        Set the Coral intaking state.
        
        :param intaking: True if Coral is being intaken
        """
        self.coral_intaking = intaking
    
    def set_coral_fully_intaken(self, fully_intaken: bool):
        """
        Set the Coral fully intaken state.
        
        :param fully_intaken: True if Coral is fully intaken
        """
        self.coral_fully_intaken = fully_intaken
    
    def stop_all(self):
        """Stop all motors."""
        self.algae_rotation_motor.set(0)
        self.algae_intake_motor.set(0)
        self.coral_left_motor.set(0)
        self.coral_right_motor.set(0) 