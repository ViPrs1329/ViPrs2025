import rev
import wpilib
from libgrapplefrc import LaserCan, CanBridge
from commands2 import SubsystemBase
from utils.caching import CachingSubsystemBase
from constants.constants import AlgaeManipulatorConstants, CoralManipulatorConstants

class EndEffector(CachingSubsystemBase):
    """
    The End Effector subsystem handles both the Algae and Coral manipulators.
    """
    
    class Cache(CachingSubsystemBase.Cache):
        """Cache specific to end effector subsystem"""
        def __init__(self):
            super().__init__()
            # Initialize with default values
            # Algae manipulator
            self.set_cached("algae_arm_position", 0.0)
            self.set_cached("algae_intake_speed", 0.0)
            self.set_setpoint("algae_target_position", None)
            self.set_setpoint("algae_target_speed", 0.0)
            
            # Coral manipulator
            self.set_cached("coral_intake_distance", -1)
            self.set_cached("coral_outlet_distance", -1)
            self.set_cached("coral_intake_detected", False)
            self.set_cached("coral_outlet_detected", False)
            self.set_setpoint("coral_intake_speed", 0.0)
            
            # State flags
            self.set_cached("coral_intaking", False)
            self.set_cached("coral_fully_intaken", False)
    
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
    
    def cache_sensors(self) -> None:
        """Cache all sensor values."""
        # Cache Algae manipulator values
        self.cache.set_cached("algae_arm_position", 
                            self.algae_rotation_motor.getEncoder().getPosition())
        self.cache.set_cached("algae_intake_speed", 
                            self.algae_intake_motor.getEncoder().getVelocity())
        
        # Cache Coral manipulator sensor values
        intake_measurement = self.coral_intake_sensor.getMeasurement()
        outlet_measurement = self.coral_outlet_sensor.getMeasurement()
        
        # Update intake sensor cache
        if intake_measurement and intake_measurement.status == 0:
            distance = intake_measurement.distance_mm
            self.cache.set_cached("coral_intake_distance", distance)
            self.cache.set_cached("coral_intake_detected", 
                                distance < CoralManipulatorConstants.DETECTION_THRESHOLD_MM)
        else:
            self.cache.set_cached("coral_intake_distance", -1)
            self.cache.set_cached("coral_intake_detected", False)
            
        # Update outlet sensor cache
        if outlet_measurement and outlet_measurement.status == 0:
            distance = outlet_measurement.distance_mm
            self.cache.set_cached("coral_outlet_distance", distance)
            self.cache.set_cached("coral_outlet_detected", 
                                distance < CoralManipulatorConstants.DETECTION_THRESHOLD_MM)
        else:
            self.cache.set_cached("coral_outlet_distance", -1)
            self.cache.set_cached("coral_outlet_detected", False)
    
    def update_hardware(self) -> None:
        """Update hardware with cached setpoints."""
        # Update Algae manipulator
        target_position = self.cache.get_setpoint("algae_target_position")
        if target_position is not None:
            self.algae_rotation_motor.getPIDController().setReference(
                target_position,
                rev.SparkMax.ControlType.kPosition
            )
        
        # Update motor speeds
        self.algae_intake_motor.set(self.cache.get_setpoint("algae_target_speed"))
        coral_speed = self.cache.get_setpoint("coral_intake_speed")
        self.coral_left_motor.set(coral_speed)
        self.coral_right_motor.set(coral_speed)
    
    def periodic_logic(self) -> None:
        """Update SmartDashboard with cached values."""
        # Algae state
        wpilib.SmartDashboard.putNumber(
            "Algae/Arm Position",
            self.cache.get_cached("algae_arm_position")
        )
        wpilib.SmartDashboard.putNumber(
            "Algae/Intake Speed",
            self.cache.get_cached("algae_intake_speed")
        )
        
        # Coral state
        wpilib.SmartDashboard.putBoolean(
            "Coral/Intaking", 
            self.cache.get_cached("coral_intaking")
        )
        wpilib.SmartDashboard.putBoolean(
            "Coral/Fully Intaken",
            self.cache.get_cached("coral_fully_intaken")
        )
        wpilib.SmartDashboard.putNumber(
            "Coral/Intake Distance (mm)",
            self.cache.get_cached("coral_intake_distance")
        )
        wpilib.SmartDashboard.putNumber(
            "Coral/Outlet Distance (mm)",
            self.cache.get_cached("coral_outlet_distance")
        )
    
    # Algae Manipulator Methods
    def set_algae_arm_position(self, position: float):
        """
        Set the Algae arm position.
        
        :param position: Target position in radians
        """
        self.cache.set_setpoint("algae_target_position", position)
    
    def set_algae_intake_speed(self, speed: float):
        """
        Set the Algae intake speed.
        
        :param speed: Speed from -1 to 1
        """
        self.cache.set_setpoint("algae_target_speed", speed)
    
    def get_algae_arm_position(self) -> float:
        """
        Get the current Algae arm position.
        
        :return: Current position in radians
        """
        return self.cache.get_cached("algae_arm_position")
    
    # Coral Manipulator Methods
    def set_coral_intake_speed(self, speed: float):
        """
        Set the Coral intake speed.
        
        :param speed: Speed from -1 to 1
        """
        self.cache.set_setpoint("coral_intake_speed", speed)
    
    def get_coral_intake_sensor(self) -> bool:
        """
        Get the state of the Coral intake sensor.
        
        :return: True if Coral is detected at intake (within detection threshold)
        """
        return self.cache.get_cached("coral_intake_detected")
    
    def get_coral_outlet_sensor(self) -> bool:
        """
        Get the state of the Coral outlet sensor.
        
        :return: True if Coral is detected at outlet (within detection threshold)
        """
        return self.cache.get_cached("coral_outlet_detected")
    
    def is_coral_fully_intaken(self) -> bool:
        """
        Check if Coral is fully intaken.
        
        :return: True if Coral is fully intaken
        """
        return self.cache.get_cached("coral_fully_intaken")
    
    def set_coral_intaking(self, intaking: bool):
        """
        Set the Coral intaking state.
        
        :param intaking: True if Coral is being intaken
        """
        self.cache.set_cached("coral_intaking", intaking)
    
    def set_coral_fully_intaken(self, fully_intaken: bool):
        """
        Set the Coral fully intaken state.
        
        :param fully_intaken: True if Coral is fully intaken
        """
        self.cache.set_cached("coral_fully_intaken", fully_intaken)
    
    def stop_all(self):
        """Stop all motors."""
        self.set_algae_arm_position(self.get_algae_arm_position())  # Hold current position
        self.set_algae_intake_speed(0)
        self.set_coral_intake_speed(0) 