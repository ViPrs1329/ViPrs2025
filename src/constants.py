# src/constants.py
import math

# ================ Common Constants ================

class CANIDs:
    """CAN bus device IDs for all robot components."""
    
    # Swerve Drive CANIDs
    SwerveModuleDrive1 = 1  # Front Left Drive
    SwerveModuleRotation1 = 2  # Front Left Rotation
    SwerveModuleDrive2 = 3  # Front Right Drive
    SwerveModuleRotation2 = 4  # Front Right Rotation
    SwerveModuleDrive3 = 5  # Back Left Drive
    SwerveModuleRotation3 = 6  # Back Left Rotation
    SwerveModuleDrive4 = 7  # Back Right Drive
    SwerveModuleRotation4 = 8  # Back Right Rotation
    
    # Swerve Encoder CANIDs
    EncoderModuleRotation1 = 11  # Front Left Encoder
    EncoderModuleRotation2 = 12  # Front Right Encoder
    EncoderModuleRotation3 = 13  # Back Left Encoder
    EncoderModuleRotation4 = 14  # Back Right Encoder
    
    # Gyro CANID
    PigeonID = 15
    
    # Elevator CANIDs
    ElevatorLeftID = 16
    ElevatorRightID = 17
    
    # End Effector CANIDs
    EECoralLeftID = 18
    EECoralRightID = 19
    EECoralInSensorID = 20
    EECoralStopSensorID = 21
    EEAlgaeArmRotationID = 22
    EEAlgaeIntakeID = 23

# ================ Controller Settings ================

class controllerConsts:
    """Constants for driver and operator controllers."""
    
    # Deadbands for joysticks
    JOYSTICK_DEADBAND = 0.05
    ROTATION_DEADBAND = 0.1
    
    # Speed scaling (0-1)
    DRIVE_SPEED_SCALE = 0.8
    ROTATION_SPEED_SCALE = 0.6
    
    # Boost and slow mode
    BOOST_MULTIPLIER = 1.5
    SLOW_MULTIPLIER = 0.5
    
    # Controller ports
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1

# ================ Swerve Drive Constants ================

class driveConsts:
    """Constants for swerve drive subsystem."""
    
    # Physical dimensions (meters)
    WHEELBASE = 0.6  # Distance between front and back wheels
    TRACKWIDTH = 0.6  # Distance between left and right wheels
    WHEEL_DIAMETER = 0.1016  # 4 inch wheels in meters
    WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
    
    # Drive motor configuration
    DRIVE_GEAR_RATIO = 6.75  # SDS Mk4i L2 gearing
    MAX_SPEED = 4.0  # meters per second
    
    # Current limits
    currentLimit = 40  # Amps
    
    # Acceleration limits
    MAX_LINEAR_ACCELERATION = 3.0  # m/s²
    MAX_ANGULAR_ACCELERATION = 4.0  # rad/s²
    
    # PID values for swerve modules
    ROTATION_kP = 4.0
    ROTATION_kI = 0.0
    ROTATION_kD = 0.2

# ================ Elevator Constants ================

class elevatorConsts:
    """Constants for elevator subsystem."""
    
    # Current and temperature thresholds
    currentLimit = 40  # Amps
    CURRENT_LIMIT_THRESHOLD = 35  # Warning threshold
    TEMP_LIMIT_THRESHOLD = 80  # Celsius
    
    # PID control constants
    kP = 0.1
    kI = 0.0
    kD = 0.005
    kF = 0.0
    kG = 0.05  # Gravity feed-forward
    
    # Position control constants
    POSITION_TOLERANCE = 0.5  # Position tolerance in units
    POSITION_CONVERSION_FACTOR = 0.01  # Convert rotations to meters
    
    # Soft limits for safety (in meters or encoder units)
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 1.2
    
    # Preset positions (in meters or encoder units)
    HOME_POSITION = 0.0
    LOW_POSITION = 0.3
    MEDIUM_POSITION = 0.6
    HIGH_POSITION = 1.0
    
    # Encoder configuration (if needed)
    ABSOLUTE_ENCODER_OFFSET = 0.0

# ================ End Effector Constants ================

class endEffectorConsts:
    """Constants for the end effector subsystem."""
    
    # ---- Coral mechanism ----
    # Current limits
    coralCurrentLimit = 30  # Amps
    CORAL_JAM_CURRENT_THRESHOLD = 25  # Amps
    
    # Distance thresholds (in mm)
    CORAL_DETECTION_THRESHOLD = 50  # Distance that indicates coral is present
    CORAL_STOP_THRESHOLD = 30  # Distance that indicates coral is in position
    
    # Motor speeds
    CORAL_INTAKE_SPEED = 0.7
    
    # ---- Algae mechanism ----
    # Current limits
    algaeRotCurrentLimit = 30  # Amps
    algaeIntakeCurrentLimit = 30  # Amps
    ALGAE_JAM_CURRENT_THRESHOLD = 25  # Amps
    
    # Encoder configuration
    ALGAE_ENCODER_OFFSET = 0.0
    
    # Position limits and presets (in degrees)
    ALGAE_MIN_ANGLE = -100.0
    ALGAE_MAX_ANGLE = 100.0
    ALGAE_POSITION_TOLERANCE = 5.0
    
    # Preset positions (in degrees)
    ALGAE_RETRACTED_POS = 0.0
    ALGAE_TOP_PICKUP_POS = 90.0
    ALGAE_BOTTOM_PICKUP_POS = -90.0
    
    # PID values for algae rotation
    ALGAE_KP = 0.1
    ALGAE_KI = 0.0
    ALGAE_KD = 0.005
    ALGAE_KF = 0.0
    
    # General safety thresholds
    CURRENT_WARNING_THRESHOLD = 30.0  # Amps
    CURRENT_CRITICAL_THRESHOLD = 40.0  # Amps
    ALGAE_INTAKE_SPEED = 0.7

# ================ Autonomous Constants ================

class autoConsts:
    """Constants for autonomous routines."""
    
    # Timeout values
    DEFAULT_TIMEOUT = 10.0  # seconds
    
    # PID values for trajectory following
    X_CONTROLLER_P = 1.0
    Y_CONTROLLER_P = 1.0
    ROTATION_CONTROLLER_P = 1.0
    
    # Common distances and speeds
    LEAVE_COMMUNITY_DISTANCE = 3.0  # meters
    DEFAULT_AUTO_SPEED = 1.0  # m/s