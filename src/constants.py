
import numpy

class controller:
    scale = 0.5
    tscale = 1
    XYdeadzone = 0.03
    Tdeadzone = 0.1

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * numpy.pi

    def rad2rev(radians):
        return radians / (2 * numpy.pi)
    
    

class CANIDs:
    # Swerve CAN IDs
    SwerveModuleDrive1 = 11
    SwerveModuleRotation1 = 10
    SwerveModuleDrive2 = 13
    SwerveModuleRotation2 = 12
    SwerveModuleDrive3 = 5
    SwerveModuleRotation3 = 4
    SwerveModuleDrive4 = 9
    SwerveModuleRotation4 = 8

    # Swerve CAN Encoders
    EncoderModuleRotation1 = 20
    EncoderModuleRotation2 = 17
    EncoderModuleRotation3 = 19
    EncoderModuleRotation4 = 18

    # Pigeon CAN ID
    PigeonID = 21


    # Elevator CAN IDs
    ElevatorLeftID = 15 # Verify 
    ElevatorRightID = 16 # Verify

    # End Effector CAN IDs
    EECoralLeftID = 24 # Verify
    EECoralRightID = 25 # Verify
    EECoralInSensorID = 26 # Verify
    EECoralStopSensorID = 27 # Verify

    EEAlgaeArmRotationID = 28 # Verify
    EEAlgaeIntakeID = 29 # Verify
    


class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    wheelDiameter = 4
    currentLimit = 10


class endEffectorConsts:
    coralCurrentLimit = 20
    algaeRotCurrentLimit = 20
    algaeIntakeCurrentLimit = 20
    
    # LaserCAN thresholds (in mm)
    CORAL_DETECTION_THRESHOLD = 50  # Distance that indicates coral is present
    CORAL_STOP_THRESHOLD = 30  # Distance that indicates coral is in position
    
    # Intake speeds
    CORAL_INTAKE_SPEED = 0.7

    # Existing constants for coral mechanism
    coralCurrentLimit = 20
    algaeRotCurrentLimit = 20
    algaeIntakeCurrentLimit = 20
    
    # LaserCAN thresholds (in mm)
    CORAL_DETECTION_THRESHOLD = 50  # Distance that indicates coral is present
    CORAL_STOP_THRESHOLD = 30  # Distance that indicates coral is in position
    
    # Intake speeds
    CORAL_INTAKE_SPEED = 0.7
    ALGAE_INTAKE_SPEED = 0.7
    
    # Algae mechanism constants
    ALGAE_ENCODER_OFFSET = 0.0  # Adjust based on your mechanism's zero position
    
    # Algae positions (in degrees)
    ALGAE_RETRACTED_POS = 0.0
    ALGAE_TOP_PICKUP_POS = 90.0
    ALGAE_BOTTOM_PICKUP_POS = -90.0
    
    # Position tolerance (in degrees)
    ALGAE_POSITION_TOLERANCE = 5.0
    
    # PID values for algae rotation
    ALGAE_KP = 0.1
    ALGAE_KI = 0.0
    ALGAE_KD = 0.005
    ALGAE_KF = 0.0

# Add these to your constants.py file in the elevatorConsts class

class elevatorConsts:
    # General elevator constants
    currentLimit = 30  # Current limit in amps
    
    # PID Control Constants
    kP = 0.1           # Proportional gain
    kI = 0.0           # Integral gain 
    kD = 0.005         # Derivative gain
    kF = 0.0           # Feedforward gain
    
    # Position control constants
    POSITION_TOLERANCE = 1.0  # Position tolerance in encoder counts
    
    # Soft limits for safety
    MIN_HEIGHT = 0.0   # Minimum safe height
    MAX_HEIGHT = 100.0  # Maximum safe height (adjust based on your mechanism)
    
    # Preset positions - adjust these based on your specific game requirements
    # These are in encoder counts (or converted units)
    HOME_POSITION = 0.0     # Fully retracted/stowed position
    LOW_POSITION = 20.0     # Low scoring position
    MEDIUM_POSITION = 50.0  # Medium scoring position
    HIGH_POSITION = 95.0    # High scoring position
    
    # You might have other specific positions for your game
    # PICKUP_POSITION = 15.0  # Position for picking up game pieces
    # HANDOFF_POSITION = 30.0 # Position for handing off to another mechanism



class sensorConsts:
    pass