
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

class elevatorConsts:
    currentLimit = 10



class sensorConsts:
    pass