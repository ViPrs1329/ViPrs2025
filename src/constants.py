import numpy
import math

class controller:
    scale = 0.5
    tscale = 1
    XYdeadzone = 0.03
    Tdeadzone = 0.1
    slowDriveScale = 0.5

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def m2in(m):
        return 39.37 * m
    
    def rev2rad(rev):
        return rev * 2 * numpy.pi

    def rad2rev(radians):
        return radians / (2 * numpy.pi)
    
class CANIDs:
    # Swerve CAD IDs
    SwerveModuleDrive1 = 10    # 
    SwerveModuleRotation1 = 11 # 
    SwerveModuleDrive2 = 12    # 
    SwerveModuleRotation2 = 13 # 
    SwerveModuleDrive3 = 4     # 
    SwerveModuleRotation3 = 5  # 
    SwerveModuleDrive4 = 8     # 
    SwerveModuleRotation4 = 9  # 
    
    # Elevator motors
    ElevatorLeft = 15
    ElevatorRight = 16

    # Encoders
    EncoderModuleRotation1 = 20  # Back Left
    EncoderModuleRotation2 = 17  # Back Right
    EncoderModuleRotation3 = 19  # Front Left
    EncoderModuleRotation4 = 18  # Front Right
    
    # Gyro
    Gyro = 21

    # End effector IDs
    CoralLeft = 6
    CoralRight = 2
    AlgaeArm = 3
    AlgaeIntake = 7
    canRange0 = 22
    canRange1 = 23

class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    # Physical characteristics
    wheelDiameter = 4.0  # inches
    wheelCircumference = wheelDiameter * math.pi
    
    # Gear ratios
    driveGearRatio = 6.75
    
    # Drive encoder conversion factors
    driveEncoderPositionFactor = wheelCircumference / driveGearRatio
    driveEncoderVelocityFactor = driveEncoderPositionFactor / 60.0  # per minute to per second

    # Swerve drive CANcoder offset values
    # TODO: Find the correct offsets for the CANcoders
    frontLeftCANcoderOffset = 0.0
    frontRightCANcoderOffset = 0.0
    backLeftCANcoderOffset = 0.0
    backRightCANcoderOffset = 0.0
    
    # Robot dimensions
    wheelBase = 0.762  # meters (distance between front and back wheels)
    trackWidth = 0.762  # meters (distance between left and right wheels)
    
    # PID values for rotation control
    kP = 2.0  # Proportional gain
    kI = 0.0  # Integral gain
    kD = 0.1  # Derivative gain
    
    # Current limits
    driveCurrentLimit = 40
    rotationCurrentLimit = 20
    
    # Motor inversion flags
    invertFrontLeftDrive = False
    invertFrontRightDrive = False
    invertBackLeftDrive = False
    invertBackRightDrive = False
    
    invertFrontLeftRotation = False
    invertFrontRightRotation = False
    invertBackLeftRotation = False
    invertBackRightRotation = False
    
    # Deadzones
    driveDeadzone = 0.05
    rotationDeadzone = 0.05
    
    # Autonomous settings
    autonomousTime = 5  # seconds

    # Gyro
    invertGyro = False

class intakeConsts:
    intakeSpeed = 0.5
    algaeIntakeSpeed = 0.5
    algaeArmAngles = [0, 0, 0, 0, 0]
    algaeThresholdCurrent = 1
    coralDetectionThreshold = 0.05

class elevatorConsts:
    # heights for ground, L1, L2, L3, and L4
    elevatorHeights = [0, 1, 2, 3, 4]
    verticalOffset = 2 #inches of clearance above target branches


class reefConsts:
    #information about the reef for defining elevator levels
    # [level index, maximum height in inches, pitch in degrees from horizontal, roll in degrees from upright]
    L0 = [0, 0, 90, 0]
    L1 = [1, 18, 0, 90]
    L2 = [2, 31.875, 35, 0]
    L3 = [3, 47.625, 35, 0]
    L4 = [4, 72, 90, 0]
    reefLevels = [L0, L1, L2, L3, L4]