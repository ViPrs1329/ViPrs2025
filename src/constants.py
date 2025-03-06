
import numpy

class controller:
    scale = 0.5
    tscale = 1
    XYdeadzone = 0.03
    Tdeadzone = 0.1
    slowDriveScale = 0.5

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * numpy.pi

    def rad2rev(radians):
        return radians / (2 * numpy.pi)
    
    

class CANIDs:
    # Swerve CAD IDs
    SwerveModuleDrive1 = 11
    SwerveModuleRotation1 = 10
    SwerveModuleDrive2 = 13
    SwerveModuleRotation2 = 12
    SwerveModuleDrive3 = 5
    SwerveModuleRotation3 = 4
    SwerveModuleDrive4 = 9
    SwerveModuleRotation4 = 8
    
    # Elevator motors
    ElevatorLeft = 15
    ElevatorRight = 16

    # Encoders

    EncoderModuleRotation1 = 20
    EncoderModuleRotation2 = 17
    EncoderModuleRotation3 = 19
    EncoderModuleRotation4 = 18

    #end effector ids
    CoralLeft = 100
    CoralRight = 100
    AlgaeArm = 100
    AlgaeIntake = 100
    canRange1 = 100
    canRange2 = 100

class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    wheelDiameter = 4
    autonomousTime = 5000


class intakeConsts:
    intakeSpeed = 0.5
    algaeIntakeSpeed = 0.5
    algaeArmAngles = [0, 0, 0, 0, 0]
    algaeThresholdCurrent = 1
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
    L3 = [3, 47.675, 35, 0]
    L4 = [4, 72, 90, 0]
    reefLevels = [L0, L1, L2, L3, L4]


class sensorConsts:
    pass