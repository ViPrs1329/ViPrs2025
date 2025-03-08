
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
    SwerveModuleDriveFL = 10
    SwerveModuleRotationFL = 11
    SwerveModuleDriveBL = 12
    SwerveModuleRotationBL = 13
    SwerveModuleDriveBR = 4
    SwerveModuleRotationBR = 5
    SwerveModuleDriveFR = 8
    SwerveModuleRotationFR = 9
    
    # Elevator motors
    ElevatorLeft = 15
    ElevatorRight = 16

    # Encoders

    EncoderModuleRotationFL = 19
    EncoderModuleRotationBL = 20
    EncoderModuleRotationBR = 17
    EncoderModuleRotationFR = 18

    # pigeon
    pigeonID = 21

    #end effector ids
    CoralLeft = 6
    CoralRight = 2
    AlgaeArm = 3
    AlgaeIntake = 7
    CanRangeFunnel = 22
    CanRangeEE = 23


class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    wheelDiameter = 4


class intakeConsts:
    intakeSpeed = 0.5
    algaeIntakeSpeed = 0.5
    algaeArmAngles = [0, 0, 0, 0, 0]
    algaeThresholdCurrent = 1

    # 5cm for the canrange
    coralDetectionThreshold = 0.05
class elevatorConsts:
    # heights for ground, L1, L2, L3, and L4
    elevatorHeights = [0, 1, 2, 3, 4]



class sensorConsts:
    pass