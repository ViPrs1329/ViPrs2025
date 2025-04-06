
import numpy
import math

from pathplannerlib.config import PIDConstants

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
    
    def rot2in(rot):
        return rot * 0.34375
    
    def in2rot(inches):
        return inches / 0.34375
    
    def rot2angAlgae(rot):
        return rot * 3.141592 * 2
    
    

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

class autoConsts:
    autoDriveDistance = 4.0 # feet travelled for simple DriveDistance command


class intakeConsts:
    intakeSpeed = 0.2
    algaeIntakeSpeed = 0.6
    algaeArmFloorIntakeAngle = math.pi/4 # 45 deg
    algaeArmReefIntakeAngle = 3*math.pi/4 # 135 deg
    algaeThresholdCurrent = 1
    algaeZeroPosition = 0.0  # Adjust this value based on your arm's actual zero position
    algaeMaxPosition = 41.0  # Adjust this value based on your arm's actual max position   

    # 5cm for the canrange
    coralDetectionThreshold = 0.1
    algaeCurrentThreshold = 15
    algaeStoredSetpoint = 1.4
class elevatorConsts:
    # heights for ground, L1, L2, L3, and L4
    elevatorHeights = [1, 2, 3, 4]
    verticalOffset = -25.5 #inches of clearance above target branches

class reefConsts:
    #information about the reef for defining elevator levels
    # [level index, maximum height in inches, pitch in degrees from horizontal, roll in degrees from upright]
    L1 = [1, 0, 90, 0]
    L2 = [2, 31, 0, 90]
    L3 = [3, 47, 0, 0] # [2, 31.875, 35, 0]
    L4 = [4, 66.5, 0, 0] # [3, 35, 35, 0]
    # L4 = [4, 72, 90, 0]
    reefLevels = [L1, L2, L3, L4]

class sensorConsts:
    pass

class visionConsts:
    alignOffset = 0.156

class RobotStates:
    noTag = 1
    Tag = 2
    aligning = 3
    aligned = 4

class PathPlanner:
    translationConsts = PIDConstants(0.02, 0, 0)
    rotationConsts = PIDConstants(0.3, 0, 0)