import rev
import math
import commands2

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
  
from wpilib import DriverStation, Field2d
from wpimath import controller
from wpimath.units import degreesToRadians

from constants import CANIDs

import constants

from phoenix6.hardware import CANcoder, Pigeon2

import ntcore

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.util import DriveFeedforwards

from wpilib import SmartDashboard


def lratio(angle):
    """converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    return (something/.5)*-math.pi

def deg2Rot2d(deg) -> Rotation2d:
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)

def ticks2radODOMETRY(something):
    # units are in rotations
    return something * 2* math.pi

def getSwerveModPos(rotEnc : CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    return SwerveModulePosition(
                                        # 2pi*r
        #                       gear ratio        in->m              wheel diameter                    pi
        (driveEnc.getPosition()/   6.75   )   *   0.31918580816,
        Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    )

def negateOdometry(pose: Pose2d):
  x = pose.X()
  y = pose.Y()
  r = pose.rotation()
  return Pose2d(Translation2d(-x, -y), r)

class DriveTrain(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()

    # create network table
    inst = ntcore.NetworkTableInstance.getDefault()
    self.table = inst.getTable("Swerve Table")

    self.FRlratio = self.table.getDoubleTopic("FR lratio").publish()
    self.robotPosXPub = self.table.getDoubleTopic("Position X").publish()
    self.robotPosYPub = self.table.getDoubleTopic("Position Y").publish()
    self.driveSpeedsPub = self.table.getStructTopic("Drive Speeds", ChassisSpeeds).publish()

    self.robotOdometryPosition = Pose2d()
    self.combinedPosition = Pose2d()
    self.currentPosition = Pose2d()

    # Drivetrain init 
    # Need to replace CAN ids with their respective
    # ids from constants.CANIDs

    self.backLeftRotation = rev.SparkMax(CANIDs.SwerveModuleRotationBL, rev.SparkMax.MotorType.kBrushless)
    self.backRightRotation = rev.SparkMax(CANIDs.SwerveModuleRotationBR, rev.SparkMax.MotorType.kBrushless)
    self.frontLeftRotation = rev.SparkMax(CANIDs.SwerveModuleRotationFL, rev.SparkMax.MotorType.kBrushless)
    self.frontRightRotation = rev.SparkMax(CANIDs.SwerveModuleRotationFR, rev.SparkMax.MotorType.kBrushless)

    self.backLeftDrive = rev.SparkMax(CANIDs.SwerveModuleDriveBL, rev.SparkMax.MotorType.kBrushless)
    self.backRightDrive = rev.SparkMax(CANIDs.SwerveModuleDriveBR, rev.SparkMax.MotorType.kBrushless)
    self.frontLeftDrive = rev.SparkMax(CANIDs.SwerveModuleDriveFL, rev.SparkMax.MotorType.kBrushless)
    self.frontRightDrive = rev.SparkMax(CANIDs.SwerveModuleDriveFR, rev.SparkMax.MotorType.kBrushless)

    # Set the configs
    self.backLeftRotationConfig = rev.SparkBaseConfig()
    self.backLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backLeftRotationConfig.smartCurrentLimit(20)
    self.backRightRotationConfig = rev.SparkBaseConfig()
    self.backRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backRightRotationConfig.smartCurrentLimit(20)
    self.frontLeftRotationConfig = rev.SparkBaseConfig()
    self.frontLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontLeftRotationConfig.smartCurrentLimit(20)
    self.frontRightRotationConfig = rev.SparkBaseConfig()
    self.frontRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontRightRotationConfig.smartCurrentLimit(20)

    self.backLeftDriveConfig = rev.SparkBaseConfig()
    self.backLeftDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backLeftDriveConfig.smartCurrentLimit(30)
    self.backRightDriveConfig = rev.SparkBaseConfig()
    self.backRightDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backRightDriveConfig.smartCurrentLimit(30)
    self.frontLeftDriveConfig = rev.SparkBaseConfig()
    self.frontLeftDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontLeftDriveConfig.smartCurrentLimit(30)
    self.frontRightDriveConfig = rev.SparkBaseConfig()
    self.frontRightDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontRightDriveConfig.smartCurrentLimit(30)
    
    self.backLeftRotation.configure(self.backLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.backRightRotation.configure(self.backRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontLeftRotation.configure(self.frontLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontRightRotation.configure(self.frontRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    self.backLeftDrive.configure(self.backLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.backRightDrive.configure(self.backRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontLeftDrive.configure(self.frontLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontRightDrive.configure(self.frontRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # Drive encoders

    self.frontRightDriveEnc = self.frontRightDrive.getEncoder()
    self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder()
    self.backRightDriveEnc = self.backRightDrive.getEncoder()
    self.backLeftDriveEnc = self.backLeftDrive.getEncoder()

    # Need to add correct CANcoder ids in constants.py

    self.FrightEnc = CANcoder(CANIDs.EncoderModuleRotationFR)
    self.FleftEnc = CANcoder(CANIDs.EncoderModuleRotationFL)
    self.BrightEnc = CANcoder(CANIDs.EncoderModuleRotationBR)
    self.BleftEnc = CANcoder(CANIDs.EncoderModuleRotationBL)

    # PID Setup (needs tuning) (Ideally we don't need to zero our encoders, Yay!)

    Kp = 2
    Ki = 0
    Kd = 0
    self.BleftPID = controller.PIDController(Kp,Ki,Kd)
    self.BleftPID.enableContinuousInput(-.5,.5)
    self.BleftPID.setSetpoint(0.0)
    self.BrightPID = controller.PIDController(Kp,Ki,Kd)
    self.BrightPID.enableContinuousInput(-.5,.5)
    self.BrightPID.setSetpoint(0.0)
    self.FleftPID = controller.PIDController(Kp,Ki,Kd)
    self.FleftPID.enableContinuousInput(-.5,.5)
    self.FleftPID.setSetpoint(0.0)
    self.FrightPID = controller.PIDController(Kp,Ki,Kd)
    self.FrightPID.enableContinuousInput(-.5,.5)
    self.FrightPID.setSetpoint(0.0)

    # Gyro init

    self.gyro = Pigeon2(CANIDs.pigeonID)
    self.gyro.set_yaw(0)

    # Kinematics (need to get back from design on exact measurments)

    lv = 0.3 #location value

    frontrightlocation = Translation2d(lv, lv) 
    frontleftlocation = Translation2d(lv, -lv) 
    backleftlocation = Translation2d(-lv, -lv)         
    backrightlocation = Translation2d(-lv, lv)    

    self.lastChassisSpeed = ChassisSpeeds(0,0,0)

    self.kinematics = SwerveDrive4Kinematics(
        frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
    )

    self.odometry = SwerveDrive4Odometry(
      self.kinematics,
      Rotation2d(),
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)

      ),

    ) 
    self.odometryHeadingOffset = Rotation2d(0)
    self.resetOdometry(Pose2d(0,0,0))

    self.field = Field2d()

    try:
      self.config = RobotConfig.fromGUISettings()
      AutoBuilder.configure(
        self.getPose,
        self.resetHarder,
        self.getSpeeds,
        self.ppRelativeDrive,
        PPHolonomicDriveController(
          constants.PathPlanner.translationConsts,
          constants.PathPlanner.rotationConsts
        ),
        self.config,
        self.shouldMirrorpath,
        self
      )
    
    except:
      raise ValueError("Failed to load PathPlanner config and configure AutoBuilder")
    
    SmartDashboard.putData("Field", self.field)

  def shouldMirrorpath(self):
    alliance = DriverStation.getAlliance()
    if alliance == alliance.kRed:
      return True
    else:
      return False

  def getModuleStates(self):
    return self.kinematics.toSwerveModuleStates(self.getChassisSpeed(), Translation2d(0, 0))

  def getSpeeds(self):
    return self.kinematics.toChassisSpeeds(self.getModuleStates())

  def resetPose(self, pose: Pose2d):
    self.odometry.resetPosition(self.gyro.getRotation2d, getSwerveModPos())

  def getPositions(self):
    return (
      getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
      getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
      getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
      getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
    )

  def resetOdometry(self, pose: Pose2d):
    self.gyro.reset()
    self.gyro.set_yaw(0)
    self.lastGyroAngleTime = 0
    self.lastGyroAngle = 0

    self.odometry.resetPosition(
      self.getGyroHeading(),
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
      ),
      pose
    )
    self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()
    self.field = Field2d()

  def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
    pose = self.getPose()
    newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
    self.odometry.resetPosition(
      pose.rotation() - self.odometryHeadingOffset,
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
      ),
      newPose
    )
    self.odometryHeadingOffset += dRot

  def getGyroHeading(self) -> Rotation2d:
    """Returns the heading of the robot, tries to be smart when gyro is disconnected

    :returns: the robot's heading as Rotation2d
    """
    return Rotation2d.fromDegrees(self.gyro.get_yaw().value_as_double)
  def resetHarder(self, initialPose = Pose2d()):
    self.gyro.set_yaw(0)

    self.odometry = SwerveDrive4Odometry(
      self.kinematics,
      deg2Rot2d(self.gyro.get_yaw().value_as_double),
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
      ),
      initialPose
    )
  
  def getPose(self):
    nonYPose = self.odometry.getPose()
    return negateOdometry(self.currentPosition)
  
  def shouldFlipPath(self): #checks if the alliance is red, therefore should mirror the path to account for being on opposite side of field
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed
  
  def getChassisSpeed(self) -> ChassisSpeeds:
    return self.lastChassisSpeed
  
  def updateOdometry(self) -> None: # weee neeeed thiiiis!
    # yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double - 90)

    # a = self.odometry.update( 
    #   yaw,
    #   (
    #     getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
    #     getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
    #     getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
    #     getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
    #   )
    # )

    yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double)
    #print(self.frontLeftDriveEnc.getPosition())
    self.robotOdometryPosition = self.odometry.update(
      yaw,
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
      )
    )
    robotRotationPose = self.odometry.getPose()
    # we get the rotation and translations seperatly as Pose2d objects and then combine them
    self.combinedPosition = Pose2d(x = self.robotOdometryPosition.x, y = self.robotOdometryPosition.y, rotation = robotRotationPose.rotation())

  def periodic(self) -> None:
    pose = self.odometry.update(
      self.getGyroHeading(),
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
      ),

    )
    self.currentPosition = negateOdometry(pose)
    self.field.setRobotPose(negateOdometry(self.currentPosition))
    self.robotPosXPub.set(self.getPose().X())
    self.robotPosYPub.set(self.getPose().Y())
    # self.updateOdometry()
#     print(
# f"""

# back rightr: {self.backRightRotation.getOutputCurrent()}
# back leftr: {self.backRightRotation.getOutputCurrent()}
# front rightr: {self.backRightRotation.getOutputCurrent()}
# front leftr: {self.backRightRotation.getOutputCurrent()}""")

  def resetMotors(self) -> None:
    pass # if we need it

  def manualDriveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None: #used in current implementation of robot.py as of 2/28
    self.lastChassisSpeed = speeds
    
    speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega)
    frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

    # bldSpeed = backLeft.speed
    # brdSpeed = backRight.speed
    # fldSpeed = frontLeft.speed
    # frdSpeed = frontRight.speed

    # frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
    # Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
    # frontRightOptimized = SwerveModuleState.optimize(frontRight,
    # Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
    # backLeftOptimized = SwerveModuleState.optimize(backLeft,
    # Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
    # backRightOptimized = SwerveModuleState.optimize(backRight,
    # Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

    frontLeft.optimize(Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
    frontRight.optimize(Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
    backLeft.optimize(Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
    backRight.optimize(Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

    # blrSpeed = -self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians()))
    # flrSpeed = -self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians()))
    # brrSpeed = -self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians()))
    # frrSpeed = -self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians()))
    # self.FRlratio.set(frrSpeed)
    self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians())))
    self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians())))
    self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians())))
    self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians())))

    self.backLeftDrive.set(backLeft.speed)
    # print(f"speedddddddd: {backLeft.speed}")
    self.backRightDrive.set(backRight.speed)
    self.frontLeftDrive.set(frontLeft.speed)
    self.frontRightDrive.set(frontRight.speed)
    # dSpeedList = [bldSpeed, brdSpeed, fldSpeed, frdSpeed]
    # rSpeedList = [blrSpeed, flrSpeed, brrSpeed, frrSpeed]


    # for i in range(len(dSpeedList)):
    #    if abs(dSpeedList[i])<0.5: #drive deadzone
    #       dSpeedList[i]=0

    # for i in range(len(rSpeedList)):
    #    if abs(rSpeedList[i])<0.5: #rotation deadzone
    #       rSpeedList[i]=0

    # self.backLeftRotation.set(rSpeedList[0])
    # self.frontLeftRotation.set(rSpeedList[1])
    # self.backRightRotation.set(rSpeedList[2])
    # self.frontRightRotation.set(rSpeedList[3])

    # self.backLeftDrive.set(dSpeedList[0])
    # self.backRightDrive.set(dSpeedList[1])
    # self.frontLeftDrive.set(dSpeedList[2])
    # self.frontRightDrive.set(dSpeedList[3])

    # print(dSpeedList)
    # print(rSpeedList)
    # print('\n')
  def ppRelativeDrive(self, speeds: ChassisSpeeds, ff):
    speeds = ChassisSpeeds(-speeds.vx * constants.driveConsts.autoScalingFactor, speeds.vy * constants.driveConsts.autoScalingFactor, speeds.omega)
    self.driveFromRelativeCoordinates(speeds, ff)
    self.driveSpeedsPub.set(speeds)
  
  def driveFromRelativeCoordinates(self, speeds: ChassisSpeeds, ff: DriveFeedforwards):
    """
    rotation = degreesToRadians(self.gyro.get_yaw().value_as_double)
    deltax = vx * math.cos(rotation) + vy * math.sin(rotation)
    deltay = vx * math.sin(rotation) - vy * math.cos(rotation)
    speeds = ChassisSpeeds(deltax, deltay, vt)
    self.manualDriveFromChassisSpeeds(speeds)
    """
    # robotRelativeSpeeds = ChassisSpeeds(vx, vy, vt)
    # targetSpeeds = robotRelativeSpeeds.discretize()
    # targetStates = self.kinematics.toSwerveModuleStates(targetSpeeds)
    # Get the current robot heading from the gyro
    currentAngle = self.getGyroHeading()
    
    # Create field-relative chassis speeds
    vx = speeds.vx
    vy = -speeds.vy
    vt = speeds.omega



    fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vx, vy, vt, currentAngle
    )
    
    # Convert to module states
    moduleStates = self.kinematics.toSwerveModuleStates(fieldRelativeSpeeds)
    
    # Desaturate wheel speeds
    maxSpeed = 4.1  # Same as in your other method
    frontLeft, frontRight, backLeft, backRight = SwerveDrive4Kinematics.desaturateWheelSpeeds(
        moduleStates, maxSpeed
    )
    
    # Optimize and set module states directly
    frontLeft.optimize(Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
    frontRight.optimize(Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
    backLeft.optimize(Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
    backRight.optimize(Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))
    
    # Set rotation motors
    self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians())))
    self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians())))
    self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians())))
    self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians())))
    
    # Set drive motors
    self.backLeftDrive.set(backLeft.speed)
    self.backRightDrive.set(backRight.speed)
    self.frontLeftDrive.set(frontLeft.speed)
    self.frontRightDrive.set(frontRight.speed)
    
    # Update lastChassisSpeed for odometry
    self.lastChassisSpeed = fieldRelativeSpeeds
    
  def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None: #not used in current robot.py implementation as of 2/28
    self.lastChassisSpeed = speeds

    # counter intuitive (should be this way plz don't change)
    Vx = speeds.vy
    Vy = speeds.vx

    speeds = ChassisSpeeds(-Vx, -Vy, -speeds.omega)
    moduleStates = self.kinematics.toSwerveModuleStates(speeds)

    # this can be changed
    maxModSpeed = 4.1

    frontLeft, frontRight, backLeft, backRight = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      moduleStates, 
      maxModSpeed
    )
    frontLeft.optimize(Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
    frontRight.optimize(Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
    backLeft.optimize(Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
    backRight.optimize(Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

    self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians())))
    self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians())))
    self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians())))
    self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians())))

    # probably fine at 13 (can change if needed)
    maxVoltage = 13

    blSpeed=-(backLeft.speed/maxModSpeed)*maxVoltage
    brSpeed=(backRight.speed/maxModSpeed)*maxVoltage
    flSpeed=(frontLeft.speed/maxModSpeed)*maxVoltage
    frSpeed=(frontRight.speed/maxModSpeed)*maxVoltage

    speedsList = [blSpeed, brSpeed, flSpeed, frSpeed]

    for i in speedsList:
       if i<0.3: #voltage deadzone
          i=0

    self.backLeftDrive.setVoltage(blSpeed)
    self.backRightDrive.setVoltage(brSpeed)
    self.frontLeftDrive.setVoltage(flSpeed)
    self.frontRightDrive.setVoltage(frSpeed)


  def stopMotors(self):
    self.frontLeftDrive.set(0)
    self.frontRightDrive.set(0)
    self.backLeftDrive.set(0)
    self.backRightDrive.set(0)

    self.frontLeftRotation.set(0)
    self.frontRightRotation.set(0)
    self.backLeftRotation.set(0)
    self.backRightRotation.set(0)