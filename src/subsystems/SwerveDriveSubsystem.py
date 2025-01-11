import rev
import math
import commands2

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpimath import controller

from constants import CANIDs

from phoenix6.hardware import CANcoder, Pigeon2


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
        (driveEnc.getPosition()/6.75)*0.31918580816,
        Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    )

class DriveTrain(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    # Drivetrain init 
    # Need to replace CAN ids with their respective
    # ids from constants.CANIDs

    self.backLeftRotation = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushless)
    self.backRightRotation = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushless)
    self.frontLeftRotation = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)
    self.frontRightRotation = rev.SparkMax(8, rev.SparkMax.MotorType.kBrushless)

    self.backLeftDrive = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
    self.backRightDrive = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)
    self.frontLeftDrive = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
    self.frontRightDrive = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)

    # Set the configs
    self.backLeftRotationConfig = rev.SparkBaseConfig()
    self.backLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backRightRotationConfig = rev.SparkBaseConfig()
    self.backRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontLeftRotationConfig = rev.SparkBaseConfig()
    self.frontLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontRightRotationConfig = rev.SparkBaseConfig()
    self.frontRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

    self.backLeftDriveConfig = rev.SparkBaseConfig()
    self.backLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.backRightDriveConfig = rev.SparkBaseConfig()
    self.backRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontLeftDriveConfig = rev.SparkBaseConfig()
    self.frontLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.frontRightDriveConfig = rev.SparkBaseConfig()
    self.frontRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    
    self.backLeftRotation.configure(self.backLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.backRightRotation.configure(self.backRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontLeftRotation.configure(self.frontLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontRightRotation.configure(self.frontRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    self.backLeftDrive.configure(self.backLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.backRightDrive.configure(self.backRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontLeftDrive.configure(self.frontLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.frontRightDrive.configure(self.frontRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # Drive encoders

    self.frontRightDriveEnc = self.frontRightDrive.getAbsoluteEncoder()
    self.frontLeftDriveEnc = self.frontLeftDrive.getAbsoluteEncoder()
    self.backRightDriveEnc = self.backRightDrive.getAbsoluteEncoder()
    self.backLeftDriveEnc = self.backLeftDrive.getAbsoluteEncoder()

    # Need to add correct CANcoder ids in constants.py

    self.FrightEnc = CANcoder(CANIDs.EncoderModuleRotation4)
    self.FleftEnc = CANcoder(CANIDs.EncoderModuleRotation3)
    self.BrightEnc = CANcoder(CANIDs.EncoderModuleRotation2)
    self.BleftEnc = CANcoder(CANIDs.EncoderModuleRotation1)

    # PID Setup (needs tuning) (Ideally we don't need to zero our encoders, Yay!)

    Kp = 4
    self.BleftPID = controller.PIDController(Kp,0,0)
    self.BleftPID.enableContinuousInput(-.5,.5)
    self.BleftPID.setSetpoint(0.0)
    self.BrightPID = controller.PIDController(Kp,0,0)
    self.BrightPID.enableContinuousInput(-.5,.5)
    self.BrightPID.setSetpoint(0.0)
    self.FleftPID = controller.PIDController(Kp,0,0)
    self.FleftPID.enableContinuousInput(-.5,.5)
    self.FleftPID.setSetpoint(0.0)
    self.FrightPID = controller.PIDController(Kp,0,0)
    self.FrightPID.enableContinuousInput(-.5,.5)
    self.FrightPID.setSetpoint(0.0)

    # Gyro init

    self.gyro = Pigeon2(13)
    self.gyro.set_yaw(0)

    # Kinematics (need to get back from design on exact measurments)

    frontrightlocation = Translation2d(.381, .381) 
    frontleftlocation = Translation2d(.381, -.381) 
    backleftlocation = Translation2d(-.381, -.381)         
    backrightlocation = Translation2d(-.381, .381)    

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
      Pose2d()

    ) 

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
    return nonYPose
  
  def shouldFlipPath(self):
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed
  
  def getChassisSpeed(self) -> ChassisSpeeds:
    return self.lastChassisSpeed
  
  def updateOdometry(self) -> None: # weee neeeed thiiiis!
    yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double - 90)

    a = self.odometry.update(
      yaw,
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
      )
    )

    yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double)

    a = self.odometry.update(
      yaw,
      (
        getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
        getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
        getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
        getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
      )
    )

  def periodic(self) -> None:
    self.updateOdometry()

  def resetMotors(self) -> None:
    pass # if we need it

  def manualDriveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
    self.lastChassisSpeed = speeds
    
    speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega)
    frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

    self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians())))
    self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians())))
    self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians())))
    self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians())))

    self.backLeftDrive.set(-backLeft.speed)
    self.backRightDrive.set(backRight.speed)
    self.frontLeftDrive.set(frontLeft.speed)
    self.frontRightDrive.set(frontRight.speed)

  def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
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
    frontRight.optimize(Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

    self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeft.angle.radians())))
    self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeft.angle.radians())))
    self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRight.angle.radians())))
    self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRight.angle.radians())))

    # probably fine at 13 (can change if needed)
    maxVoltage = 13
    self.backLeftDrive.setVoltage(-(backLeft.speed/maxModSpeed)*maxVoltage)
    self.backRightDrive.setVoltage((backRight.speed/maxModSpeed)*maxVoltage)
    self.frontLeftDrive.setVoltage((frontLeft.speed/maxModSpeed)*maxVoltage)
    self.frontRightDrive.setVoltage((frontRight.speed/maxModSpeed)*maxVoltage)

  def stopMotors(self):
    self.frontLeftDrive.set(0)
    self.frontRightDrive.set(0)
    self.backLeftDrive.set(0)
    self.backRightDrive.set(0)

    self.frontLeftRotation.set(0)
    self.frontRightRotation.set(0)
    self.backLeftRotation.set(0)
    self.backRightRotation.set(0)