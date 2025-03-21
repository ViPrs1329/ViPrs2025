import commands2
import wpilib
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.SwerveDriveSubsystem import DriveTrain
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
import constants

class AutoAlign(commands2.Command):
  def __init__(self, llSubsystem: LimelightSubsystem, drivetrain: DriveTrain):
    super().__init__()
    self.llSubsystem = llSubsystem
    self.drivetrain = drivetrain
    kp = 0.5
    ki = 0
    kd = 0
    self.xController = PIDController(kp, ki, kd)
    self.xController.setSetpoint(0)
    self.yController = PIDController(kp, ki, kd)
    self.yController.setSetpoint(0)
    
    tkp = 0.5
    tki = 0
    tkd = 0
    self.tController = PIDController(tkp, tki, tkd)
    self.tController.setSetpoint(0)

    self.dx = self.dy = self.dt = 1000


  def initialize(self):
    pass

  def execute(self):
    targetPose = self.llSubsystem.getTargetPose()
    self.dx = targetPose.X()
    self.dy = targetPose.Y()
    self.dt = targetPose.rotation().Z()
    xSpeed = self.xController.calculate(self.dx, 0)
    ySpeed = self.yController.calculate(self.dy, 0)
    tSpeed = self.tController.calculate(self.dt, 0)
    speeds = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)
    
  def end(self, interrupted: bool):
    pass

  def inTollerance(self):
    if self.dx < 0.02 and self.dy < 0.02 and self.dt < 0.1:
      return True
    else:
      return False
    
  def isFinished(self) -> bool:
    if (not self.llSubsystem.limelightLeftDetectsTag()) and (not self.llSubsystem.limelightRightDetectsTag()):
      return True
    if self.inTollerance():
      return True
