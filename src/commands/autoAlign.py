import commands2
import wpilib
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.SwerveDriveSubsystem import DriveTrain
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
import constants
import math
import ntcore

class AutoAlign(commands2.Command):
  def __init__(self, llSubsystem: LimelightSubsystem, drivetrain: DriveTrain, alignLocation: str):
    super().__init__()

    inst = ntcore.NetworkTableInstance.getDefault()
    self.table = inst.getTable("Auto Align")

    self.llSubsystem = llSubsystem
    self.drivetrain = drivetrain
    self.alignPosition = 0
    if alignLocation == "left":
      self.alignPosition = constants.visionConsts.alignOffset
    elif alignLocation == "right":
      self.alignPosition = -constants.visionConsts.alignOffset
    else:
      raise ValueError(f"robot can't align to {alignLocation}. must be 'left' or 'right'")
    
    self.alignLocationXPub = self.table.getDoubleTopic("Align Location X").publish()
    self.alignLocationYPub = self.table.getDoubleTopic("Align Location Y").publish()
    self.alignLocationTPub = self.table.getDoubleTopic("Align Location T").publish()
    self.currentXPub = self.table.getDoubleTopic("Current X").publish()
    self.currentYPub = self.table.getDoubleTopic("Current Y").publish()
    self.currentTPub = self.table.getDoubleTopic("Current T").publish()

    xkp = 0.6
    xki = 0.048
    xkd = 0.03

    ykp = 0.5
    yki = 0.06
    ykd = 0.08
    self.xController = PIDController(xkp, xki, xkd)
    self.xController.setSetpoint(self.alignPosition)
    self.alignLocationXPub.set(self.alignPosition)

    self.yController = PIDController(ykp, yki, ykd)
    self.yController.setSetpoint(0)
    self.alignLocationYPub.set(0.5)
    
    tkp = 0.7
    tki = 0.1
    tkd = 0.1
    self.tController = PIDController(tkp, tki, tkd)
    self.tController.setSetpoint(0)
    self.alignLocationTPub.set(0)

    self.dx = self.dy = self.dt = 1000


  def initialize(self):
    pass

  def execute(self):
    if self.llSubsystem.limelightLeftDetectsTag() or self.llSubsystem.limelightRightDetectsTag():
      targetPose = self.llSubsystem.getTargetPose()
      self.dx = targetPose.X()
      self.dy = targetPose.Y()
      self.dz = targetPose.Z()


      # self.dt = targetPose.rotation().Z()
      self.dt = targetPose.rotation().Y()
      # self.dt = targetPose.rotation().X()

      self.currentXPub.set(self.dx)
      self.currentYPub.set(self.dz)
      self.currentTPub.set(self.dt)
      
      xSpeed = self.xController.calculate(self.dx)
      ySpeed = self.yController.calculate(self.dz)
      tSpeed = -self.tController.calculate(self.dt)
      speeds = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
      # print(f"dx: {self.dx}, setPoint: {self.alignPosition}, tSpeed: {tSpeed}, dy: {self.dy}, dt: {self.dt}")
      print(type(self.alignPosition))
      print(f"dx: {abs(self.dx - self.alignPosition)} dy; {abs(self.dz)} dt: {abs(self.dt)}")
      # self.drivetrain.driveFromRelativeCoordinates(ySpeed, xSpeed, 0)
      self.drivetrain.driveFromRelativeCoordinates(ySpeed, xSpeed, tSpeed)
    else:
      print("sum ting wong (no RIMEright deTECted)")
    
  def end(self, interrupted: bool):
    pass

  def inTollerance(self):
    if (abs(self.dx - self.alignPosition) < 0.02) and (abs(self.dz) < 0.02) and (abs(self.dt) < 0.05):
      return True
    else:
      return False
    
  def isFinished(self) -> bool:
    if (not self.llSubsystem.limelightLeftDetectsTag()) and (not self.llSubsystem.limelightRightDetectsTag()):
      print("wi tu lo (out no tag)")
      return True
    if self.inTollerance():
      print("bang ding ow (out toller)")
      return True
    return False
