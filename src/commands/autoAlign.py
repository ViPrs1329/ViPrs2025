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
    self.speedYPub = self.table.getDoubleTopic("Speed Y").publish()
    self.dXPub = self.table.getDoubleTopic("dx").publish()
    self.dYPub = self.table.getDoubleTopic("dy").publish()
    self.dTPub = self.table.getDoubleTopic("dt").publish()
  def initialize(self):

    xkp = 0.4
    xki = 0.04
    xkd = 0.06

    ykp = 0.2
    yki = 0.1
    ykd = 0.1
    self.xController = PIDController(xkp, xki, xkd)
    self.xController.setSetpoint(self.alignPosition)
    self.alignLocationXPub.set(self.alignPosition)

    self.yController = PIDController(ykp, yki, ykd)
    self.yController.setSetpoint(0.02)
    self.alignLocationYPub.set(self.yController.getSetpoint())
    
    tkp = 0.7
    tki = 0.15
    tkd = 0.1
    self.tController = PIDController(tkp, tki, tkd)
    self.tController.setSetpoint(0)
    self.alignLocationTPub.set(0)

    self.dx = self.dy = self.dt = 1000

  def execute(self):
    if self.llSubsystem.limelightLeftDetectsTag() or self.llSubsystem.limelightRightDetectsTag():
      try:
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

        self.speedYPub.set(ySpeed)

        speeds = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
        # print(f"dx: {self.dx}, setPoint: {self.alignPosition}, tSpeed: {tSpeed}, dy: {self.dy}, dt: {self.dt}")
        self.dXPub.set(abs(self.dx - self.alignPosition))
        self.dYPub.set(abs(self.dz - self.yController.getSetpoint()))
        self.dTPub.set(abs(self.dt))
        # self.drivetrain.driveFromRelativeCoordinates(ySpeed, xSpeed, 0)
        self.drivetrain.driveFromRelativeCoordinates(ySpeed, xSpeed, tSpeed)
      except:

        self.cancel()
    else:
      print("sum ting wong (no RIMEright deTECted)")
    
  def end(self, interrupted: bool):
    pass

  def inTollerance(self):
    if (abs(self.dx - self.alignPosition) < 0.02) and (abs(self.dz - self.yController.getSetpoint()) < 0.02) and (abs(self.dt) < 0.05):
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
