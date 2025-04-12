import commands2
import wpilib
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.SwerveDriveSubsystem import DriveTrain
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d
import constants
import math
import ntcore

class DriveToWaypoint(commands2.Command):
  def __init__(self, drivetrain: DriveTrain, odometrySupplier: callable, waypoint: Pose2d, precisionXY: float, precisionT: float):
    super().__init__()

    inst = ntcore.NetworkTableInstance.getDefault()
    self.table = inst.getTable("Waypoint")

    self.drivetrain = drivetrain
    self.odSupplier: callable = odometrySupplier
    self.targetLoc = waypoint

    self.targetPub = self.table.getStructTopic("target pose", Pose2d).publish()

    self.precisionXY = precisionXY
    self.precisionT = precisionT

  def initialize(self):

    xkp = 0.3
    xki = 0.0
    xkd = 0.0

    ykp = 0.3
    yki = 0.0
    ykd = 0.0
    self.xController = PIDController(xkp, xki, xkd)
    self.xController.setSetpoint(self.targetLoc.X())

    self.yController = PIDController(ykp, yki, ykd)
    self.yController.setSetpoint(self.targetLoc.Y())
    
    tkp = 0.1
    tki = 0.0
    tkd = 0.0
    self.tController = PIDController(tkp, tki, tkd)
    self.tController.setSetpoint(self.targetLoc.rotation().radians())
    self.tController.enableContinuousInput(0, 2 * math.pi)
    self.dx = self.dy = self.dt = 1000

  def execute(self):
    odometry: Pose2d = self.odSupplier()
    self.dx = odometry.X()
    self.dy = odometry.Y()

    self.dt = odometry.rotation().radians()
    
    xSpeed = self.xController.calculate(self.dx)
    ySpeed = self.yController.calculate(self.dy)
    tSpeed = self.tController.calculate(self.dt)

    speeds = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)
    
  def end(self, interrupted: bool):
    pass

  def inTollerance(self):
    if (abs(self.dx - self.xController.getSetpoint()) < self.precisionXY) and (abs(self.dy - self.yController.getSetpoint()) < self.precisionXY) and (abs(self.dt - self.tController.getSetpoint()) < self.precisionT):
      return True
    else:
      return False
    
  def isFinished(self) -> bool:
    if self.inTollerance():
      print("bang ding ow (out toller)")
      return True
    return False