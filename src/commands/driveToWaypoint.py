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

    self.targetPub = self.table.getStructTopic("target pose", Pose2d).publish().set(self.targetLoc)

    self.dxPub = self.table.getDoubleTopic("dx").publish()
    self.dyPub = self.table.getDoubleTopic("dy").publish()
    self.dtPub = self.table.getDoubleTopic("dt").publish()

    self.vxPub = self.table.getDoubleTopic("vx").publish()
    self.vyPub = self.table.getDoubleTopic("vy").publish()
    self.vtPub = self.table.getDoubleTopic("vt").publish()

    self.precisionXY = precisionXY
    self.precisionT = precisionT

  def initialize(self):

    xkp = 0.1
    xki = 0.0
    xkd = 0.0

    ykp = 0.1
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
    self.tController.enableContinuousInput(-math.pi, math.pi)
    self.dx = self.dy = self.dt = 1000

  def execute(self):
    odometry: Pose2d = self.odSupplier()
    self.dx = odometry.X()
    self.dy = odometry.Y()

    self.dt = odometry.rotation().radians()
    print(f"dt: {self.dt}, error: {self.tController.getError()}")

    self.dxPub.set(self.dx)
    self.dyPub.set(self.dy)
    self.dtPub.set(self.dt)
    
    xSpeed = self.xController.calculate(self.dx)
    ySpeed = self.yController.calculate(self.dy)
    tSpeed = -self.tController.calculate(self.dt)

    self.vxPub.set(xSpeed)
    self.vyPub.set(ySpeed)
    self.vtPub.set(tSpeed)

    speeds = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)
    
  def end(self, interrupted: bool):
    pass

  def inTollerance(self):
    if (abs(self.xController.getError()) < self.precisionXY) and (abs(self.yController.getError()) < self.precisionXY) and (abs(self.tController.getError()) < self.precisionT):
      return True
    else:
      return False
    
  def isFinished(self) -> bool:
    if self.inTollerance():
      print("bang ding ow (out toller)")
      return True
    return False