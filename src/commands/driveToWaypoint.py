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

    self.exPub = self.table.getDoubleTopic("ex").publish()
    self.eyPub = self.table.getDoubleTopic("ey").publish()
    self.etPub = self.table.getDoubleTopic("et").publish()

    self.xSpeed = self.ySpeed = self.tSpeed = 0

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
    
    tkp = 0.3
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
    # print(f"dt: {self.dt}, error: {self.tController.getError()}")

    self.dxPub.set(self.dx)
    self.dyPub.set(self.dy)
    self.dtPub.set(self.dt)

    self.exPub.set(self.xController.getError())
    self.eyPub.set(self.yController.getError())
    self.etPub.set(self.tController.getError())
    
    self.xSpeed = self.xController.calculate(self.dx)
    self.ySpeed = self.yController.calculate(self.dy)
    self.tSpeed = -self.tController.calculate(self.dt)

    self.xSpeed = max(min(self.xSpeed, constants.autoConsts.maxTranslationSpeed), -constants.autoConsts.maxTranslationSpeed)
    self.ySpeed = max(min(self.ySpeed, constants.autoConsts.maxTranslationSpeed), -constants.autoConsts.maxTranslationSpeed)
    self.tSpeed = max(min(self.tSpeed, constants.autoConsts.maxRotationSpeed), -constants.autoConsts.maxRotationSpeed)

    self.vxPub.set(self.xSpeed)
    self.vyPub.set(self.ySpeed)
    self.vtPub.set(self.tSpeed)

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.tSpeed, odometry.rotation())
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)

    # print(f"Position error: ({self.xController.getError():.2f}, {self.yController.getError():.2f})")
    # print(f"Rotation error: {math.degrees(self.tController.getError()):.1f} degrees")
    # print(f"Speeds: x={xSpeed:.2f}, y={ySpeed:.2f}, t={tSpeed:.2f}")
    
  def end(self, interrupted: bool):
    self.drivetrain.stopMotors()

  def inTollerance(self):
    if (abs(self.xSpeed) < self.precisionXY) and (abs(self.ySpeed) < self.precisionXY) and (abs(self.tSpeed) < self.precisionT):
      return True
    else:
      # print("Errors")
      # print(f"{abs(self.xController.getError()):.3f}, {self.precisionXY}")
      # print(f"{abs(self.yController.getError()):.3f}, {self.precisionXY}")
      # print(f"{abs(self.tController.getError()):.3f}, {self.precisionT}")
      return False
    
  def isFinished(self) -> bool:
    if self.inTollerance():
      print("bang ding ow (out toller)")
      return True
    return False