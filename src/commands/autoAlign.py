import commands2
import wpilib
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.LedSubsystem import LED
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
import constants
import math
import ntcore

class AutoAlign(commands2.Command):
  def __init__(self, llSubsystem: LimelightSubsystem, drivetrain: DriveTrain, alignLocation: str, ledSubsystem: LED, timeout_seconds: float = constants.visionConsts.autoAlignTimeout):
    super().__init__()

    inst = ntcore.NetworkTableInstance.getDefault()
    self.table = inst.getTable("Auto Align")

    self.llSubsystem = llSubsystem
    self.drivetrain = drivetrain
    self.alignPosition = 0
    # if alignLocation == "left":
    #   self.alignPosition = constants.visionConsts.alignOffset
    # elif alignLocation == "right":
    #   self.alignPosition = -constants.visionConsts.alignOffset
    # else:
    #   raise ValueError(f"robot can't align to {alignLocation}. must be 'left' or 'right'")
    
    self.timeout_seconds = timeout_seconds
    self.timer = wpilib.Timer()

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
    self.timeElapsedPub = self.table.getDoubleTopic("Time Elapsed").publish()

    self.alignSide = alignLocation

    self.led = ledSubsystem

  def initialize(self):
    # Start the timer
    self.timer.reset()
    self.timer.start()

    if self.alignSide == "left":
      self.alignPosition = constants.visionConsts.alignOffset
    elif self.alignSide == "right":
      self.alignPosition = -constants.visionConsts.alignOffset
    else:
      raise ValueError(f"robot can't align to {self.alignSide}. must be 'left' or 'right'")

    xkp = 0.3
    xki = 0.08
    xkd = 0.0

    ykp = 0.4
    yki = 0.05
    ykd = 0.0
    self.xController = PIDController(xkp, xki, xkd)
    self.xController.setSetpoint(self.alignPosition)
    self.alignLocationXPub.set(self.alignPosition)

    self.yController = PIDController(ykp, yki, ykd)
    self.yController.setSetpoint(0.04)
    self.alignLocationYPub.set(self.yController.getSetpoint())
    
    tkp = 0.5
    tki = 0.07
    tkd = 0.0
    self.tController = PIDController(tkp, tki, tkd)
    self.tController.setSetpoint(0)
    self.alignLocationTPub.set(0)

    self.dx = self.dy = self.dt = self.vx = self.vy = self.vt = 1000

    self.led.changeStates(constants.RobotStates.aligning)

  def execute(self):
    # Update the time elapsed
    elapsed_time = self.timer.get()
    self.timeElapsedPub.set(elapsed_time)
    
    if self.llSubsystem.limelightLeftDetectsTag() or self.llSubsystem.limelightRightDetectsTag():
      try:
        targetPose = self.llSubsystem.getTargetPose()
        self.dx = targetPose.X()
        self.dy = targetPose.Y()
        self.dz = targetPose.Z()


        # self.dt = targetPose.rotation().Z()
        self.dt = targetPose.rotation().Y()
        # self.dt = targetPose.rotation().X()

        tagAngle = math.atan2(self.dx, self.dz)

        self.currentXPub.set(self.dx)
        self.currentYPub.set(self.dz)
        self.currentTPub.set(self.dt)
        
        self.vx = self.xController.calculate(self.dx)
        self.vy = self.yController.calculate(self.dz)
        self.vt = self.tController.calculate(self.dt)

        self.speedYPub.set(self.vy)

        # speeds = ChassisSpeeds(-xSpeed, -ySpeed, -tSpeed)
        speeds = ChassisSpeeds(-self.vx, -self.vy, -self.vt)
        # print(f"dx: {self.dx}, setPoint: {self.alignPosition}, tSpeed: {tSpeed}, dy: {self.dy}, dt: {self.dt}")
        self.dXPub.set(abs(self.dx - self.alignPosition))
        self.dYPub.set(abs(self.dz - self.yController.getSetpoint()))
        self.dTPub.set(abs(self.dt))
        # self.drivetrain.driveFromRelativeCoordinates(ySpeed, xSpeed, 0)
        self.drivetrain.driveFromRelativeCoordinates(speeds, None)
      except:

        self.cancel()
    else:
      print("sum ting wong (no RIMEright deTECted)")
    
  def end(self, interrupted: bool):
    # Stop the timer
    self.timer.stop()

  def inTollerance(self):
    if (abs(self.vx) < 0.02) and (abs(self.vy) < 0.02) and (abs(self.vt) < 0.05):
      return True
    else:
      return False
    
  def isFinished(self) -> bool:
    # Check for timeout
    if self.timer.get() >= self.timeout_seconds:
      print(f"AUTO ALIGN TIMEOUT: Exceeded {self.timeout_seconds} seconds")
      return True
    
    if (not self.llSubsystem.limelightLeftDetectsTag()) and (not self.llSubsystem.limelightRightDetectsTag()):
      print("wi tu lo (out no tag)")
      return True
    
    if self.inTollerance():
      print("bang ding ow (out toller)")
      self.led.changeStates(constants.RobotStates.aligned)
      return True
    
    return False

