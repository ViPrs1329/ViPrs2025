import commands2, constants
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from commands.driveToWaypoint import DriveToWaypoint
from subsystems.SwerveDriveSubsystem import DriveTrain
import ntcore

class Waypoint(commands2.Subsystem):
  def __init__(self, drivetrain: DriveTrain) -> None:
    super().__init__()
    self.queue: list[commands2.Command] = []
    self.drivetrain = drivetrain
    self.odSupplier: callable = drivetrain.getPose
    self.startingPoint = Pose2d(0, 0, Rotation2d())
    self.resetOdometry = drivetrain.resetOdometry

    self.table = ntcore.NetworkTableInstance.getDefault().getTable("waypoint table")
  def reset(self):
    self.queue.clear()

  def addWaypoint(self, waypoint: Pose2d, precisionXY: float = 0.05, precisionT: float = 0.1):
    x = 2 * self.startingPoint.X() - waypoint.X()
    y = 2 * self.startingPoint.Y() - waypoint.Y()
    r = waypoint.rotation()
    waypoint = Pose2d(x, y, r)
    self.queue.append(
      DriveToWaypoint(
        self.drivetrain,
        self.odSupplier,
        waypoint,
        precisionXY,
        precisionT
      )
    )

  def addCommand(self, command: commands2.Command):
    self.queue.append(command)

  def setStartingPose(self, startingPoint: Pose2d):
    self.startingPoint = startingPoint

  def getStartingPose(self):
    return self.startingPoint

  def getAutonomousCommand(self):
    # the "*" unpacks a list into seperate arguments
    return commands2.SequentialCommandGroup(
      commands2.InstantCommand(
        lambda: self.resetOdometry(self.startingPoint)
      ),
      *self.queue,
      commands2.InstantCommand(
        self.drivetrain.stopMotors
      )
    ) 

  def periodic(self):
    pass