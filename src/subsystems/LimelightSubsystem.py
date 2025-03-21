import commands2
import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d
from wpimath.units import degreesToRadians

class LimelightSubsystem(commands2.Subsystem):
  def __init__(self):
    super().__init__()
    inst: ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()
    self.limelightTable: ntcore.NetworkTable = inst.getTable("limelight")
  
  def limelightPose2AdvantageScopePose(self, pose: Pose3d):
    translation = pose.translation()
    tx = translation.X()
    ty = translation.Y()
    tz = translation.Z()

    rotation = pose.rotation()
    roll = rotation.X()
    pitch = rotation.Y()
    yaw = rotation.Z()
    return Pose3d(
      Translation3d(tx, tz, -ty),
      Rotation3d(-yaw, roll, -pitch - 3.14159/2)
    )

  def getTargetPoseInCameraSpace(self):
    botPoseArray = self.limelightTable.getEntry("targetpose_robotspace").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tx = botPoseArray[0]
    ty = botPoseArray[1]
    tz = botPoseArray[2]

    roll = botPoseArray[3]
    pitch = botPoseArray[4]
    yaw = botPoseArray[5]

    return Pose3d(
      Translation3d(tx, ty, tz), 
      Rotation3d(
        degreesToRadians(roll), 
        degreesToRadians(pitch),
        degreesToRadians(yaw)
      )
    )

  def targetExists(self):
    exists = self.limelightTable.getNumber("tv", 0)
    if exists == 1:
      return True
    else:
      return False