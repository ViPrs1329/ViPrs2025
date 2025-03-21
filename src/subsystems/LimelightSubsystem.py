import commands2
import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d
from wpimath.units import degreesToRadians

class LimelightSubsystem(commands2.Subsystem):
  def __init__(self):
    super().__init__()
    inst: ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()
    self.limelightTableLeft: ntcore.NetworkTable = inst.getTable("limelight-left")
    self.limelightTableRight: ntcore.NetworkTable = inst.getTable("limelight-right")
  
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

<<<<<<< HEAD
  def getTargetPoseInCameraSpace(self):
    botPoseArray = self.limelightTable.getEntry("targetpose_robotspace").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tx = botPoseArray[0]
    ty = botPoseArray[1]
    tz = botPoseArray[2]
=======
  def limelightLeftDetectsTag(self) -> bool :
    tv = self.limelightTableLeft.getNumber("tv")
    if tv == 1:
      return True
    elif tv == 0:
      return False
    else:
      raise ValueError(f"(should be 0 or 1) limelight left detects apriltags: {tv}")
>>>>>>> da98f6bf4e19280facadd865352460dc3b3c1a30

  def limelightRightDetectsTag(self) -> bool :
    tv = self.limelightTableRight.getNumber("tv")
    if tv == 1:
      return True
    elif tv == 0:
      return False
    else:
      raise ValueError(f"(should be 0 or 1) limelight left detects apriltags: {tv}")

  def getTargetPose(self) -> Pose3d | bool:
    botPoseArrayLeft = self.limelightTableLeft.getEntry("targetpose_botspace").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ltx = botPoseArrayLeft[0]
    lty = botPoseArrayLeft[1]
    ltz = botPoseArrayLeft[2]

    lroll = botPoseArrayLeft[3]
    lpitch = botPoseArrayLeft[4]
    lyaw = botPoseArrayLeft[5]

    botPoseArrayRight = self.limelightTableRight.getEntry("targetpose_botspace").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rtx = botPoseArrayRight[0]
    rty = botPoseArrayRight[1]
    rtz = botPoseArrayRight[2]

    rroll = botPoseArrayRight[3]
    rpitch = botPoseArrayRight[4]
    ryaw = botPoseArrayRight[5]

    if self.limelightLeftDetectsTag():
      if self.limelightRightDetectsTag():
        # both limelights detect april tags
        return (Pose3d(
          Translation3d(ltx, lty, ltz), 
          Rotation3d(
            degreesToRadians(lroll), 
            degreesToRadians(lpitch),
            degreesToRadians(lyaw)
          )
        ) + Pose3d(
          Translation3d(rtx, rty, rtz),
          Rotation3d(
            degreesToRadians(rroll),
            degreesToRadians(rpitch),
            degreesToRadians(ryaw)
          )
        )) * 0.5
      else:
        # limelight left detects tag but right doesn't
        return Pose3d(
          Translation3d(ltx, lty, ltz), 
          Rotation3d(
            degreesToRadians(lroll), 
            degreesToRadians(lpitch),
            degreesToRadians(lyaw)
          )
        )
    else:
      if self.limelightRightDetectsTag():
        # limelight right detects tag but left doesn't
        return Pose3d(
          Translation3d(rtx, rty, rtz),
          Rotation3d(
            degreesToRadians(rroll),
            degreesToRadians(rpitch),
            degreesToRadians(ryaw)
          )
        )
      else:
        # neither limelight detects april tags
        return False


  def targetExists(self):
    exists = self.limelightTableLeft.getNumber("tv", 0)
    if exists == 1:
      return True
    else:
      return False