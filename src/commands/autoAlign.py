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
    self.alignPosition = constants.visionConsts.alignOffset
    # Store the requested alignment side rather than computing offset now
    self.alignSide = alignLocation  # "left" or "right"
    
    # Network tables for debugging
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
    self.relativePosePub = self.table.getDoubleTopic("Relative Pose").publish()
    
  def initialize(self):
    # PID Controllers
    xkp = 0.4
    xki = 0.02
    xkd = 0.06
    self.xController = PIDController(xkp, xki, xkd)
    
    ykp = 0.4
    yki = 0.02
    ykd = 0.2
    self.yController = PIDController(ykp, yki, ykd)
    
    tkp = 0.7
    tki = 0.0
    tkd = 0.1
    self.tController = PIDController(tkp, tki, tkd)
    
    # Default setpoint values - will be updated in first execute()
    self.alignLocationXPub.set(0)
    self.alignLocationYPub.set(0.02)  # Default forward distance
    self.alignLocationTPub.set(0)

    self.dx = self.dy = self.dt = 1000
    
    # Wait to set actual setpoints until we get tag data in execute()
    self.setpointsInitialized = False

  def execute(self):
    if self.llSubsystem.limelightLeftDetectsTag() or self.llSubsystem.limelightRightDetectsTag():
      try:
        targetPose = self.llSubsystem.getTargetPose()
        self.dx = targetPose.X()
        self.dy = targetPose.Y()
        self.dz = targetPose.Z()
        
        # Get the target's rotation - this tells us its orientation
        target_rot_y = targetPose.rotation().Y()
        self.dt = target_rot_y
        
        # Determine tag orientation angle (in radians)
        tag_angle = math.atan2(self.dx, self.dz)
        self.relativePosePub.set(tag_angle)
        
        # Initialize setpoints on first detection
        if not self.setpointsInitialized:
          offset_amount = constants.visionConsts.alignOffset
          
          # Determine which side to align to based on the tag's orientation
          # Note: tag_angle will be near 0 when directly in front,
          # positive on one side, negative on the other
          if self.alignSide == "left":
            # Offset to the left relative to the tag's orientation
            target_x = offset_amount * math.cos(tag_angle + math.pi/2)
          else:  # "right"
            # Offset to the right relative to the tag's orientation
            target_x = offset_amount * math.cos(tag_angle - math.pi/2)
            
          # Set the PID controller setpoints
          self.xController.setSetpoint(target_x)
          self.yController.setSetpoint(0.02)  # Fixed distance from tag
          self.tController.setSetpoint(0)  # Want to be facing directly at the tag
          
          self.alignLocationXPub.set(target_x)
          self.setpointsInitialized = True
        
        # Update current pose information
        self.currentXPub.set(self.dx)
        self.currentYPub.set(self.dz)
        self.currentTPub.set(self.dt)
        
        # Calculate control outputs
        xSpeed = self.xController.calculate(self.dx)
        ySpeed = self.yController.calculate(self.dz)
        tSpeed = -self.tController.calculate(self.dt)
        
        self.speedYPub.set(ySpeed)
        
        # Convert to chassis speeds 
        speeds = ChassisSpeeds(ySpeed, -xSpeed, -tSpeed)
        
        # Publish debug info
        self.dXPub.set(abs(self.dx - self.xController.getSetpoint()))
        self.dYPub.set(abs(self.dz - self.yController.getSetpoint()))
        self.dTPub.set(abs(self.dt))
        
        # Drive the robot
        self.drivetrain.driveFromRelativeCoordinates(speeds, None)
        
      except Exception as e:
        print(f"Error in AutoAlign: {e}")
        self.cancel()
    else:
      print("No April tag detected")
    
  def end(self, interrupted: bool):
    if interrupted:
      print("Auto align interrupted")
    else:
      print("Auto align completed")
    self.drivetrain.stopMotors()
    
  def inTolerance(self):
    x_tolerance = 0.02
    y_tolerance = 0.02
    t_tolerance = 0.05
    
    # Check if we're within tolerance of the setpoints
    return (abs(self.dx - self.xController.getSetpoint()) < x_tolerance and 
            abs(self.dz - self.yController.getSetpoint()) < y_tolerance and 
            abs(self.dt) < t_tolerance)
    
  def isFinished(self) -> bool:
    if (not self.llSubsystem.limelightLeftDetectsTag()) and (not self.llSubsystem.limelightRightDetectsTag()):
      print("Auto align finished: No tag visible")
      return True
      
    if self.inTolerance():
      print("Auto align finished: At target position")
      return True
      
    return False