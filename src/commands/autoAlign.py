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
    
    # Store alignment side
    if alignLocation not in ["left", "right"]:
      raise ValueError(f"robot can't align to {alignLocation}. must be 'left' or 'right'")
    self.alignSide = alignLocation
    
    # Status flag to track success/failure
    self.alignmentSuccessful = False
    
    # Create timer for timeout
    self.timer = wpilib.Timer()
    self.timeout = 4.0  # 4 second timeout
    
    # Network table entries for debugging
    self.alignLocationXPub = self.table.getDoubleTopic("Align Location X").publish()
    self.alignLocationYPub = self.table.getDoubleTopic("Align Location Y").publish()
    self.alignLocationTPub = self.table.getDoubleTopic("Align Location T").publish()
    self.currentXPub = self.table.getDoubleTopic("Current X").publish()
    self.currentYPub = self.table.getDoubleTopic("Current Y").publish()
    self.currentTPub = self.table.getDoubleTopic("Current T").publish()
    self.speedXPub = self.table.getDoubleTopic("Speed X").publish()
    self.speedYPub = self.table.getDoubleTopic("Speed Y").publish()
    self.speedTPub = self.table.getDoubleTopic("Speed T").publish()
    self.dXPub = self.table.getDoubleTopic("dx").publish()
    self.dYPub = self.table.getDoubleTopic("dy").publish()
    self.dTPub = self.table.getDoubleTopic("dt").publish()
    self.tagAnglePub = self.table.getDoubleTopic("Tag Angle").publish()
    self.offsetXPub = self.table.getDoubleTopic("Offset X").publish()
    self.timerPub = self.table.getDoubleTopic("Timer").publish()
    
    # Initialize previous speeds for smoothing
    self.prev_xSpeed = 0
    self.prev_ySpeed = 0
    self.prev_tSpeed = 0
    
  def initialize(self):
    # Start the timeout timer
    self.timer.reset()
    self.timer.start()
    
    # Reset success flag
    self.alignmentSuccessful = False
    
    # PID values - increased for more aggressive movement
    xkp = 0.3    # Increased from 0.15
    xki = 0.01   # Increased from 0.005
    xkd = 0.12
    self.xController = PIDController(xkp, xki, xkd)
    
    ykp = 0.3    # Increased from 0.15
    yki = 0.01   # Increased from 0.005
    ykd = 0.3
    self.yController = PIDController(ykp, yki, ykd)
    
    tkp = 0.4    # Increased from 0.3
    tki = 0.0
    tkd = 0.18
    self.tController = PIDController(tkp, tki, tkd)
    
    # Wait to set setpoints until we get tag data
    self.setpointsInitialized = False
    
    # Initialize tracking variables
    self.dx = self.dy = self.dz = self.dt = 1000
    print(f"AutoAlign initialized for {self.alignSide} position with {self.timeout}s timeout")
  
  def execute(self):
    # Update timer debug
    self.timerPub.set(self.timer.get())
    
    if self.llSubsystem.limelightLeftDetectsTag() or self.llSubsystem.limelightRightDetectsTag():
      try:
        targetPose = self.llSubsystem.getTargetPose()
        self.dx = targetPose.X()
        self.dy = targetPose.Y()
        self.dz = targetPose.Z()
        
        # Get target rotation
        self.dt = targetPose.rotation().Y()
        
        # Calculate tag angle in the horizontal plane
        tag_angle = math.atan2(self.dx, self.dz)
        self.tagAnglePub.set(math.degrees(tag_angle))
        
        # Initialize setpoints on first detection
        if not self.setpointsInitialized:
          offset_amount = constants.visionConsts.alignOffset
          
          # Set the target position based on the tag's orientation and the desired side
          if self.alignSide == "left":
            offset_angle = tag_angle + (math.pi/2)
          else:  # "right"
            offset_angle = tag_angle - (math.pi/2)
          
          # Calculate the X and Z offsets in the tag's coordinate system
          offset_x = offset_amount * math.sin(offset_angle)
          offset_z = offset_amount * math.cos(offset_angle)
          
          # Final target position is tag position plus the calculated offset
          target_x = self.dx + offset_x
          
          # Make sure we get close to the tag - get closer than current position
          target_z = self.dz - 0.1  # 10cm closer to the tag than current position
          
          # Set PID controller setpoints
          self.xController.setSetpoint(target_x)
          self.yController.setSetpoint(target_z)
          self.tController.setSetpoint(0)  # We always want to face the tag
          
          # Update debug values
          self.alignLocationXPub.set(target_x)
          self.alignLocationYPub.set(target_z)
          self.alignLocationTPub.set(0)
          self.offsetXPub.set(offset_x)
          
          print(f"Aligning to {self.alignSide} side, target pos: X={target_x:.3f}, Z={target_z:.3f}")
          self.setpointsInitialized = True
        
        # Update current pose information for debugging
        self.currentXPub.set(self.dx)
        self.currentYPub.set(self.dz)
        self.currentTPub.set(self.dt)
        
        # Calculate errors
        x_error = self.dx - self.xController.getSetpoint()
        y_error = self.dz - self.yController.getSetpoint()
        t_error = self.dt
        
        # Apply deadbands - slightly reduced for more aggressive movement
        x_deadband = 0.015  # Reduced from 0.02
        y_deadband = 0.015  # Reduced from 0.02
        t_deadband = 0.04   # Reduced from 0.045
        
        # Calculate PID outputs with deadbands
        xSpeed = 0 if abs(x_error) < x_deadband else self.xController.calculate(self.dx)
        ySpeed = 0 if abs(y_error) < y_deadband else self.yController.calculate(self.dz)
        tSpeed = 0 if abs(t_error) < t_deadband else -self.tController.calculate(self.dt)
        
        # Apply minimum speeds to ensure movement happens
        min_speed = 0.05  # Minimum speed threshold
        
        if xSpeed > 0 and xSpeed < min_speed:
            xSpeed = min_speed
        elif xSpeed < 0 and xSpeed > -min_speed:
            xSpeed = -min_speed
            
        if ySpeed > 0 and ySpeed < min_speed:
            ySpeed = min_speed
        elif ySpeed < 0 and ySpeed > -min_speed:
            ySpeed = -min_speed
        
        # Apply less smoothing for faster response
        max_accel = 0.03  # Increased from 0.02
        
        xSpeed = self.limitAcceleration(xSpeed, self.prev_xSpeed, max_accel)
        ySpeed = self.limitAcceleration(ySpeed, self.prev_ySpeed, max_accel)
        tSpeed = self.limitAcceleration(tSpeed, self.prev_tSpeed, max_accel)
        
        # Save current speeds for next cycle
        self.prev_xSpeed = xSpeed
        self.prev_ySpeed = ySpeed
        self.prev_tSpeed = tSpeed
        
        # Apply speed cap to prevent excessive speed
        max_speed = 0.6  # Maximum speed cap
        xSpeed = max(min(xSpeed, max_speed), -max_speed)
        ySpeed = max(min(ySpeed, max_speed), -max_speed)
        tSpeed = max(min(tSpeed, max_speed), -max_speed)
        
        # Update debug values
        self.speedXPub.set(xSpeed)
        self.speedYPub.set(ySpeed)
        self.speedTPub.set(tSpeed)
        self.dXPub.set(abs(x_error))
        self.dYPub.set(abs(y_error))
        self.dTPub.set(abs(t_error))
        
        # Convert to chassis speeds and drive
        speeds = ChassisSpeeds(ySpeed, -xSpeed, -tSpeed)
        self.drivetrain.driveFromRelativeCoordinates(speeds, None)
        
      except Exception as e:
        print(f"Error in AutoAlign: {e}")
        self.cancel()
    else:
      print("No April tag detected")
  
  def limitAcceleration(self, target_speed, previous_speed, max_accel):
    """Limits the rate of change of speed to prevent jerky movements"""
    if target_speed > previous_speed + max_accel:
      return previous_speed + max_accel
    elif target_speed < previous_speed - max_accel:
      return previous_speed - max_accel
    else:
      return target_speed
    
  def end(self, interrupted: bool):
    # Stop the timer
    self.timer.stop()
    
    # Stop the robot when command ends
    self.drivetrain.stopMotors()
    
    if interrupted:
      print("Auto align was interrupted")
      self.alignmentSuccessful = False
    elif self.timer.hasElapsed(self.timeout):
      print(f"Auto align timed out after {self.timeout} seconds")
      self.alignmentSuccessful = False
    else:
      print("Auto align completed successfully")
      self.alignmentSuccessful = self.inTolerance()
    
  def inTolerance(self):
    # Only check if we're within tolerance of setpoints if they've been initialized
    if not self.setpointsInitialized:
      return False
      
    # Check if we're within tolerance of all setpoints
    x_tolerance = 0.025
    y_tolerance = 0.025
    t_tolerance = 0.06
    
    return (abs(self.dx - self.xController.getSetpoint()) < x_tolerance and 
            abs(self.dz - self.yController.getSetpoint()) < y_tolerance and 
            abs(self.dt) < t_tolerance)
    
  def isFinished(self) -> bool:
    # Check for timeout
    if self.timer.hasElapsed(self.timeout):
      print(f"Auto align timed out after {self.timeout} seconds")
      return True
      
    # Check if tag is visible
    if (not self.llSubsystem.limelightLeftDetectsTag()) and (not self.llSubsystem.limelightRightDetectsTag()):
      print("Auto align finished: No tag visible")
      return True
      
    # Check if we've reached the target position
    if self.inTolerance():
      print("Auto align finished: At target position")
      self.alignmentSuccessful = True
      return True
      
    return False
    
  def wasSuccessful(self):
    """Returns whether the alignment was successful or not"""
    return self.alignmentSuccessful