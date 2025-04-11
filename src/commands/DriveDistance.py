import commands2
import wpilib
import math
from subsystems.SwerveDriveSubsystem import DriveTrain
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d

class DriveDistance(commands2.Command):
    """Command to drive the robot forward a specified distance."""
    
    def __init__(self, driveTrain: DriveTrain, distance_meters: float, speed: float = 0.5):
        """
        Creates a new DriveDistance command.
        
        Args:
            driveTrain: The drive subsystem to use
            distance_meters: The distance to drive in meters
            speed: The speed to drive at (from 0 to 1)
        """
        super().__init__()
        self.driveTrain = driveTrain
        self.distance_meters = distance_meters
        self.speed = speed
        self.addRequirements(driveTrain)
        
        # Will be set in initialize
        self.start_pose = None
        self.end_pose = None
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # First, align all swerve modules to face forward
        self._align_modules_forward()
        
        # Get the starting position
        self.start_pose = self.driveTrain.getPose()
        
        # Calculate the target position (moving in the robot's forward direction)
        current_x = self.start_pose.X()
        current_y = self.start_pose.Y()
        
        # Get the robot's current orientation and compute forward direction
        yaw_rad = self.driveTrain.gyro.get_yaw().value_as_double * math.pi / 180.0
        
        # Calculate the target position (X and Y offsets)
        dx = self.distance_meters * math.cos(yaw_rad)
        dy = self.distance_meters * math.sin(yaw_rad)
        
        # Set the target end position
        self.end_pose = Pose2d(current_x + dx, current_y + dy, self.start_pose.rotation())
        
        print(f"Starting drive from ({self.start_pose.X():.2f}, {self.start_pose.Y():.2f}) to ({self.end_pose.X():.2f}, {self.end_pose.Y():.2f})")
        print(f"Distance to drive: {self.distance_meters:.2f} meters")
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Drive forward at the specified speed
        self.driveTrain.manualDriveFromChassisSpeeds(ChassisSpeeds(-self.speed, 0, 0))
        
        # Debug output
        current_pose = self.driveTrain.getPose()
        distance_traveled = self._calculate_distance_traveled(current_pose)
        distance_remaining = self.distance_meters - distance_traveled
        
        if distance_traveled % 0.2 < 0.01:  # Print approximately every 20cm
            print(f"Distance traveled: {distance_traveled:.2f}m, Remaining: {distance_remaining:.2f}m")
    
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Stop the robot
        self.driveTrain.stopMotors()
        
        if interrupted:
            print("Drive distance command was interrupted!")
        else:
            print("Drive distance command completed successfully!")
            
        # Final distance report
        current_pose = self.driveTrain.getPose()
        distance_traveled = self._calculate_distance_traveled(current_pose)
        print(f"Final distance traveled: {distance_traveled:.2f} meters")
    
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        current_pose = self.driveTrain.getPose()
        distance_traveled = self._calculate_distance_traveled(current_pose)
        
        # End the command when we've traveled the desired distance
        return distance_traveled >= self.distance_meters
    
    def _calculate_distance_traveled(self, current_pose):
        """
        Calculate the distance traveled from the start position.
        
        Args:
            current_pose: The current robot pose
            
        Returns:
            The straight-line distance traveled from the start position
        """
        dx = current_pose.X() - self.start_pose.X()
        dy = current_pose.Y() - self.start_pose.Y()
        return math.sqrt(dx*dx + dy*dy)
        
    def _align_modules_forward(self):
        """
        Aligns all swerve modules to face forward before driving.
        This ensures the robot moves in a straight line.
        """
        print("Aligning swerve modules to face forward...")
        
        # Set all modules to zero rotation angle but zero speed
        # This uses the chassis speeds with zero velocity but ensures 
        # the module states get set to the forward position
        self.driveTrain.manualDriveFromChassisSpeeds(ChassisSpeeds(0.0001, 0, 0))
        
        # Small delay to allow modules to align
        timer = wpilib.Timer()
        timer.start()
        
        # Wait for alignment (500ms should be enough for the modules to rotate)
        while timer.get() < 0.5:
            # We could potentially check encoder values here to confirm alignment
            wpilib.SmartDashboard.putNumber("Alignment Timer", timer.get())
            
        print("Swerve modules aligned forward, ready to drive")