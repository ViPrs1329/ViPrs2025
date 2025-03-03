# commands/AutonomousCommands.py
import commands2
import math
import wpilib
import ntcore

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController

from subsystems.SwerveDriveSubsystem import SwerveDrive
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffectorSubsystem import EndEffector
from constants import driveConsts, autoConsts


class DriveForwardCommand(commands2.Command):
    """
    Command to drive the robot forward a specified distance.
    """
    
    def __init__(self, drivetrain: SwerveDrive, distance_meters: float, speed: float = 1.0,
                 timeout: float = None):
        """
        Initialize the DriveForwardCommand.
        
        Args:
            drivetrain (SwerveDrive): The drivetrain subsystem
            distance_meters (float): Distance to drive in meters
            speed (float, optional): The speed to drive at (0-1). Defaults to 1.0.
            timeout (float, optional): Command timeout in seconds. Defaults to None.
        """
        super().__init__()
        self.setName("DriveForward")
        
        # Store parameters
        self.drivetrain = drivetrain
        self.distance = distance_meters
        self.speed = speed
        self.timeout = timeout if timeout is not None else autoConsts.DEFAULT_TIMEOUT
        
        # Require the drivetrain subsystem
        self.addRequirements(drivetrain)
        
        # Create PID controller for distance
        self.distance_controller = PIDController(
            autoConsts.X_CONTROLLER_P, 0.0, 0.0
        )
        
        # Create PID controller for heading
        self.heading_controller = PIDController(
            autoConsts.ROTATION_CONTROLLER_P, 0.0, 0.0
        )
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        
        # Initialize variables
        self.initial_pose = None
        self.target_pose = None
        self.timer = wpilib.Timer()
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Get the current robot pose
        self.initial_pose = self.drivetrain.getPose()
        
        # Calculate the target pose (moving forward in the robot's current direction)
        current_rotation = self.initial_pose.rotation()
        target_x = self.initial_pose.x + self.distance * math.cos(current_rotation.radians())
        target_y = self.initial_pose.y + self.distance * math.sin(current_rotation.radians())
        self.target_pose = Pose2d(target_x, target_y, current_rotation)
        
        # Reset and start the timer
        self.timer.reset()
        self.timer.start()
        
        # Log information
        print(f"DriveForward: {self.distance:.2f}m at {self.speed:.2f} speed")
        print(f"  Initial: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f}, {current_rotation.degrees():.2f}°)")
        print(f"  Target: ({target_x:.2f}, {target_y:.2f}, {current_rotation.degrees():.2f}°)")
        
        # Update dashboard
        wpilib.SmartDashboard.putNumber("Autonomous/TargetX", target_x)
        wpilib.SmartDashboard.putNumber("Autonomous/TargetY", target_y)
        wpilib.SmartDashboard.putNumber("Autonomous/TargetHeading", current_rotation.degrees())
    
    def execute(self):
        """Called repeatedly during command execution."""
        # Get current pose
        current_pose = self.drivetrain.getPose()
        
        # Calculate distance to target
        dx = self.target_pose.x - current_pose.x
        dy = self.target_pose.y - current_pose.y
        distance_remaining = math.sqrt(dx*dx + dy*dy)
        
        # Calculate heading to target
        target_heading = math.atan2(dy, dx)
        
        # Get current heading
        current_heading = current_pose.rotation().radians()
        
        # Calculate heading error
        heading_error = target_heading - current_heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Calculate speed based on distance remaining
        forward_speed = self.distance_controller.calculate(0, distance_remaining)
        forward_speed = min(forward_speed, self.speed)
        
        # Calculate rotation based on heading error
        rotation_speed = self.heading_controller.calculate(0, heading_error)
        
        # Create chassis speeds (field-relative)
        field_x_speed = forward_speed * math.cos(target_heading)
        field_y_speed = forward_speed * math.sin(target_heading)
        
        # Drive the robot
        self.drivetrain.drive(field_x_speed, field_y_speed, rotation_speed, True)
        
        # Update dashboard
        wpilib.SmartDashboard.putNumber("Autonomous/DistanceRemaining", distance_remaining)
        wpilib.SmartDashboard.putNumber("Autonomous/HeadingError", math.degrees(heading_error))
        
        # Update NetworkTables for simulation dashboard
        try:
            nt_inst = ntcore.NetworkTableInstance.getDefault()
            auto_table = nt_inst.getTable("Autonomous")
            
            # Calculate progress as percentage
            initial_distance = self.distance
            distance_traveled = initial_distance - distance_remaining
            progress = (distance_traveled / initial_distance) * 100 if initial_distance > 0 else 100
            progress = max(0, min(progress, 100))  # Clamp to 0-100
            
            # Publish progress
            progress_pub = auto_table.getDoubleTopic("progress").publish()
            progress_pub.set(progress)
            
            # Publish current command
            cmd_pub = auto_table.getStringTopic("current_command").publish()
            cmd_pub.set(self.getName())
        except Exception as e:
            print(f"Error updating NetworkTables: {e}")
    
    def end(self, interrupted):
        """
        Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted
        """
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        
        # Stop the timer
        self.timer.stop()
        
        # Log information
        if interrupted:
            print(f"DriveForward interrupted after {self.timer.get():.2f} seconds")
        else:
            print(f"DriveForward completed in {self.timer.get():.2f} seconds")
        
        # Update dashboard
        wpilib.SmartDashboard.putString("Autonomous/Status", 
                                      "Interrupted" if interrupted else "Completed")
    
    def isFinished(self):
        """
        Returns whether the command is finished.
        
        Returns:
            bool: True if the command is finished
        """
        # Get current pose
        current_pose = self.drivetrain.getPose()
        
        # Calculate distance to target
        dx = self.target_pose.x - current_pose.x
        dy = self.target_pose.y - current_pose.y
        distance_remaining = math.sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the target
        at_target = distance_remaining < 0.05  # 5cm tolerance
        
        # Check if we've timed out
        timed_out = self.timer.get() > self.timeout
        
        return at_target or timed_out


class RotateToAngleCommand(commands2.Command):
    """
    Command to rotate the robot to a specified angle.
    """
    
    def __init__(self, drivetrain: SwerveDrive, target_angle_degrees: float, speed: float = 0.5,
                 timeout: float = None):
        """
        Initialize the RotateToAngleCommand.
        
        Args:
            drivetrain (SwerveDrive): The drivetrain subsystem
            target_angle_degrees (float): Target angle in degrees (relative to field)
            speed (float, optional): Maximum rotation speed (0-1). Defaults to 0.5.
            timeout (float, optional): Command timeout in seconds. Defaults to None.
        """
        super().__init__()
        self.setName("RotateToAngle")
        
        # Store parameters
        self.drivetrain = drivetrain
        self.target_angle = math.radians(target_angle_degrees)
        self.speed = speed
        self.timeout = timeout if timeout is not None else autoConsts.DEFAULT_TIMEOUT
        
        # Require the drivetrain subsystem
        self.addRequirements(drivetrain)
        
        # Create PID controller for rotation
        self.rotation_controller = PIDController(
            autoConsts.ROTATION_CONTROLLER_P, 0.0, 0.0
        )
        self.rotation_controller.enableContinuousInput(-math.pi, math.pi)
        
        # Initialize timer
        self.timer = wpilib.Timer()
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Reset and start the timer
        self.timer.reset()
        self.timer.start()
        
        # Get current heading
        current_heading = self.drivetrain.getPose().rotation().radians()
        
        # Log information
        print(f"RotateToAngle: {math.degrees(self.target_angle):.2f}° at {self.speed:.2f} speed")
        print(f"  Current heading: {math.degrees(current_heading):.2f}°")
        
        # Update dashboard
        wpilib.SmartDashboard.putNumber("Autonomous/TargetAngle", math.degrees(self.target_angle))
    
    def execute(self):
        """Called repeatedly during command execution."""
        # Get current heading
        current_heading = self.drivetrain.getPose().rotation().radians()
        
        # Calculate rotation speed using PID
        rotation_speed = self.rotation_controller.calculate(current_heading, self.target_angle)
        
        # Limit rotation speed
        max_speed = self.speed * driveConsts.MAX_ANGULAR_ACCELERATION
        rotation_speed = max(-max_speed, min(rotation_speed, max_speed))
        
        # Create zero translation, rotation-only chassis speeds
        self.drivetrain.drive(0, 0, rotation_speed, False)
        
        # Calculate heading error for display
        heading_error = self.target_angle - current_heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Update dashboard
        wpilib.SmartDashboard.putNumber("Autonomous/HeadingError", math.degrees(heading_error))
        
        # Update NetworkTables for simulation dashboard
        try:
            nt_inst = ntcore.NetworkTableInstance.getDefault()
            auto_table = nt_inst.getTable("Autonomous")
            
            # Calculate progress as percentage of angle
            initial_error = math.pi  # Worst case error
            current_error = abs(heading_error)
            progress = ((initial_error - current_error) / initial_error) * 100
            progress = max(0, min(progress, 100))  # Clamp to 0-100
            
            # Publish progress
            progress_pub = auto_table.getDoubleTopic("progress").publish()
            progress_pub.set(progress)
            
            # Publish current command
            cmd_pub = auto_table.getStringTopic("current_command").publish()
            cmd_pub.set(self.getName())
        except Exception as e:
            print(f"Error updating NetworkTables: {e}")
    
    def end(self, interrupted):
        """
        Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted
        """
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        
        # Stop the timer
        self.timer.stop()
        
        # Log information
        if interrupted:
            print(f"RotateToAngle interrupted after {self.timer.get():.2f} seconds")
        else:
            print(f"RotateToAngle completed in {self.timer.get():.2f} seconds")
        
        # Update dashboard
        wpilib.SmartDashboard.putString("Autonomous/Status", 
                                      "Interrupted" if interrupted else "Completed")
    
    def isFinished(self):
        """
        Returns whether the command is finished.
        
        Returns:
            bool: True if the command is finished
        """
        # Get current heading
        current_heading = self.drivetrain.getPose().rotation().radians()
        
        # Calculate heading error
        heading_error = self.target_angle - current_heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Check if we've reached the target
        at_target = abs(heading_error) < math.radians(2)  # 2 degree tolerance
        
        # Check if we've timed out
        timed_out = self.timer.get() > self.timeout
        
        return at_target or timed_out


class LeaveStartingZoneAuto(commands2.SequentialCommandGroup):
    """
    Simple autonomous routine to leave the starting zone.
    """
    
    def __init__(self, drivetrain: SwerveDrive):
        """
        Initialize the LeaveStartingZoneAuto command.
        
        Args:
            drivetrain (SwerveDrive): The drivetrain subsystem
        """
        super().__init__()
        self.setName("LeaveStartingZoneAuto")
        
        # Add commands to the sequential group
        self.addCommands(
            # Drive forward to leave the starting zone
            DriveForwardCommand(
                drivetrain,
                autoConsts.LEAVE_COMMUNITY_DISTANCE,
                autoConsts.DEFAULT_AUTO_SPEED
            )
        )


class ScoreAndLeaveAuto(commands2.SequentialCommandGroup):
    """
    Autonomous routine to score a pre-loaded game piece and leave the starting zone.
    """
    
    def __init__(self, drivetrain: SwerveDrive, elevator: Elevator, end_effector: EndEffector):
        """
        Initialize the ScoreAndLeaveAuto command.
        
        Args:
            drivetrain (SwerveDrive): The drivetrain subsystem
            elevator (Elevator): The elevator subsystem
            end_effector (EndEffector): The end effector subsystem
        """
        super().__init__()
        self.setName("ScoreAndLeaveAuto")
        
        # Add commands to the sequential group
        self.addCommands(
            # Move elevator to medium position
            commands2.RunCommand(
                lambda: elevator.moveToPosition(elevator.elevatorConsts.MEDIUM_POSITION),
                elevator
            ).withTimeout(2.0),
            
            # Wait for stability
            commands2.WaitCommand(0.5),
            
            # Eject coral
            commands2.RunCommand(
                lambda: end_effector.ejectCoral(),
                end_effector
            ).withTimeout(1.0),
            
            # Return elevator to home position
            commands2.RunCommand(
                lambda: elevator.moveToPosition(elevator.elevatorConsts.HOME_POSITION),
                elevator
            ).withTimeout(2.0),
            
            # Drive backwards (away from scoring location)
            DriveForwardCommand(
                drivetrain,
                -1.0,  # 1 meter backwards
                autoConsts.DEFAULT_AUTO_SPEED
            ),
            
            # Turn 180 degrees
            RotateToAngleCommand(
                drivetrain,
                180.0
            ),
            
            # Drive forward to leave the starting zone
            DriveForwardCommand(
                drivetrain,
                autoConsts.LEAVE_COMMUNITY_DISTANCE,
                autoConsts.DEFAULT_AUTO_SPEED
            )
        )