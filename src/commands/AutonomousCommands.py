# commands/AutonomousCommands.py
import commands2
import math
import wpilib
import ntcore
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
import wpimath.geometry as geometry
from subsystems.SwerveDriveSubsystem import DriveTrain

class DriveForwardCommand(commands2.CommandBase):
    """Command to drive the robot forward a specified distance."""
    
    def __init__(self, drivetrain: DriveTrain, distance_feet: float, speed: float = 0.5):
        """Initialize the DriveForwardCommand.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            distance_feet (float): The distance to drive in feet.
            speed (float, optional): The speed to drive at, from 0 to 1. Defaults to 0.5.
        """
        super().__init__()
        self.setName("DriveForwardCommand")
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
        # Convert feet to meters (1 foot = 0.3048 meters)
        self.distance_meters = distance_feet * 0.3048
        self.speed = speed
        
        # Create a PID controller for distance control
        self.distance_pid = PIDController(1.0, 0.0, 0.0)
        
        # Create a PID controller for rotation (to keep robot straight)
        self.rotation_pid = PIDController(0.5, 0.0, 0.0)
        self.rotation_pid.enableContinuousInput(-math.pi, math.pi)
        
        # Initial position and heading
        self.initial_pose = None
        self.initial_heading = None
        
        # Add a timer to prevent infinite driving
        self.timer = wpilib.Timer()
        
        # Maximum allowed time to complete the drive
        self.max_drive_time = 5.0  # 5 seconds
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        print(f"Starting to drive forward {self.distance_meters:.2f} meters")
        
        # Reset PID controllers
        self.distance_pid.reset()
        self.rotation_pid.reset()
        
        # Reset and start timer
        self.timer.reset()
        self.timer.start()
        
        # Record initial position and heading
        self.initial_pose = self.drivetrain.getPose()
        self.initial_heading = self.initial_pose.rotation().radians()
        
        # Set distance PID setpoint to current position + desired distance
        # We're moving in the robot's X direction
        self.target_distance = self.distance_meters
        self.distance_traveled = 0.0
        

    # Updates for the DriveForwardCommand.execute() method in AutonomousCommands.py

    def execute(self):
        """Called repeatedly during command execution."""
        # Get current pose
        current_pose = self.drivetrain.getPose()

        # Debug the current pose
        print(f"Current Pose - x: {current_pose.x:.2f}, y: {current_pose.y:.2f}, " + 
            f"rotation: {current_pose.rotation().degrees():.2f}°")
        
        # Calculate distance traveled
        dx = current_pose.x - self.initial_pose.x
        dy = current_pose.y - self.initial_pose.y
        self.distance_traveled = math.sqrt(dx**2 + dy**2)
        
        # Calculate remaining distance
        remaining_distance = self.target_distance - self.distance_traveled
        
        # Calculate forward speed using PID
        # We'll cap the output to our desired speed
        forward_speed = self.distance_pid.calculate(self.distance_traveled, self.target_distance)
        forward_speed = max(-self.speed, min(forward_speed, self.speed))
        
        # If we're close to the target, start slowing down
        if remaining_distance < 0.5:  # Begin slowing down at 0.5 meters
            forward_speed *= (remaining_distance / 0.5)
        
        # Calculate rotation correction to maintain heading
        current_heading = current_pose.rotation().radians()
        rotation_speed = self.rotation_pid.calculate(current_heading, self.initial_heading)
        
        # Create chassis speeds (robot-oriented)
        # X is forward, Y is left-right, omega is rotation
        speeds = ChassisSpeeds(forward_speed, 0.0, rotation_speed)
        
        # Print speeds for debugging
        print(f"Command speeds - vx: {forward_speed:.2f}, vy: 0.00, omega: {rotation_speed:.2f}")
        
        # Drive the robot using the calculated speeds
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Update progress in NetworkTables for dashboard
        try:
            progress = (self.distance_traveled / self.target_distance) * 100
            progress = min(progress, 100.0)  # Cap at 100%
            
            nt_inst = ntcore.NetworkTableInstance.getDefault()
            auto_table = nt_inst.getTable("Autonomous")
            progress_pub = auto_table.getDoubleTopic("progress").publish()
            progress_pub.set(progress)
            
            # Also publish position for redundancy
            x_pub = auto_table.getDoubleTopic("robot_x").publish()
            y_pub = auto_table.getDoubleTopic("robot_y").publish()
            x_pub.set(current_pose.x)
            y_pub.set(current_pose.y)
        except Exception as e:
            # Log but don't crash if there's a NetworkTables error
            print(f"Warning: Failed to update progress: {e}")
        
        # Debug output
        print(f"Distance: {self.distance_traveled:.2f}/{self.target_distance:.2f} m, " +
            f"Speed: {forward_speed:.2f}, Heading Correction: {rotation_speed:.2f}")
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the robot has traveled the desired distance or timed out.
        """
        # Command is finished when:
        # 1. We've traveled the requested distance
        # 2. We've exceeded the maximum allowed time
        return (self.distance_traveled >= self.target_distance) or \
               (self.timer.get() >= self.max_drive_time)
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        self.timer.stop()
        
        if interrupted:
            print(f"Drive forward interrupted at {self.distance_traveled:.2f}/{self.target_distance:.2f} meters")
        else:
            print(f"Drive forward completed, traveled {self.distance_traveled:.2f} meters")

class TurnToAngleCommand(commands2.CommandBase):
    """Command to turn the robot to a specified angle."""
    
    def __init__(self, drivetrain: DriveTrain, target_angle_degrees: float, speed: float = 0.3):
        """Initialize the TurnToAngleCommand.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            target_angle_degrees (float): The target angle in degrees (relative to current heading).
            speed (float, optional): The maximum rotation speed. Defaults to 0.3.
        """
        super().__init__()
        self.setName("TurnToAngleCommand")
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
        # Convert degrees to radians
        self.target_angle_rad = math.radians(target_angle_degrees)
        self.max_rotation_speed = speed
        
        # Create a PID controller for rotation
        self.rotation_pid = PIDController(1.0, 0.0, 0.1)
        self.rotation_pid.enableContinuousInput(-math.pi, math.pi)
        
        # Initial heading
        self.initial_heading = None
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Get initial heading
        self.initial_heading = self.drivetrain.getPose().rotation().radians()
        
        # Calculate target heading (relative to initial)
        self.target_heading = self.initial_heading + self.target_angle_rad
        
        # Normalize target heading to [-π, π]
        self.target_heading = math.atan2(math.sin(self.target_heading), 
                                         math.cos(self.target_heading))
        
        # Reset PID controller
        self.rotation_pid.reset()
        
        print(f"Turning from {math.degrees(self.initial_heading):.1f}° to {math.degrees(self.target_heading):.1f}°")
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Get current heading
        current_heading = self.drivetrain.getPose().rotation().radians()
        
        # Calculate rotation speed using PID
        rotation_speed = self.rotation_pid.calculate(current_heading, self.target_heading)
        
        # Limit rotation speed
        rotation_speed = max(-self.max_rotation_speed, min(rotation_speed, self.max_rotation_speed))
        
        # Create chassis speeds (zero translation, only rotation)
        speeds = ChassisSpeeds(0.0, 0.0, rotation_speed)
        
        # Drive the robot using the calculated speeds
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Debug output
        print(f"Current: {math.degrees(current_heading):.1f}°, Target: {math.degrees(self.target_heading):.1f}°, " +
              f"Speed: {rotation_speed:.2f}")
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the robot has turned to the desired angle.
        """
        # Get current heading
        current_heading = self.drivetrain.getPose().rotation().radians()
        
        # Calculate error
        error = abs(self.target_heading - current_heading)
        error = min(error, 2 * math.pi - error)  # Handle wrap-around
        
        # Command is finished when error is less than threshold
        return error < math.radians(2.0)  # 2 degree tolerance
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        
        if interrupted:
            current_heading = self.drivetrain.getPose().rotation().radians()
            print(f"Turn interrupted at {math.degrees(current_heading):.1f}°")
        else:
            print("Turn completed")


class DriveDistanceThenTurnCommand(commands2.SequentialCommandGroup):
    """Command to drive forward a distance and then turn to an angle."""
    
    def __init__(self, drivetrain: DriveTrain, distance_feet: float, angle_degrees: float):
        """Initialize the DriveDistanceThenTurnCommand.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            distance_feet (float): The distance to drive in feet.
            angle_degrees (float): The angle to turn in degrees.
        """
        super().__init__()
        self.setName("DriveDistanceThenTurnCommand")
        
        # Add the sequential commands
        self.addCommands(
            DriveForwardCommand(drivetrain, distance_feet),
            TurnToAngleCommand(drivetrain, angle_degrees)
        )


class ScorePreloadedCoralAutonomous(commands2.SequentialCommandGroup):
    """Autonomous routine to score a preloaded coral game piece and then drive away."""
    
    def __init__(self, drivetrain: DriveTrain, elevator, endEffector, score_level="medium"):
        """Initialize the ScorePreloadedCoralAutonomous command.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
            score_level (str, optional): Scoring level ("low", "medium", or "high"). Defaults to "medium".
        """
        super().__init__()
        self.setName("ScorePreloadedCoralAutonomous")
        
        # Import commands needed for scoring sequence
        from commands.ElevatorCommands import (
            ElevatorLowPositionCommand, 
            ElevatorMediumPositionCommand,
            ElevatorHighPositionCommand,
            ElevatorHomePositionCommand
        )
        from commands.IntakeCommands import EjectCoralCommand
        
        # Select the appropriate elevator position command based on score level
        if score_level.lower() == "low":
            elevator_command = ElevatorLowPositionCommand(elevator)
        elif score_level.lower() == "high":
            elevator_command = ElevatorHighPositionCommand(elevator)
        else:  # Default to medium
            elevator_command = ElevatorMediumPositionCommand(elevator)
        
        # Define the autonomous routine sequence
        self.addCommands(
            # Step 1: Move elevator to scoring position
            elevator_command,
            
            # Step 2: Pause briefly to ensure stability
            commands2.WaitCommand(0.5),
            
            # Step 3: Score coral (eject)
            EjectCoralCommand(endEffector, eject_speed=0.7, timeout=1.0),
            
            # Step 4: Return elevator to home position
            ElevatorHomePositionCommand(elevator),
            
            # Step 5: Drive away from the scoring area (backward)
            DriveForwardCommand(drivetrain, distance_feet=-5.0)  # Negative distance to drive backward
        )


class ComplexAutonomousRoutine(commands2.SequentialCommandGroup):
    """A complex autonomous routine that scores, drives to another position, and picks up another game piece."""
    
    def __init__(self, drivetrain: DriveTrain, elevator, endEffector):
        """Initialize the ComplexAutonomousRoutine command.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            elevator (Elevator): The elevator subsystem.
            endEffector (EndEffector): The end effector subsystem.
        """
        super().__init__()
        self.setName("ComplexAutonomousRoutine")
        
        # Import commands needed for the sequence
        from commands.ElevatorCommands import (
            ElevatorLowPositionCommand, 
            ElevatorMediumPositionCommand,
            ElevatorHighPositionCommand,
            ElevatorHomePositionCommand
        )
        from commands.IntakeCommands import IntakeCoralCommand, EjectCoralCommand
        from commands.AlgaeCommands import (
            AlgaeBottomPickupCommand,
            AlgaeIntakeCommand,
            AlgaeRetractedCommand
        )
        
        # Define the autonomous routine sequence
        self.addCommands(
            # Phase 1: Score preloaded coral
            # Move elevator to medium scoring position
            ElevatorMediumPositionCommand(elevator),
            
            # Score the coral
            EjectCoralCommand(endEffector, eject_speed=0.7, timeout=1.0),
            
            # Return elevator to home position
            ElevatorHomePositionCommand(elevator),
            
            # Phase 2: Move to a new position to pick up a game piece
            # Drive backward from scoring area
            DriveForwardCommand(drivetrain, distance_feet=-3.0, speed=0.6),
            
            # Turn toward the game piece
            TurnToAngleCommand(drivetrain, target_angle_degrees=90.0),
            
            # Drive to the game piece
            DriveForwardCommand(drivetrain, distance_feet=8.0, speed=0.6),
            
            # Phase 3: Pick up the game piece (using algae collector)
            # Move algae collector to bottom pickup position
            AlgaeBottomPickupCommand(endEffector),
            
            # Create a parallel race group that runs until a game piece is detected
            commands2.ParallelRaceGroup(
                # Run the intake until the command is canceled
                AlgaeIntakeCommand(endEffector),
                
                # Drive slowly forward while intaking (this will end when we hit the timeout)
                commands2.SequentialCommandGroup(
                    DriveForwardCommand(drivetrain, distance_feet=1.0, speed=0.2),
                    # Allow intake to continue for up to 3 seconds after driving
                    commands2.WaitCommand(3.0)
                )
            ),
            
            # Retract the algae collector
            AlgaeRetractedCommand(endEffector),
            
            # Phase 4: Return to scoring position
            # Turn back toward the goal
            TurnToAngleCommand(drivetrain, target_angle_degrees=-90.0),
            
            # Drive to the scoring area
            DriveForwardCommand(drivetrain, distance_feet=8.0, speed=0.6),
            
            # Turn to face the goal
            TurnToAngleCommand(drivetrain, target_angle_degrees=-90.0)
            
            # Note: From here, you could add another scoring sequence
            # if time allows during the autonomous period
        )


class DriveToPositionCommand(commands2.CommandBase):
    """Command to drive the robot to a specific position on the field."""
    
    def __init__(self, drivetrain: DriveTrain, target_x: float, target_y: float, speed: float = 0.5):
        """Initialize the DriveToPositionCommand.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            target_x (float): The target X position in meters.
            target_y (float): The target Y position in meters.
            speed (float, optional): The maximum speed to drive at. Defaults to 0.5.
        """
        super().__init__()
        self.setName("DriveToPositionCommand")
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
        self.target_x = target_x
        self.target_y = target_y
        self.max_speed = speed
        
        # PID controllers for X and Y control
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)
        
        # PID controller for rotation (to keep robot pointed in desired direction)
        self.rotation_controller = PIDController(0.5, 0.0, 0.1)
        self.rotation_controller.enableContinuousInput(-math.pi, math.pi)
        
        # Target heading - we'll calculate this in initialize
        self.target_heading = 0.0
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Get current position
        current_pose = self.drivetrain.getPose()
        
        # Calculate direction to target (for setting initial heading)
        dx = self.target_x - current_pose.x
        dy = self.target_y - current_pose.y
        
        # Calculate target heading (pointing toward the target)
        self.target_heading = math.atan2(dy, dx)
        
        # Reset PID controllers
        self.x_controller.reset()
        self.y_controller.reset()
        self.rotation_controller.reset()
        
        print(f"Driving to position ({self.target_x:.2f}, {self.target_y:.2f}) meters")
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Get current position
        current_pose = self.drivetrain.getPose()
        current_x = current_pose.x
        current_y = current_pose.y
        current_heading = current_pose.rotation().radians()
        
        # Calculate distance to target
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate PID outputs for X and Y
        x_speed = self.x_controller.calculate(current_x, self.target_x)
        y_speed = self.y_controller.calculate(current_y, self.target_y)
        
        # Normalize combined XY speed if it exceeds max_speed
        combined_speed = math.sqrt(x_speed*x_speed + y_speed*y_speed)
        if combined_speed > self.max_speed:
            scale_factor = self.max_speed / combined_speed
            x_speed *= scale_factor
            y_speed *= scale_factor
            
        # If we're close to the target, slow down proportionally
        if distance < 0.5:  # Within 0.5 meters
            slowdown_factor = distance / 0.5
            x_speed *= slowdown_factor
            y_speed *= slowdown_factor
        
        # Calculate rotation correction to maintain heading toward target
        rotation_speed = self.rotation_controller.calculate(current_heading, self.target_heading)
        
        # Create field-relative chassis speeds
        # Note: We use field-relative here since we're navigating to a field position
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed, y_speed, rotation_speed, current_pose.rotation()
        )
        
        # Drive the robot using the calculated speeds
        self.drivetrain.driveFromChassisSpeeds(speeds)
        
        # Debug output
        print(f"Current: ({current_x:.2f}, {current_y:.2f}) m, " +
              f"Target: ({self.target_x:.2f}, {self.target_y:.2f}) m, " +
              f"Distance: {distance:.2f} m")
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the robot is at the target position.
        """
        # Get current position
        current_pose = self.drivetrain.getPose()
        
        # Calculate distance to target
        dx = self.target_x - current_pose.x
        dy = self.target_y - current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Consider the command finished if we're close enough to the target
        return distance < 0.1  # Within 10 cm
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        
        if interrupted:
            print("Drive to position interrupted")
        else:
            print("Drive to position completed")


class FollowTrajectoryCommand(commands2.CommandBase):
    """Command to follow a pre-defined trajectory."""
    
    def __init__(self, drivetrain: DriveTrain, waypoints, max_velocity=2.0, max_acceleration=1.0):
        """Initialize the FollowTrajectoryCommand.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
            waypoints (list): List of Pose2d objects defining the path.
            max_velocity (float, optional): Maximum velocity in m/s. Defaults to 2.0.
            max_acceleration (float, optional): Maximum acceleration in m/s². Defaults to 1.0.
        """
        super().__init__()
        self.setName("FollowTrajectoryCommand")
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
        self.waypoints = waypoints
        
        # Create trajectory config
        self.config = TrajectoryConfig(max_velocity, max_acceleration)
        self.config.setKinematics(self.drivetrain.kinematics)
        
        # Generate trajectory from waypoints
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            waypoints,
            self.config
        )
        
        # Create PID controllers for following the trajectory
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)
        self.rotation_controller = ProfiledPIDController(
            1.0, 0.0, 0.0,
            TrapezoidProfile.Constraints(max_velocity, max_acceleration)
        )
        self.rotation_controller.enableContinuousInput(-math.pi, math.pi)
        
        # Timer for trajectory following
        self.timer = None
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Reset controllers
        self.x_controller.reset()
        self.y_controller.reset()
        self.rotation_controller.reset(
            self.drivetrain.getPose().rotation().radians(),
            0.0  # Initial angular velocity
        )
        
        # Reset and start timer
        self.timer = wpilib.Timer()
        self.timer.reset()
        self.timer.start()
        
        print(f"Following trajectory with {len(self.waypoints)} waypoints")
        
    def execute(self):
        """Called repeatedly during command execution."""
        # Get the current time
        current_time = self.timer.get()
        
        # Get trajectory state at the current time
        state = self.trajectory.sample(current_time)
        
        # Get the current robot pose
        current_pose = self.drivetrain.getPose()
        
        # Calculate the desired chassis speeds using the controllers
        target_x = state.pose.x
        target_y = state.pose.y
        target_heading = state.pose.rotation().radians()
        
        # Calculate chassis speeds using the controllers
        x_speed = self.x_controller.calculate(current_pose.x, target_x)
        y_speed = self.y_controller.calculate(current_pose.y, target_y)
        
        # Get the next rotation reference point
        heading_reference = self.rotation_controller.calculate(
            current_pose.rotation().radians(),
            target_heading
        )
        
        # Create field-relative chassis speeds
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed, y_speed, heading_reference, current_pose.rotation()
        )
        
        # Drive the robot using the calculated speeds
        self.drivetrain.driveFromChassisSpeeds(speeds)
        
        # Debug output
        print(f"Time: {current_time:.2f}s, " +
              f"Current: ({current_pose.x:.2f}, {current_pose.y:.2f}) m, " +
              f"Target: ({target_x:.2f}, {target_y:.2f}) m")
        
    def isFinished(self):
        """Return whether the command has finished.
        
        Returns:
            bool: True if the trajectory is complete.
        """
        # Check if trajectory is complete
        return self.timer.get() >= self.trajectory.totalTime()
        
    def end(self, interrupted):
        """Called when the command ends.
        
        Args:
            interrupted (bool): Whether the command was interrupted.
        """
        # Stop the timer
        self.timer.stop()
        
        # Stop the drivetrain
        self.drivetrain.stopMotors()
        
        if interrupted:
            print("Trajectory following interrupted")
        else:
            print("Trajectory following completed")




class LeaveStartingZoneAuto(commands2.SequentialCommandGroup):
    """Simple autonomous routine to leave the starting zone."""
    
    def __init__(self, drivetrain: DriveTrain):
        """Initialize the LeaveStartingZoneAuto command.
        
        Args:
            drivetrain (DriveTrain): The drivetrain subsystem.
        """
        super().__init__()
        self.setName("LeaveStartingZoneAuto")
        
        # Print for debugging
        print("Creating LeaveStartingZoneAuto with DriveForwardCommand")
        
        # Simply drive forward 5 feet to leave the starting zone
        self.addCommands(
            DriveForwardCommand(drivetrain, distance_feet=5.0, speed=0.5)
        )