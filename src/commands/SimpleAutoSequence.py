import commands2
import math
import wpilib
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
from subsystems.LimelightSubsystem import LimelightSubsystem
from commands.DriveDistance import DriveDistance
from commands.autoAlign import AutoAlign
import constants

class SimpleAutoSequence(commands2.SequentialCommandGroup):
    """
    A simple autonomous routine that:
    1. Rotates the robot 180 degrees
    2. Drives forward 10 feet
    3. Aligns with a game element on the left
    4. Raises the elevator to L3
    5. Ejects coral
    
    This command doesn't use PathPlanner and relies only on built-in commands.
    """
    
    def __init__(self, drivetrain: DriveTrain, elevator: Elevator, endEffector: EndEffector, 
                 limelightSubsystem: LimelightSubsystem):
        
        # Create the rotation command (custom in-line command)
        rotateCommand = Rotate180Degrees(drivetrain)
        
        # Create drive forward command (using existing DriveDistance command)
        # Convert 10 feet to meters
        feet_to_meters = 0.3048
        distance_meters = 10 * feet_to_meters
        driveCommand = DriveDistance(drivetrain, distance_meters, 0.5)  # 0.5 speed
        
        # Use existing auto align command
        alignCommand = AutoAlign(limelightSubsystem, drivetrain, "left")
        
        # Construct the command sequence
        super().__init__(
            # Step 1: Reset gyro and drivetrain
            commands2.InstantCommand(lambda: drivetrain.resetHarder()),
            commands2.InstantCommand(lambda: print("Starting Simple Auto Sequence")),
            
            # Step 2: Rotate 180 degrees
            commands2.InstantCommand(lambda: print("Rotating 180 degrees")),
            rotateCommand,
            
            # Step 3: Drive forward 10 feet
            commands2.InstantCommand(lambda: print("Driving forward 10 feet")),
            driveCommand,
            
            # Step 4: Align with game element on the left
            commands2.InstantCommand(lambda: print("Starting auto-alignment left")),
            alignCommand,
            
            # Step 5: Raise elevator to L3
            commands2.InstantCommand(lambda: print("Raising elevator to L3")),
            commands2.InstantCommand(
                lambda: self.raiseElevatorToL3(elevator)
            ),
            
            # Allow time for elevator to reach position
            commands2.WaitCommand(1.5),
            
            # Step 6: Eject coral
            commands2.InstantCommand(lambda: print("Ejecting coral")),
            commands2.InstantCommand(
                lambda: self.ejectCoral(endEffector)
            ),
            
            # Run the motors for 1 second to ensure coral is ejected
            commands2.WaitCommand(1.0),
            
            # Stop the coral motors
            commands2.InstantCommand(
                lambda: endEffector.stopCoralMotors()
            ),
            
            # End autonomous
            commands2.InstantCommand(lambda: print("Simple Auto Sequence completed"))
        )
    
    def raiseElevatorToL3(self, elevator: Elevator):
        """Raises the elevator to Level 3"""
        level_index = 2  # L3 is index 2 (0-based indexing)
        target_height = constants.reefConsts.reefLevels[level_index][1] + constants.elevatorConsts.verticalOffset
        target_position = constants.convert.in2rot(target_height) / 2
        elevator.currentLevel = 3
        elevator.gotoPosition(target_position)
    
    def ejectCoral(self, endEffector: EndEffector):
        """Ejects coral at the appropriate speed"""
        endEffector.coral_intake_left_motor.set(constants.intakeConsts.intakeSpeed)
        endEffector.coral_intake_right_motor.set(constants.intakeConsts.intakeSpeed)


class Rotate180Degrees(commands2.Command):
    """
    Command to rotate the robot 180 degrees in place.
    Uses the gyro to track progress and stops when the rotation is complete.
    """
    
    def __init__(self, drivetrain: DriveTrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.timer = wpilib.Timer()
        
        # PID constants for rotation control
        self.kP = 0.02  # Proportional gain
        self.target_angle = 180.0  # Target angle in degrees
        self.tolerance = 5.0  # Tolerance in degrees
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Record the starting angle
        self.starting_angle = self.drivetrain.gyro.get_yaw().value_as_double
        self.target_angle = self.starting_angle + 180.0
        
        # Normalize the target angle to be between -180 and 180
        if self.target_angle > 180.0:
            self.target_angle -= 360.0
            
        print(f"Starting rotation: current={self.starting_angle:.1f}°, target={self.target_angle:.1f}°")
        
        # Reset the timer
        self.timer.reset()
        self.timer.start()
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        
        # Calculate the error (how far we are from the target angle)
        error = self.target_angle - current_angle
        
        # Normalize the error to be between -180 and 180
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
            
        # Calculate the rotation speed (proportional control)
        rotation_speed = self.kP * error
        
        # Clamp the rotation speed between -0.6 and 0.6
        rotation_speed = max(-0.6, min(0.6, rotation_speed))
        
        # Set a minimum rotation speed to overcome friction
        if abs(rotation_speed) < 0.15 and abs(error) > self.tolerance:
            rotation_speed = 0.15 if error > 0 else -0.15
            
        # Create a ChassisSpeeds object with only rotation
        speeds = ChassisSpeeds(0, 0, rotation_speed)
        
        # Drive the robot
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Debug output
        if self.timer.advanceIfElapsed(0.2):  # Print every 0.2 seconds
            print(f"Rotation: current={current_angle:.1f}°, target={self.target_angle:.1f}°, " +
                  f"error={error:.1f}°, speed={rotation_speed:.2f}")
    
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Stop the robot
        self.drivetrain.stopMotors()
        
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        error = abs(self.target_angle - current_angle)
        while error > 180.0:
            error = 360.0 - error
            
        if interrupted:
            print(f"Rotation interrupted! Final angle: {current_angle:.1f}°, error: {error:.1f}°")
        else:
            print(f"Rotation completed! Final angle: {current_angle:.1f}°, error: {error:.1f}°")
        
        self.timer.stop()
    
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        
        # Calculate how far we are from the target
        error = abs(self.target_angle - current_angle)
        
        # Normalize the error
        while error > 180.0:
            error = 360.0 - error
            
        # Check if we're within tolerance
        is_on_target = error <= self.tolerance
        
        # Also set a maximum time for the rotation (safety)
        timeout = self.timer.get() > 3.0  # 3 seconds max
        
        if timeout and not is_on_target:
            print("Rotation timed out!")
            
        return is_on_target or timeout