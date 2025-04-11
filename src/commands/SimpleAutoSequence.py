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
from commands.Rotate import Rotate
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
        rotateCommand = Rotate(drivetrain, 180.0)
        
        # Create drive forward command (using existing DriveDistance command)
        # Convert feet to meters
        feet_to_meters = 0.3048
        distance_meters = 1 * feet_to_meters
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
            
            # Step 5: Raise elevator to L3 (now using the elevator subsystem method)
            commands2.InstantCommand(lambda: print("Raising elevator to L3")),
            commands2.InstantCommand(
                lambda: elevator.goToL3()
            ),
            
            # Allow time for elevator to reach position
            commands2.WaitCommand(1.5),
            
            # Step 6: Eject coral
            commands2.InstantCommand(lambda: print("Ejecting coral")),
            commands2.InstantCommand(
                # lambda: self.ejectCoral(endEffector)
                lambda: self.endEffector.ejectCoral(elevator.currentLevel)
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
    



