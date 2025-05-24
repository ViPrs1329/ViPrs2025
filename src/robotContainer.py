import commands2.button
from commands2 import SequentialCommandGroup
from commands2.button import CommandXboxController

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import NamedCommands
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.events import EventTrigger
from pathplannerlib.path import GoalEndState
from pathplannerlib.path import PathConstraints
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.path import Waypoint

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from wpilib import SendableChooser
from wpilib import SmartDashboard
from wpilib import DriverStation
from wpilib import XboxController

from commands2 import Command
from commands2 import PrintCommand
from commands2 import InstantCommand
from commands2.button import CommandXboxController
from commands2.button import Trigger

from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.ElevatorSubsystem import ElevatorSubsystem
from subsystems.IntakeSubsystem import IntakeSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem

from constants import Drive

class RobotContainer:
    """
    This class is where the bulk of the robot's resources are declared. Here, subsystems
    are instantiated and commands and button bindings are configured.
    """
    def __init__(self):
        self.initSubsystems()
        self.initControls()
        self.initCommands()
        self.configureButtonBindings()

        self.autoChooser: SendableChooser = AutoBuilder.buildAutoChooser("Autos")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        NamedCommands.registerCommand("marker1", PrintCommand("marker1"))
        NamedCommands.registerCommand("marker2", PrintCommand("marker2"))
        NamedCommands.registerCommand("Hello", PrintCommand("Hello"))
        
    def initSubsystems(self):
        """Instantiate the robot's subsystems."""
        
        # create subsystems
        self.drivetrain: DriveSubsystem = DriveSubsystem()
        self.elevator: ElevatorSubsystem = ElevatorSubsystem()
        self.intake: IntakeSubsystem = IntakeSubsystem()
        self.limelight: LimelightSubsystem = LimelightSubsystem()

        # register subsystems with the command scheduler
        commands2.CommandScheduler.getInstance().registerSubsystem(self.drivetrain)
        commands2.CommandScheduler.getInstance().registerSubsystem(self.elevator)
        commands2.CommandScheduler.getInstance().registerSubsystem(self.intake)
        commands2.CommandScheduler.getInstance().registerSubsystem(self.limelight)

    def initControls(self):
        """Instantiate the robot's control objects"""
        
        self.drivingController: CommandXboxController = CommandXboxController(0)


    def initCommands(self):
        """Instantiate the robot's commands."""
        
        pass

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
               
        self.drivetrain.setDefaultCommand(
            InstantCommand(
                lambda: self.drivetrain.controllerDrive(
                    -self.drivingController.getLeftY(),
                    -self.drivingController.getLeftX(),
                    -self.drivingController.getRightX()
                )
            )
        )

        self.drivingController.x().onTrue(
            InstantCommand(
                lambda: self.drivetrain.rezeroGyro(),
                self.drivetrain
            )
        )
    def getAutonomousCommand(self) -> Command:
        return self.autoChooser.getSelected()

    # not needed since commands2.Subsystem automatically 
    # calls the update function inside the subsystem 
    # each robot iteration

    # def updateHardware(self):
    #     """Call the update methods of each subsystem."""
    #     pass
