"""
This class is where the bulk of the robot should be declared.
"""
import wpilib
import commands2
import commands2.button
from typing import Optional

from constants import *
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.coralsubsystem import CoralSubsystem
from commands.drivecommands import DefaultDriveCommand
from commands.elevatorcommands import SetElevatorHeight, ManualElevatorControl
from commands.coralcommands import IntakeCoral, EjectCoral

class RobotContainer:
    """
    This class hosts the bulk of the robot's functions. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the Robot
    periodic methods (other than the scheduler calls). Instead, the structure of the robot
    (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        """
        The container for the robot. Contains subsystems, OI devices, and commands.
        """
        # Create controllers
        self.driver_controller = wpilib.XboxController(DRIVER_CONTROLLER_PORT)
        self.operator_controller = wpilib.XboxController(OPERATOR_CONTROLLER_PORT)

        # Create subsystems
        self.drive = DriveSubsystem()
        self.elevator = ElevatorSubsystem()
        self.coral = CoralSubsystem()
        # TODO: Initialize other subsystems
        # self.end_effector = EndEffectorSubsystem()

        # Configure button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.drive.setDefaultCommand(DefaultDriveCommand(self.drive))

    def configureButtonBindings(self) -> None:
        """
        Use this method to define button->command mappings.
        """
        # Field-relative drive toggle (Y button)
        commands2.button.JoystickButton(
            self.driver_controller, wpilib.XboxController.Button.kY
        ).onTrue(commands2.RunCommand(
            lambda: self.drive.toggleFieldRelative(),
            [self.drive]
        ))

        # Reset gyro (X button)
        commands2.button.JoystickButton(
            self.driver_controller, wpilib.XboxController.Button.kX
        ).onTrue(commands2.RunCommand(
            lambda: self.drive.resetGyro(),
            [self.drive]
        ))

        # Get the operator controller
        operator = wpilib.XboxController(OPERATOR_CONTROLLER_PORT)
        
        # Elevator height controls
        operator.a().onTrue(SetElevatorHeight(self.elevator, "BASE"))
        operator.b().onTrue(SetElevatorHeight(self.elevator, "L1"))
        operator.x().onTrue(SetElevatorHeight(self.elevator, "L2"))
        operator.y().onTrue(SetElevatorHeight(self.elevator, "L3"))
        
        # Manual elevator control
        operator.leftBumper().whileTrue(ManualElevatorControl(self.elevator, 0.5))
        operator.rightBumper().whileTrue(ManualElevatorControl(self.elevator, -0.5))
        
        # CORAL controls
        operator.leftTrigger().whileTrue(IntakeCoral(self.coral))
        operator.rightTrigger().whileTrue(EjectCoral(self.coral))

        # TODO: Add other button bindings for:
        # - End effector control
        # - Autonomous routines

    def getAutonomousCommand(self) -> Optional[commands2.Command]:
        """
        Use this to pass the autonomous command to the main Robot class.
        
        Returns
        -------
        Optional[commands2.Command]
            the command to run in autonomous
        """
        # TODO: Implement autonomous command
        return None 