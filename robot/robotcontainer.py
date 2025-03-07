"""
This class is where the bulk of the robot should be declared.
"""
import wpilib
import commands2
import commands2.button
from typing import Optional

from constants import *
from subsystems.drivesubsystem import DriveSubsystem
from commands.defaultdrivecommand import DefaultDriveCommand

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
        # TODO: Initialize other subsystems
        # self.elevator = ElevatorSubsystem()
        # self.end_effector = EndEffectorSubsystem()

        # Configure button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.drive.setDefaultCommand(
            DefaultDriveCommand(
                self.drive,
                lambda: -self.driver_controller.getLeftY(),  # Forward/Backward
                lambda: -self.driver_controller.getLeftX(),  # Left/Right
                lambda: -self.driver_controller.getRightX(),  # Rotation
                lambda: self.driver_controller.getLeftBumper()  # Precision mode
            )
        )

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

        # TODO: Add other button bindings for:
        # - Elevator control
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
        # TODO: Return the command to run in autonomous
        # For now, return None which means no autonomous
        return None 