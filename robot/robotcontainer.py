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
from subsystems.algaesubsystem import AlgaeSubsystem
from commands.drivecommands import DefaultDriveCommand
from commands.elevatorcommands import SetElevatorHeight, ManualElevatorControl
from commands.coralcommands import IntakeCoral, EjectCoral
from commands.algaecommands import SetArmPosition, IntakeAlgae, EjectAlgae

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
        self.algae = AlgaeSubsystem()
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
        
        # Elevator height controls
        self.operator_controller.a().onTrue(SetElevatorHeight(self.elevator, "BASE"))
        self.operator_controller.b().onTrue(SetElevatorHeight(self.elevator, "L1"))
        self.operator_controller.x().onTrue(SetElevatorHeight(self.elevator, "L2"))
        self.operator_controller.y().onTrue(SetElevatorHeight(self.elevator, "L3"))
        
        # Manual elevator control
        self.operator_controller.leftBumper().whileTrue(ManualElevatorControl(self.elevator, 0.5))
        self.operator_controller.rightBumper().whileTrue(ManualElevatorControl(self.elevator, -0.5))
        
        # CORAL controls
        self.operator_controller.leftTrigger().whileTrue(IntakeCoral(self.coral))
        self.operator_controller.rightTrigger().whileTrue(EjectCoral(self.coral))
        
        # ALGAE controls
        self.operator_controller.dpadUp().onTrue(SetArmPosition(self.algae, ALGAE_ARM_WORKING_ANGLE))
        self.operator_controller.dpadDown().onTrue(SetArmPosition(self.algae, ALGAE_ARM_REST_ANGLE))
        self.operator_controller.dpadLeft().whileTrue(IntakeAlgae(self.algae))
        self.operator_controller.dpadRight().whileTrue(EjectAlgae(self.algae))

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