import commands2
import wpilib
from wpilib import XboxController
from wpilib.interfaces import GenericHID
from constants.constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.end_effector import EndEffector
from commands.drive_command import DriveCommand
from commands.test_swerve_command import TestSwerveCommand
from commands.coral_intake_command import CoralIntakeCommand
from commands.algae_manipulate_command import AlgaeManipulateCommand

class RobotContainer:
    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled in the Robot periodic methods (other than the scheduler calls).
    The structure of this class is the following:
    
    1. Instance variables for robot subsystems
    2. Instance variables for commands
    3. Instance variables for controller(s)
    4. The constructor doing the following:
       - Creating all subsystems
       - Creating the autonomous chooser
       - Configuring button bindings
    """

    def __init__(self):
        """Constructor"""
        # Create controllers
        self.driver_controller = XboxController(DriveConstants.DRIVER_CONTROLLER_PORT)
        self.operator_controller = XboxController(DriveConstants.OPERATOR_CONTROLLER_PORT)

        # Create subsystems
        self.drive_subsystem = DriveSubsystem()
        self.end_effector = EndEffector()

        # Create commands
        self.test_swerve_command = TestSwerveCommand(self.drive_subsystem)
        self.coral_intake_command = CoralIntakeCommand(self.end_effector)
        
        # Create Algae commands for different positions
        self.algae_retract_command = AlgaeManipulateCommand(
            self.end_effector,
            target_position=0.0
        )
        self.algae_pickup_command = AlgaeManipulateCommand(
            self.end_effector,
            target_position=2.1  # ~120 degrees
        )
        self.algae_intake_command = AlgaeManipulateCommand(
            self.end_effector,
            intake_speed=0.8
        )
        self.algae_outtake_command = AlgaeManipulateCommand(
            self.end_effector,
            intake_speed=-0.8
        )

        # Set default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(
                self.drive_subsystem,
                self.driver_controller
            )
        )

        # Configure button bindings
        self.configureButtonBindings()

        # Create autonomous chooser
        self.autonomous_chooser = wpilib.SendableChooser()
        # TODO: Add autonomous options
        wpilib.SmartDashboard.putData("Auto Mode", self.autonomous_chooser)

    def configureButtonBindings(self):
        """
        Use this method to define button->command mappings. Buttons can be created via the
        button factories on Controllers or Commands.
        """
        # Driver controller bindings
        commands2.button.JoystickButton(self.driver_controller, XboxController.Button.kY).onTrue(
            self.test_swerve_command
        )

        # Operator controller bindings
        # Coral intake
        commands2.button.JoystickButton(self.operator_controller, XboxController.Button.kA).onTrue(
            self.coral_intake_command
        )
        
        # Algae manipulation
        commands2.button.JoystickButton(self.operator_controller, XboxController.Button.kX).onTrue(
            self.algae_retract_command
        )
        commands2.button.JoystickButton(self.operator_controller, XboxController.Button.kY).onTrue(
            self.algae_pickup_command
        )
        commands2.button.JoystickButton(self.operator_controller, XboxController.Button.kB).onTrue(
            self.algae_intake_command
        )
        commands2.button.JoystickButton(self.operator_controller, XboxController.Button.kA).onTrue(
            self.algae_outtake_command
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main Robot class.
        
        Returns
        -------
        Command: the command to run in autonomous
        """
        return self.autonomous_chooser.getSelected() 