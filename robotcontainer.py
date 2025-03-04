import commands2
import wpilib
from wpilib import XboxController
from wpilib.interfaces import GenericHID
from constants.constants import DriveConstants, OIConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.end_effector import EndEffector
from subsystems.elevator_subsystem import ElevatorSubsystem
from commands.drive_command import DriveCommand
from commands.test_swerve_command import TestSwerveCommand
from commands.coral_intake_command import CoralIntakeCommand
from commands.algae_manipulate_command import AlgaeManipulateCommand
from commands.elevator_to_position_command import ElevatorToPositionCommand

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
        self.driver_controller = XboxController(OIConstants.DRIVER_CONTROLLER_PORT)
        self.operator_controller = XboxController(OIConstants.OPERATOR_CONTROLLER_PORT)

        # Create subsystems
        self.drive_subsystem = DriveSubsystem()
        self.end_effector = EndEffector()
        self.elevator_subsystem = ElevatorSubsystem()

        # Create commands
        self.test_swerve_command = TestSwerveCommand(self.drive_subsystem)
        
        # Create Elevator commands
        self.elevator_base_command = ElevatorToPositionCommand(
            self.elevator_subsystem,
            target_position=0.0
        )
        self.elevator_l1_command = ElevatorToPositionCommand(
            self.elevator_subsystem,
            target_position=0.5
        )
        self.elevator_l2_command = ElevatorToPositionCommand(
            self.elevator_subsystem,
            target_position=1.0
        )
        self.elevator_l3_command = ElevatorToPositionCommand(
            self.elevator_subsystem,
            target_position=1.5
        )
        self.elevator_l4_command = ElevatorToPositionCommand(
            self.elevator_subsystem,
            target_position=2.0
        )
        
        # Create Coral commands
        self.coral_intake_command = CoralIntakeCommand(self.end_effector)
        self.coral_outtake_command = CoralIntakeCommand(self.end_effector, outtake=True)
        
        # Create Algae commands for different positions
        self.algae_retract_command = AlgaeManipulateCommand(
            self.end_effector,
            target_position=0.0
        )
        self.algae_top_pickup_command = AlgaeManipulateCommand(
            self.end_effector,
            target_position=2.1  # ~120 degrees
        )
        self.algae_bottom_pickup_command = AlgaeManipulateCommand(
            self.end_effector,
            target_position=-0.52  # ~-30 degrees
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
        commands2.button.JoystickButton(
            self.driver_controller, 
            OIConstants.X_FORMATION_BUTTON
        ).onTrue(
            self.drive_subsystem.runOnce(lambda: self.drive_subsystem.setX())
        )
        
        commands2.button.JoystickButton(
            self.driver_controller, 
            OIConstants.RESET_GYRO_BUTTON
        ).onTrue(
            self.drive_subsystem.runOnce(lambda: self.drive_subsystem.zeroHeading())
        )
        
        commands2.button.JoystickButton(
            self.driver_controller, 
            OIConstants.FIELD_RELATIVE_TOGGLE_BUTTON
        ).onTrue(
            self.drive_subsystem.runOnce(lambda: self.drive_subsystem.toggleFieldRelative())
        )
        
        # Test swerve command (temporary)
        commands2.button.JoystickButton(
            self.driver_controller, 
            XboxController.Button.kY
        ).onTrue(
            self.test_swerve_command
        )

        # Operator controller bindings
        # Elevator controls
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ELEVATOR_BASE_BUTTON
        ).onTrue(
            self.elevator_base_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ELEVATOR_L1_BUTTON
        ).onTrue(
            self.elevator_l1_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ELEVATOR_L2_BUTTON
        ).onTrue(
            self.elevator_l2_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ELEVATOR_L3_BUTTON
        ).onTrue(
            self.elevator_l3_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ELEVATOR_L4_BUTTON
        ).onTrue(
            self.elevator_l4_command
        )
        
        # Coral manipulator controls
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.CORAL_INTAKE_BUTTON
        ).whileTrue(
            self.coral_intake_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.CORAL_OUTTAKE_BUTTON
        ).whileTrue(
            self.coral_outtake_command
        )
        
        # Algae manipulator controls
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ALGAE_RETRACTED_BUTTON
        ).onTrue(
            self.algae_retract_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ALGAE_TOP_PICKUP_BUTTON
        ).onTrue(
            self.algae_top_pickup_command
        )
        
        commands2.button.JoystickButton(
            self.operator_controller, 
            OIConstants.ALGAE_BOTTOM_PICKUP_BUTTON
        ).onTrue(
            self.algae_bottom_pickup_command
        )
        
        # Algae intake/outtake with triggers
        commands2.button.Trigger(
            lambda: self.operator_controller.getRawAxis(OIConstants.ALGAE_INTAKE_AXIS) > OIConstants.AXIS_THRESHOLD
        ).whileTrue(
            self.algae_intake_command
        )
        
        commands2.button.Trigger(
            lambda: self.operator_controller.getRawAxis(OIConstants.ALGAE_OUTTAKE_AXIS) > OIConstants.AXIS_THRESHOLD
        ).whileTrue(
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