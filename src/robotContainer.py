# robotContainer.py
import commands2
from commands2.button import CommandXboxController
from commands2 import RunCommand

import math
import ntcore
import wpimath
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.EndEffector import EndEffector
from commands.IntakeCommands import IntakeCoralCommand
import constants

class RobotContainer:
    """
    This class is where the bulk of the robot's resources are declared.
    The RobotContainer handles wiring together all subsystems, controllers, 
    and commands to create the full robot functionality.
    """
    def __init__(self) -> None:
        # Create controllers
        self.drivingController = CommandXboxController(0)
        self.operatorController = CommandXboxController(1) 
        
        # Initialize subsystems
        self.drivetrain = DriveTrain()
        self.endEffector = EndEffector()
        
        # Initialize network tables
        self.setup_network_tables()
        
        # Create commands
        self.intakeCoralCommand = IntakeCoralCommand(self.endEffector)
        
        # Configure button bindings
        self.configureButtonBindings()
        
        # Configure default commands
        self.configureDefaultCommands()

    def setup_network_tables(self):
        """Initialize network tables for telemetry."""
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable("datatable")
        self.controllerXPub = self.table.getDoubleTopic("controller x").publish()
        self.controllerYPub = self.table.getDoubleTopic("controller y").publish()
        self.robotPosition = self.table.getStructTopic("robot pose", wpimath.geometry.Pose2d).publish()

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
        # Driver controls
        # Example: self.drivingController.a().onTrue(commands2.InstantCommand(lambda: self.drivetrain.resetGyro()))
        
        # Operator controls - Intake Coral with the X button
        self.operatorController.x().onTrue(self.intakeCoralCommand)
        
        # Additional controls can be added here

    def configureDefaultCommands(self):
        """Configure default commands for subsystems."""
        # Create a default command for driving
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drive_with_controller(),
                self.drivetrain  # Pass subsystem directly, not in a list
            )
        )

    def inputCurve(self, input: float):
        """Apply a cubic curve to input for smoother control."""
        return (input ** 3)

    def tinputCurve(self, input: float):
        """Apply a cubic curve to rotational input."""
        return (input ** 3) * constants.controller.tscale

    def distanceCorrectedInputCurve(self, x: float, y: float):
        """Apply distance-corrected input curve for smoother control."""
        d = math.sqrt(x * x + y * y)
        s = self.inputCurve(d)
        sx = x * s
        sy = y * s
        if sx * sx + sy * sy > 1:
            scale = 1 / math.sqrt(sx * sx + sy * sy)
            sx *= scale
            sy *= scale
        return sx * constants.controller.scale, sy * constants.controller.scale

    def drive_with_controller(self):
        """Drive the robot based on controller input."""
        # Get joystick inputs and apply curves for smoother control
        xSpeed, ySpeed = self.distanceCorrectedInputCurve(
            self.drivingController.getLeftY(), 
            self.drivingController.getLeftX()
        )
        
        # Publish to network tables for telemetry
        self.controllerXPub.set(xSpeed)
        self.controllerYPub.set(ySpeed)

        # Get rotation input with curve applied
        tSpeed = self.tinputCurve(-self.drivingController.getRightX())

        # Apply deadzone to eliminate small unwanted inputs
        if abs(xSpeed) < constants.controller.XYdeadzone:
            xSpeed = 0
        if abs(ySpeed) < constants.controller.XYdeadzone:
            ySpeed = 0
        if abs(tSpeed) < constants.controller.Tdeadzone:
            tSpeed = 0

        # Get current yaw from gyro
        yaw = self.drivetrain.gyro.get_yaw().value_as_double

        # Normalize heading to 0-360 degrees
        h = yaw % 360
        if h < 0:
            h += 360

        # Convert to radians for field-relative driving
        h2 = h / 360
        heading = h2 * (math.pi * 2)

        # Create field-relative chassis speeds
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, -tSpeed, Rotation2d(heading)
        )
        
        # Drive the swerve drivetrain with calculated speeds
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Update robot position for telemetry
        self.robotPosition.set(self.drivetrain.combinedPosition)

    def getAutonomousCommand(self):
        """Return the command to run in autonomous mode."""
        # TODO: Implement autonomous command(s)
        return commands2.InstantCommand()  # Placeholder

    def systemTempCheck(self):
        """Check temperature of motor controllers."""
        motorControllers = [
            self.drivetrain.frontLeftDrive,
            self.drivetrain.frontRightDrive,
            self.drivetrain.backLeftDrive,
            self.drivetrain.backRightDrive,
            self.drivetrain.backLeftRotation,
            self.drivetrain.backRightRotation,
            self.drivetrain.frontLeftRotation,
            self.drivetrain.frontRightRotation
        ]

        burntFlag = False
        for motorController in motorControllers:
            temp = motorController.getMotorTemperature()
            if temp > 90:
                print(f"[x] Motor {motorController.getDeviceId()}, {temp}C")
                burntFlag = True
            else:
                print(f"[-] Motor {motorController.getDeviceId()}, {temp}C")
        
        return burntFlag