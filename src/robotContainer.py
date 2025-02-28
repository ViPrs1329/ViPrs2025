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
from subsystems.ElevatorSubsystem import Elevator
from commands.IntakeCommands import IntakeCoralCommand, EjectCoralCommand
from commands.ElevatorCommands import (
    ElevatorHomePositionCommand,
    ElevatorLowPositionCommand,
    ElevatorMediumPositionCommand,
    ElevatorHighPositionCommand
)
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
        self.elevator = Elevator()  # Initialize the elevator subsystem
        
        # Initialize network tables
        self.setup_network_tables()
        
        # Create EndEffector commands
        self.intakeCoralCommand = IntakeCoralCommand(self.endEffector)
        self.ejectCoralCommand = EjectCoralCommand(self.endEffector)
        
        # Create elevator commands
        self.elevatorHomeCommand = ElevatorHomePositionCommand(self.elevator)
        self.elevatorLowCommand = ElevatorLowPositionCommand(self.elevator)
        self.elevatorMediumCommand = ElevatorMediumPositionCommand(self.elevator)
        self.elevatorHighCommand = ElevatorHighPositionCommand(self.elevator)
        
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
        # Reset drivetrain gyro with press of Start button
        self.drivingController.start().onTrue(
            commands2.InstantCommand(lambda: self.drivetrain.resetHarder())
        )
        
        # Operator controls - EndEffector
        # X button for intake coral
        self.operatorController.x().onTrue(self.intakeCoralCommand)
        # B button for eject coral
        self.operatorController.b().onTrue(self.ejectCoralCommand)
        
        # Operator controls - Elevator
        # A button for home position
        self.operatorController.a().onTrue(self.elevatorHomeCommand)
        # Y button for low position
        self.operatorController.y().onTrue(self.elevatorLowCommand)
        # Left bumper for medium position
        self.operatorController.leftBumper().onTrue(self.elevatorMediumCommand)
        # Right bumper for high position
        self.operatorController.rightBumper().onTrue(self.elevatorHighCommand)
        
        # Manual elevator control - Uses the right joystick Y-axis for manual elevator control when held
        # This is triggered by holding the right trigger
        self.operatorController.rightTrigger().whileTrue(
            RunCommand(
                lambda: self.manualElevatorControl(),
                self.elevator
            )
        )

    def configureDefaultCommands(self):
        """Configure default commands for subsystems."""
        # Create a default command for driving
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drive_with_controller(),
                self.drivetrain  # Pass subsystem directly, not in a list
            )
        )
        
        # Default command for the elevator to hold position
        self.elevator.setDefaultCommand(
            RunCommand(
                lambda: self.elevator.holdPosition(),
                self.elevator
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

    def manualElevatorControl(self):
        """Control the elevator manually with the operator controller."""
        # Get the Y-axis of the right joystick (inverted so up is positive)
        joystick_y = -self.operatorController.getRightY()
        
        # Apply deadband to prevent small unintended movements
        if abs(joystick_y) < 0.1:
            joystick_y = 0
            
        # Scale the joystick input to appropriate elevator speed
        # You might want to adjust the scaling factor based on your elevator
        elevator_speed = joystick_y * 0.5  # 50% of full speed for manual control
        
        # Set the elevator speed
        self.elevator.setManualSpeed(elevator_speed)

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
            self.drivetrain.frontRightRotation,
            # Add elevator motors to temperature check
            self.elevator.LEM,
            self.elevator.REM
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