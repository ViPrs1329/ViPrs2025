# robotContainer.py
import commands2
from commands2.button import CommandXboxController, Trigger
from commands2 import RunCommand, button, Command

import math
import ntcore
import wpimath
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.EndEffector import EndEffector
from subsystems.ElevatorSubsystem import Elevator
from commands.AutonomousCommands import LeaveStartingZoneAuto
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
        self.elevator = Elevator()
        
        # Create mode state tracking
        self.isAlgaeMode = False
        self.isCoralMode = False
        
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
        
        # Add mode indicator to dashboard
        self.modePub = self.table.getStringTopic("operator mode").publish()
        self.modePub.set("Base")

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
        # Driver controls
        # Reset drivetrain gyro with press of Start button
        self.drivingController.start().onTrue(
            commands2.InstantCommand(lambda: self.drivetrain.resetHarder())
        )
        
        # Toggle field-oriented control with Back/Select button
        self.drivingController.back().onTrue(
            commands2.InstantCommand(lambda: self.toggleFieldOriented())
        )
        
        # Emergency stop with Y button
        self.drivingController.y().onTrue(
            commands2.InstantCommand(lambda: self.emergencyStop())
        )
        
        # Operator controls - Mode Selection
        # A Button: Base Mode (Elevator to home position)
        self.operatorController.a().onTrue(
            commands2.SequentialCommandGroup(
                commands2.InstantCommand(lambda: self.setBaseMode()),
                self.elevatorHomeCommand
            )
        )
        
        # X Button: Coral Mode
        self.operatorController.x().onTrue(
            commands2.InstantCommand(lambda: self.setCoralMode())
        )
        
        # Y Button: Algae Mode
        self.operatorController.y().onTrue(
            commands2.InstantCommand(lambda: self.setAlgaeMode())
        )
        
        # Create conditional triggers based on mode
        # Coral Mode Controls
        coralModeActive = Trigger(lambda: self.isCoralMode)
        
        # Coral intake/expel in Coral Mode
        coralModeActive.and_(self.operatorController.leftBumper()).onTrue(self.intakeCoralCommand)
        coralModeActive.and_(self.operatorController.rightBumper()).onTrue(self.ejectCoralCommand)
        
        # Elevator positions in Coral Mode using D-Pad
        coralModeActive.and_(self.operatorController.povUp()).onTrue(self.elevatorHighCommand)
        coralModeActive.and_(self.operatorController.povLeft()).onTrue(self.elevatorMediumCommand)
        coralModeActive.and_(self.operatorController.povRight()).onTrue(self.elevatorLowCommand)
        coralModeActive.and_(self.operatorController.povDown()).onTrue(self.elevatorHomeCommand)
        
        # Algae Mode Controls
        algaeModeActive = Trigger(lambda: self.isAlgaeMode)
        
        # Import algae commands
        from commands.AlgaeCommands import AlgaeTopPickupCommand, AlgaeBottomPickupCommand, AlgaeRetractedCommand
        
        # Algae intake positions in Algae Mode using D-Pad
        algaeModeActive.and_(self.operatorController.povUp()).onTrue(
            AlgaeTopPickupCommand(self.endEffector)
        )
        algaeModeActive.and_(self.operatorController.povDown()).onTrue(
            AlgaeBottomPickupCommand(self.endEffector)
        )
        algaeModeActive.and_(self.operatorController.povLeft()).onTrue(
            AlgaeRetractedCommand(self.endEffector)
        )
        
        # Import algae intake/eject commands
        from commands.AlgaeCommands import AlgaeIntakeCommand, AlgaeEjectCommand
        # Import scoring commands
        from commands.ScoringCommands import ScoreLowCommand, ScoreMediumCommand, ScoreHighCommand
        
        # Algae intake/expel in Algae Mode
        algaeModeActive.and_(self.operatorController.leftBumper()).whileTrue(
            AlgaeIntakeCommand(self.endEffector)
        )
        
        algaeModeActive.and_(self.operatorController.rightBumper()).whileTrue(
            AlgaeEjectCommand(self.endEffector)
        )
        
        # Quick-score commands with B button + D-pad in Coral Mode
        b_button_held = self.operatorController.b()
        coralModeActive.and_(b_button_held).and_(self.operatorController.povUp()).onTrue(
            ScoreHighCommand(self.elevator, self.endEffector)
        )
        
        coralModeActive.and_(b_button_held).and_(self.operatorController.povLeft()).onTrue(
            ScoreMediumCommand(self.elevator, self.endEffector)
        )
        
        coralModeActive.and_(b_button_held).and_(self.operatorController.povRight()).onTrue(
            ScoreLowCommand(self.elevator, self.endEffector)
        )
        
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
                self.drivetrain
            )
        )
        
        # Default command for the elevator to hold position
        self.elevator.setDefaultCommand(
            RunCommand(
                lambda: self.elevator.holdPosition(),
                self.elevator
            )
        )

    # Mode management methods
    def setBaseMode(self):
        """Set the operator controller to Base mode."""
        self.isCoralMode = False
        self.isAlgaeMode = False
        self.modePub.set("Base")
        print("Operator Mode: Base")
        
    def setCoralMode(self):
        """Set the operator controller to Coral mode."""
        self.isCoralMode = True
        self.isAlgaeMode = False
        self.modePub.set("Coral")
        print("Operator Mode: Coral")
        
    def setAlgaeMode(self):
        """Set the operator controller to Algae mode."""
        self.isCoralMode = False
        self.isAlgaeMode = True
        self.modePub.set("Algae")
        print("Operator Mode: Algae")
    
    # Algae control methods
    def setAlgaePosition(self, position):
        """Set the algae intake to a specific position.
        
        Args:
            position (str): Position name ("top", "bottom", "retracted")
        """
        print(f"Setting algae position to: {position}")
        
        # Implementation will depend on your algae mechanism
        # This is a placeholder - replace with actual implementation
        if position == "top":
            # Command to move algae to top position
            # For example, rotate to 90 degrees
            self.endEffector.setAlgaeRotationSpeed(0.5)  # Example only
            # In real implementation, you would use a position-based command
        elif position == "bottom":
            # Command to move algae to bottom position
            # For example, rotate to -90 degrees
            self.endEffector.setAlgaeRotationSpeed(-0.5)  # Example only
        elif position == "retracted":
            # Command to move algae to retracted position
            # For example, rotate to 0 degrees
            self.endEffector.setAlgaeRotationSpeed(0)  # Example only
    
    def setAlgaeIntakeSpeed(self, speed):
        """Set the algae intake speed.
        
        Args:
            speed (float): Speed value (-1.0 to 1.0)
        """
        print(f"Setting algae intake speed to: {speed}")
        self.endEffector.setAlgaeIntakeSpeed(speed)
    
    # Drive control methods
    field_oriented = True
    
    def toggleFieldOriented(self):
        """Toggle between field-oriented and robot-oriented driving."""
        self.field_oriented = not self.field_oriented
        print(f"Field-oriented driving: {self.field_oriented}")
    
    def emergencyStop(self):
        """Emergency stop all robot subsystems."""
        print("EMERGENCY STOP")
        self.drivetrain.stopMotors()
        self.elevator.stopMotors()
        self.endEffector.stopAllMotors()

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
        
        # Apply boost or precision mode if configured
        if self.drivingController.getRightBumper():
            # Boost mode - increase speed
            xSpeed *= 1.5
            ySpeed *= 1.5
        elif self.drivingController.getLeftBumper():
            # Precision mode - reduce speed
            xSpeed *= 0.5
            ySpeed *= 0.5
        
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

        # Create chassis speeds (field-relative or robot-relative)
        if self.field_oriented:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, -tSpeed, Rotation2d(heading)
            )
        else:
            speeds = ChassisSpeeds(xSpeed, ySpeed, -tSpeed)
        
        # Drive the swerve drivetrain with calculated speeds
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Update robot position for telemetry
        try:
            self.robotPosition.set(self.drivetrain.getPose())
        except:
            pass  # In case getPose() is not properly implemented

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
        auto_command = LeaveStartingZoneAuto(self.drivetrain)
        
        # Publish autonomous command details to NetworkTables for debugging
        inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = inst.getTable("Autonomous")
        command_name_pub = auto_table.getStringTopic("current_command").publish()
        command_name_pub.set(auto_command.getName())
        
        return auto_command

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