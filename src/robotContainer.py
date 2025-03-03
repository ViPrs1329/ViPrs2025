# src/robotContainer.py
import commands2
from commands2.button import CommandXboxController
import wpilib
import math
import ntcore

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

from subsystems.SwerveDriveSubsystem import SwerveDrive
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffectorSubsystem import EndEffector

from constants import controllerConsts, driveConsts, elevatorConsts, endEffectorConsts

class RobotContainer:
    """
    Container for all robot components and subsystems.
    
    This class wires together controllers, subsystems, and commands.
    It also configures button bindings and default commands.
    """
    
    def __init__(self):
        """Initialize the RobotContainer."""
        # Create controllers
        self.driver_controller = CommandXboxController(controllerConsts.DRIVER_CONTROLLER_PORT)
        self.operator_controller = CommandXboxController(controllerConsts.OPERATOR_CONTROLLER_PORT)
        
        # Create subsystems
        self.drivetrain = SwerveDrive()
        self.elevator = Elevator()
        self.end_effector = EndEffector()
        
        # Initialize mode flags
        self.is_algae_mode = False
        self.is_coral_mode = False
        
        # Initialize NetworkTables
        self.setup_network_tables()
        
        # Configure button bindings
        self.configure_button_bindings()
        
        # Configure default commands
        self.configure_default_commands()
        
        print("RobotContainer initialized")
    
    def setup_network_tables(self):
        """Initialize network tables for telemetry."""
        instance = ntcore.NetworkTableInstance.getDefault()
        self.table = instance.getTable("RobotData")
        
        # Create publisher for operator mode
        self.mode_pub = self.table.getStringTopic("operator_mode").publish()
        self.mode_pub.set("Base")
    
    def configure_button_bindings(self):
        """Configure button bindings for user input."""
        # ============ Driver Controls ============
        # Reset gyro with Start button
        self.driver_controller.start().onTrue(
            commands2.RunCommand(
                lambda: self.drivetrain.resetGyro(),
                self.drivetrain
            )
        )
        
        # Toggle field-oriented driving with Back button
        self.driver_controller.back().onTrue(
            commands2.RunCommand(
                lambda: self.drivetrain.toggleFieldOriented(),
                self.drivetrain
            )
        )
        
        # Emergency stop with Y button
        self.driver_controller.y().onTrue(
            commands2.RunCommand(
                lambda: self.emergency_stop(),
                [self.drivetrain, self.elevator, self.end_effector]
            )
        )
        
        # ============ Operator Controls ============
        # Mode selection
        # A Button: Base Mode (Home position)
        self.operator_controller.a().onTrue(
            commands2.SequentialCommandGroup(
                commands2.InstantCommand(lambda: self.set_base_mode()),
                commands2.RunCommand(
                    lambda: self.elevator.moveToPosition(elevatorConsts.HOME_POSITION),
                    self.elevator
                )
            )
        )
        
        # X Button: Coral Mode
        self.operator_controller.x().onTrue(
            commands2.InstantCommand(lambda: self.set_coral_mode())
        )
        
        # Y Button: Algae Mode
        self.operator_controller.y().onTrue(
            commands2.InstantCommand(lambda: self.set_algae_mode())
        )
        
        # ---- Coral Mode Controls ----
        # Only active when in Coral mode
        coral_mode_active = commands2.button.Trigger(lambda: self.is_coral_mode)
        
        # Intake coral with left bumper
        coral_mode_active.and_(self.operator_controller.leftBumper()).whileTrue(
            commands2.RunCommand(
                lambda: self.end_effector.intakeCoral(),
                self.end_effector
            )
        )
        
        # Eject coral with right bumper
        coral_mode_active.and_(self.operator_controller.rightBumper()).whileTrue(
            commands2.RunCommand(
                lambda: self.end_effector.ejectCoral(),
                self.end_effector
            )
        )
        
        # X + A: Elevator to low position
        coral_mode_active.and_(self.operator_controller.x()).and_(self.operator_controller.a()).onTrue(
            commands2.RunCommand(
                lambda: self.elevator.moveToPosition(elevatorConsts.LOW_POSITION),
                self.elevator
            )
        )
        
        # X + X: Elevator to medium position
        coral_mode_active.and_(self.operator_controller.x()).and_(self.operator_controller.x()).onTrue(
            commands2.RunCommand(
                lambda: self.elevator.moveToPosition(elevatorConsts.MEDIUM_POSITION),
                self.elevator
            )
        )
        
        # X + Y: Elevator to high position
        coral_mode_active.and_(self.operator_controller.x()).and_(self.operator_controller.y()).onTrue(
            commands2.RunCommand(
                lambda: self.elevator.moveToPosition(elevatorConsts.HIGH_POSITION),
                self.elevator
            )
        )
        
        # ---- Algae Mode Controls ----
        # Only active when in Algae mode
        algae_mode_active = commands2.button.Trigger(lambda: self.is_algae_mode)
        
        # Intake algae with left bumper
        algae_mode_active.and_(self.operator_controller.leftBumper()).whileTrue(
            commands2.RunCommand(
                lambda: self.end_effector.setAlgaeIntakeSpeed(endEffectorConsts.ALGAE_INTAKE_SPEED),
                self.end_effector
            )
        )
        
        # Eject algae with right bumper
        algae_mode_active.and_(self.operator_controller.rightBumper()).whileTrue(
            commands2.RunCommand(
                lambda: self.end_effector.setAlgaeIntakeSpeed(-endEffectorConsts.ALGAE_INTAKE_SPEED),
                self.end_effector
            )
        )
        
        # Y + A: Algae to retracted position
        algae_mode_active.and_(self.operator_controller.y()).and_(self.operator_controller.a()).onTrue(
            commands2.RunCommand(
                lambda: self.end_effector.moveAlgaeToRetracted(),
                self.end_effector
            )
        )
        
        # Y + X: Algae to bottom pickup position
        algae_mode_active.and_(self.operator_controller.y()).and_(self.operator_controller.x()).onTrue(
            commands2.RunCommand(
                lambda: self.end_effector.moveAlgaeToBottomPickup(),
                self.end_effector
            )
        )
        
        # Y + Y: Algae to top pickup position
        algae_mode_active.and_(self.operator_controller.y()).and_(self.operator_controller.y()).onTrue(
            commands2.RunCommand(
                lambda: self.end_effector.moveAlgaeToTopPickup(),
                self.end_effector
            )
        )
        
        # ---- Manual Controls ----
        # Manual elevator control with right trigger + right joystick Y
        self.operator_controller.rightTrigger().whileTrue(
            commands2.RunCommand(
                lambda: self.manual_elevator_control(),
                self.elevator
            )
        )
        
        # Manual algae rotation with left trigger + right joystick Y
        self.operator_controller.leftTrigger().whileTrue(
            commands2.RunCommand(
                lambda: self.manual_algae_control(),
                self.end_effector
            )
        )
    
    def configure_default_commands(self):
        """Configure default commands for subsystems."""
        # Set default command for drivetrain - drive with controller
        self.drivetrain.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive_with_controller(),
                self.drivetrain
            )
        )
        
        # Set default command for elevator - hold position
        self.elevator.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.elevator.holdPosition(),
                self.elevator
            )
        )
    
    # ============ Mode Management ============
    
    def set_base_mode(self):
        """Set to Base mode."""
        self.is_coral_mode = False
        self.is_algae_mode = False
        self.mode_pub.set("Base")
        print("Operator Mode: Base")
    
    def set_coral_mode(self):
        """Set to Coral mode."""
        self.is_coral_mode = True
        self.is_algae_mode = False
        self.mode_pub.set("Coral")
        print("Operator Mode: Coral")
    
    def set_algae_mode(self):
        """Set to Algae mode."""
        self.is_coral_mode = False
        self.is_algae_mode = True
        self.mode_pub.set("Algae")
        print("Operator Mode: Algae")
    
    # ============ Manual Control Helpers ============
    
    def manual_elevator_control(self):
        """Handle manual elevator control."""
        # Get joystick Y (inverted so up is positive)
        joystick_y = -self.operator_controller.getRightY()
        
        # Apply deadband
        if abs(joystick_y) < controllerConsts.JOYSTICK_DEADBAND:
            joystick_y = 0
        
        # Scale for manual control
        speed = joystick_y * 0.4  # 40% speed for safety
        
        # Set manual speed
        self.elevator.setManualSpeed(speed)
    
    def manual_algae_control(self):
        """Handle manual algae rotation control."""
        # Get joystick Y (inverted so up is positive)
        joystick_y = -self.operator_controller.getRightY()
        
        # Apply deadband
        if abs(joystick_y) < controllerConsts.JOYSTICK_DEADBAND:
            joystick_y = 0
        
        # Scale for manual control
        speed = joystick_y * 0.3  # 30% speed for safety
        
        # Set manual speed
        self.end_effector.setAlgaeRotationSpeed(speed)
    
    def drive_with_controller(self):
        """Handle driving based on controller input."""
        # Get joystick values
        x_speed = -self.driver_controller.getLeftY()  # Forward/backward
        y_speed = -self.driver_controller.getLeftX()  # Left/right
        rot_speed = -self.driver_controller.getRightX()  # Rotation
        
        # Apply deadbands
        x_speed = self.apply_deadband(x_speed, controllerConsts.JOYSTICK_DEADBAND)
        y_speed = self.apply_deadband(y_speed, controllerConsts.JOYSTICK_DEADBAND)
        rot_speed = self.apply_deadband(rot_speed, controllerConsts.ROTATION_DEADBAND)
        
        # Apply speed scaling
        x_speed *= controllerConsts.DRIVE_SPEED_SCALE
        y_speed *= controllerConsts.DRIVE_SPEED_SCALE
        rot_speed *= controllerConsts.ROTATION_SPEED_SCALE
        
        # Apply cube-function for smoother control
        x_speed = math.copysign(x_speed * x_speed * x_speed, x_speed)
        y_speed = math.copysign(y_speed * y_speed * y_speed, y_speed)
        rot_speed = math.copysign(rot_speed * rot_speed * rot_speed, rot_speed)
        
        # Check for boost/slow mode
        if self.driver_controller.rightBumper().getAsBoolean():
            # Boost mode
            x_speed *= controllerConsts.BOOST_MULTIPLIER
            y_speed *= controllerConsts.BOOST_MULTIPLIER
            rot_speed *= controllerConsts.BOOST_MULTIPLIER
        elif self.driver_controller.leftBumper().getAsBoolean():
            # Slow mode
            x_speed *= controllerConsts.SLOW_MULTIPLIER
            y_speed *= controllerConsts.SLOW_MULTIPLIER
            rot_speed *= controllerConsts.SLOW_MULTIPLIER
        
        # Convert to meters per second
        x_speed *= driveConsts.MAX_SPEED
        y_speed *= driveConsts.MAX_SPEED
        rot_speed *= 2 * math.pi  # Convert to radians per second
        
        # Drive the robot
        self.drivetrain.drive(x_speed, y_speed, rot_speed)
    
    def apply_deadband(self, value, deadband):
        """Apply deadband to a value."""
        if abs(value) < deadband:
            return 0
        
        # Scale the remaining values to cover the full range
        return (value - math.copysign(deadband, value)) / (1.0 - deadband)
    
    def emergency_stop(self):
        """Stop all subsystems in case of emergency."""
        self.drivetrain.stopMotors()
        self.elevator.stopMotors()
        self.end_effector.stopMotors()
        wpilib.SmartDashboard.putString("Robot/Status", "EMERGENCY STOP")
        print("*** EMERGENCY STOP ACTIVATED ***")
    
    # ============ Autonomous ============
    
    def getAutonomousCommand(self):
        """
        Get the autonomous command to run.
        
        Returns:
            Command: The command to run during autonomous
        """
        # Import here to avoid circular imports
        from commands.AutonomousCommands import LeaveStartingZoneAuto
        
        # Get autonomous selection from NetworkTables if available
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = nt_inst.getTable("Autonomous")
        selected_routine = auto_table.getStringTopic("selected_routine").subscribe("DefaultAuto").get()
        
        # Choose the appropriate command based on selection
        if selected_routine == "LeaveStartingZone":
            return LeaveStartingZoneAuto(self.drivetrain)
        # Add other auto routines as needed
        
        # Default to a simple routine if no valid selection
        return LeaveStartingZoneAuto(self.drivetrain)
    
    def systemTempCheck(self):
        """
        Check system temperatures.
        
        Returns:
            bool: True if any component exceeds temperature limits
        """
        warning_temps = []
        
        # Check elevator motors
        left_temp = self.elevator.cache.left_temp
        right_temp = self.elevator.cache.right_temp
        
        # Check end effector motors
        coral_left_temp = self.end_effector.cache.motor_temps["CoralLeft"]
        coral_right_temp = self.end_effector.cache.motor_temps["CoralRight"]
        algae_rot_temp = self.end_effector.cache.motor_temps["AlgaeRotation"]
        algae_intake_temp = self.end_effector.cache.motor_temps["AlgaeIntake"]
        
        # Add any hot components to the warning list
        temp_threshold = 80  # Degrees Celsius
        
        if left_temp > temp_threshold:
            warning_temps.append(f"Elevator Left: {left_temp:.1f}°C")
            
        if right_temp > temp_threshold:
            warning_temps.append(f"Elevator Right: {right_temp:.1f}°C")
            
        if coral_left_temp > temp_threshold:
            warning_temps.append(f"Coral Left: {coral_left_temp:.1f}°C")
            
        if coral_right_temp > temp_threshold:
            warning_temps.append(f"Coral Right: {coral_right_temp:.1f}°C")
            
        if algae_rot_temp > temp_threshold:
            warning_temps.append(f"Algae Rotation: {algae_rot_temp:.1f}°C")
            
        if algae_intake_temp > temp_threshold:
            warning_temps.append(f"Algae Intake: {algae_intake_temp:.1f}°C")
        
        # Report warnings to dashboard
        if warning_temps:
            warning_str = ", ".join(warning_temps)
            wpilib.SmartDashboard.putString("Robot/TempWarnings", warning_str)
            print(f"WARNING: Temperature limits exceeded - {warning_str}")
            return True
        
        wpilib.SmartDashboard.putString("Robot/TempWarnings", "None")
        return False