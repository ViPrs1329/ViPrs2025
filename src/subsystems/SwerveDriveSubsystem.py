# src/subsystems/SwerveDriveSubsystem.py
import rev
import math
import commands2
import wpilib
import ntcore

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath import controller
from wpilib import DriverStation

from constants import CANIDs, driveConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory
from subsystems.BaseSubsystem import BaseSubsystem
from phoenix6.hardware import CANcoder, Pigeon2


def lratio(angle):
    """Converts -pi, pi to -.5, .5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    """Convert CANcoder ticks to radians"""
    return (something/.5)*-math.pi

def deg2Rot2d(deg) -> Rotation2d:
    """Convert degrees to Rotation2d"""
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)

def ticks2radODOMETRY(something):
    """Convert CANcoder ticks to radians for odometry"""
    # units are in rotations
    return something * 2 * math.pi

def getSwerveModPos(rotEnc: CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    """Get the position of a swerve module for odometry"""
    return SwerveModulePosition(
        # Convert drive encoder position to meters
        (driveEnc.getPosition()/driveConsts.DRIVE_GEAR_RATIO)*driveConsts.WHEEL_CIRCUMFERENCE,
        # Convert rotation encoder position to Rotation2d
        Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    )


class SwerveModule:
    """
    Class representing a single swerve module with drive and rotation motors.
    """
    def __init__(self, name: str, drive_id: int, rotation_id: int, encoder_id: int, 
                 location: Translation2d, drive_inverted: bool = False, rot_inverted: bool = False):
        """
        Initialize a swerve module.
        
        Args:
            name (str): The name of this module (e.g., "FrontLeft")
            drive_id (int): CAN ID of the drive motor
            rotation_id (int): CAN ID of the rotation motor
            encoder_id (int): CAN ID of the CANcoder
            location (Translation2d): Position of this module relative to robot center
            drive_inverted (bool): Whether to invert the drive motor
            rot_inverted (bool): Whether to invert the rotation motor
        """
        self.name = name
        self.location = location
        
        # Create motor configurations
        drive_config = SparkMaxFactory.Configuration()
        drive_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        drive_config.current_limit = driveConsts.currentLimit
        drive_config.voltage_comp_enabled = True
        drive_config.voltage_comp_saturation = 12.0
        drive_config.inverted = drive_inverted
        
        rotation_config = SparkMaxFactory.Configuration()
        rotation_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        rotation_config.current_limit = driveConsts.currentLimit
        rotation_config.voltage_comp_enabled = True
        rotation_config.inverted = rot_inverted
        
        # Create motors
        self.drive_motor = SparkMaxFactory.createSparkMax(drive_id, drive_config)
        self.rotation_motor = SparkMaxFactory.createSparkMax(rotation_id, rotation_config)
        
        # Get drive encoder
        self.drive_encoder = self.drive_motor.getEncoder()
        
        # Create CANcoder
        self.rotation_encoder = CANcoder(encoder_id)
        
        # Create PID controller for rotation
        Kp = 4.0  # Proportional gain for rotation control
        self.rotation_pid = controller.PIDController(Kp, 0, 0)
        self.rotation_pid.enableContinuousInput(-.5, .5)
        self.rotation_pid.setSetpoint(0.0)
        
        # Cache for sensor values
        self.drive_position = 0.0
        self.rotation_position = 0.0
        self.drive_current = 0.0
        self.rotation_current = 0.0
        
    def cacheSensors(self):
        """Cache sensor values to reduce CAN traffic"""
        try:
            self.drive_position = self.drive_encoder.getPosition()
            self.rotation_position = self.rotation_encoder.get_absolute_position()._value
        except Exception as e:
            print(f"Error caching {self.name} module sensors: {e}")
    
    def cacheCurrents(self):
        """Cache current values (called less frequently)"""
        try:
            self.drive_current = self.drive_motor.getOutputCurrent()
            self.rotation_current = self.rotation_motor.getOutputCurrent()
        except Exception as e:
            print(f"Error caching {self.name} module currents: {e}")
    
    def getPosition(self) -> SwerveModulePosition:
        """Get the position of this module for odometry"""
        return getSwerveModPos(self.rotation_encoder, self.drive_encoder)
    
    def setDesiredState(self, desired_state: SwerveModuleState):
        """Set the desired state of this module"""
        try:
            # Optimize the module state to avoid spinning more than 90 degrees
            optimized_state = SwerveModuleState.optimize(
                desired_state,
                Rotation2d(ticks2rad(self.rotation_position))
            )
            
            # Calculate PID output for rotation
            rotation_output = -self.rotation_pid.calculate(
                self.rotation_position, 
                lratio(optimized_state.angle.radians())
            )
            
            # Limit rotation output to valid range
            rotation_output = max(min(rotation_output, 1.0), -1.0)
            
            # Set rotation motor
            self.rotation_motor.set(rotation_output)
            
            # Set drive motor
            max_voltage = 12.0
            normalized_speed = optimized_state.speed / driveConsts.MAX_SPEED
            self.drive_motor.setVoltage(normalized_speed * max_voltage)
            
        except Exception as e:
            print(f"Error setting {self.name} module state: {e}")
            self.stop()
    
    def stop(self):
        """Stop this module's motors"""
        self.drive_motor.set(0)
        self.rotation_motor.set(0)


class SwerveDrive(BaseSubsystem):
    """
    Swerve drive subsystem using four swerve modules.
    """
    def __init__(self):
        """Initialize the swerve drive subsystem."""
        super().__init__("SwerveDrive")
        
        try:
            # Create locations for swerve modules
            # Assuming square drivetrain with modules at the corners
            lv = driveConsts.WHEELBASE / 2  # Distance from center to module
            
            # Initialize swerve modules
            self.frontLeftModule = SwerveModule(
                "FrontLeft",
                CANIDs.SwerveModuleDrive1,
                CANIDs.SwerveModuleRotation1,
                CANIDs.EncoderModuleRotation1,
                Translation2d(lv, lv)
            )
            
            self.frontRightModule = SwerveModule(
                "FrontRight",
                CANIDs.SwerveModuleDrive4,
                CANIDs.SwerveModuleRotation4,
                CANIDs.EncoderModuleRotation4,
                Translation2d(lv, -lv)
            )
            
            self.backLeftModule = SwerveModule(
                "BackLeft",
                CANIDs.SwerveModuleDrive2,
                CANIDs.SwerveModuleRotation2,
                CANIDs.EncoderModuleRotation2,
                Translation2d(-lv, lv)
            )
            
            self.backRightModule = SwerveModule(
                "BackRight",
                CANIDs.SwerveModuleDrive3,
                CANIDs.SwerveModuleRotation3,
                CANIDs.EncoderModuleRotation3,
                Translation2d(-lv, -lv)
            )
            
            # Initialize gyro
            self.gyro = Pigeon2(CANIDs.PigeonID)
            
            # Reset gyro to zero heading
            self.resetGyro()
            
            # Create kinematics object
            self.kinematics = SwerveDrive4Kinematics(
                self.frontLeftModule.location,
                self.frontRightModule.location,
                self.backLeftModule.location,
                self.backRightModule.location
            )
            
            # Initialize odometry
            self.odometry = SwerveDrive4Odometry(
                self.kinematics,
                Rotation2d(),
                (
                    self.frontLeftModule.getPosition(),
                    self.frontRightModule.getPosition(),
                    self.backLeftModule.getPosition(),
                    self.backRightModule.getPosition()
                ),
                Pose2d()
            )
            
            # Track the last commanded chassis speeds
            self.last_chassis_speed = ChassisSpeeds(0, 0, 0)
            
            # Field-oriented driving
            self.field_oriented = True
            
            # Cache for sensor values
            self.gyro_yaw = 0.0
            self.gyro_pitch = 0.0
            self.gyro_roll = 0.0
            self.robot_pose = Pose2d()
            
            # Initial states
            self.modules = [self.frontLeftModule, self.frontRightModule, 
                           self.backLeftModule, self.backRightModule]
            
            # Cache initial sensor values
            self.cacheSensors()
            
            # Create NetworkTables entries
            self.setupNetworkTables()
            
            # Set status to ready
            wpilib.SmartDashboard.putString(f"{self.subsystem_name}/Status", "Ready")
            
        except Exception as e:
            self.handleError("__init__", e)
            self.modules = []  # Empty if initialization failed
    
    def setupNetworkTables(self):
        """Set up NetworkTables entries for telemetry."""
        try:
            self.nt_instance = ntcore.NetworkTableInstance.getDefault()
            self.table = self.nt_instance.getTable("SwerveDrive")
            
            # Create pose publishers
            self.pose_x_pub = self.table.getDoubleTopic("pose_x").publish()
            self.pose_y_pub = self.table.getDoubleTopic("pose_y").publish()
            self.pose_rot_pub = self.table.getDoubleTopic("pose_rot").publish()
            
            # Create gyro publishers
            self.gyro_yaw_pub = self.table.getDoubleTopic("gyro_yaw").publish()
            self.gyro_pitch_pub = self.table.getDoubleTopic("gyro_pitch").publish()
            self.gyro_roll_pub = self.table.getDoubleTopic("gyro_roll").publish()
            
            # Create speed publishers
            self.speed_x_pub = self.table.getDoubleTopic("speed_x").publish()
            self.speed_y_pub = self.table.getDoubleTopic("speed_y").publish()
            self.speed_rot_pub = self.table.getDoubleTopic("speed_rot").publish()
            
            # Initialize with zeros
            self.pose_x_pub.set(0)
            self.pose_y_pub.set(0)
            self.pose_rot_pub.set(0)
            
            self.gyro_yaw_pub.set(0)
            self.gyro_pitch_pub.set(0)
            self.gyro_roll_pub.set(0)
            
            self.speed_x_pub.set(0)
            self.speed_y_pub.set(0)
            self.speed_rot_pub.set(0)
            
        except Exception as e:
            self.handleError("setupNetworkTables", e)
    
    def cacheSensors(self):
        """Cache sensor values to reduce CAN traffic."""
        try:
            # Update gyro cache
            self.gyro_yaw = self.gyro.get_yaw().value_as_double
            
            # Less frequent gyro reads for pitch and roll
            if self.periodic_counter % 5 == 0:
                self.gyro_pitch = self.gyro.get_pitch().value_as_double
                self.gyro_roll = self.gyro.get_roll().value_as_double
            
            # Cache module sensor values
            for module in self.modules:
                module.cacheSensors()
            
            # Less frequent current reads
            if self.periodic_counter % 10 == 0:
                for module in self.modules:
                    module.cacheCurrents()
                    
        except Exception as e:
            self.handleError("cacheSensors", e)
    
    def subsystemPeriodic(self):
        """Periodic code for the swerve drive subsystem."""
        try:
            # Update odometry
            self.updateOdometry()
            
            # Update network tables with latest values
            self.updateNetworkTables()
            
            # Update dashboard with values
            self.updateDashboard()
            
        except Exception as e:
            self.handleError("subsystemPeriodic", e)
    
    def updateNetworkTables(self):
        """Update NetworkTables with latest values."""
        try:
            # Update pose values
            self.pose_x_pub.set(self.robot_pose.x)
            self.pose_y_pub.set(self.robot_pose.y)
            self.pose_rot_pub.set(self.robot_pose.rotation().degrees())
            
            # Update gyro values
            self.gyro_yaw_pub.set(self.gyro_yaw)
            self.gyro_pitch_pub.set(self.gyro_pitch)
            self.gyro_roll_pub.set(self.gyro_roll)
            
            # Update speed values
            self.speed_x_pub.set(self.last_chassis_speed.vx)
            self.speed_y_pub.set(self.last_chassis_speed.vy)
            self.speed_rot_pub.set(self.last_chassis_speed.omega)
            
        except Exception as e:
            self.handleError("updateNetworkTables", e)
    
    def updateDashboard(self):
        """Update SmartDashboard with latest values."""
        try:
            # Update robot pose
            wpilib.SmartDashboard.putNumber("SwerveDrive/Pose/X", self.robot_pose.x)
            wpilib.SmartDashboard.putNumber("SwerveDrive/Pose/Y", self.robot_pose.y)
            wpilib.SmartDashboard.putNumber("SwerveDrive/Pose/Rotation", self.robot_pose.rotation().degrees())
            
            # Update gyro values
            wpilib.SmartDashboard.putNumber("SwerveDrive/Gyro/Yaw", self.gyro_yaw)
            
            # Update field-oriented status
            wpilib.SmartDashboard.putBoolean("SwerveDrive/FieldOriented", self.field_oriented)
            
        except Exception as e:
            self.handleError("updateDashboard", e)
    
    def updateOdometry(self):
        """Update odometry with latest sensor values."""
        try:
            # Get rotation from gyro
            rotation = deg2Rot2d(self.gyro_yaw)
            
            # Update odometry with latest module positions
            self.robot_pose = self.odometry.update(
                rotation,
                (
                    self.frontLeftModule.getPosition(),
                    self.frontRightModule.getPosition(),
                    self.backLeftModule.getPosition(),
                    self.backRightModule.getPosition()
                )
            )
            
        except Exception as e:
            self.handleError("updateOdometry", e)
    
    def resetOdometry(self, pose: Pose2d = Pose2d()):
        """Reset odometry to the given pose."""
        try:
            # Reset gyro to match pose heading
            self.gyro.set_yaw(pose.rotation().degrees())
            
            # Update cached gyro value
            self.gyro_yaw = pose.rotation().degrees()
            
            # Recreate odometry with current module positions
            self.odometry = SwerveDrive4Odometry(
                self.kinematics,
                pose.rotation(),
                (
                    self.frontLeftModule.getPosition(),
                    self.frontRightModule.getPosition(),
                    self.backLeftModule.getPosition(),
                    self.backRightModule.getPosition()
                ),
                pose
            )
            
            # Update robot pose
            self.robot_pose = pose
            
            print(f"Odometry reset to: x={pose.x:.2f}, y={pose.y:.2f}, rot={pose.rotation().degrees():.2f}°")
            
        except Exception as e:
            self.handleError("resetOdometry", e)
    
    def resetGyro(self):
        """Reset the gyro to zero heading."""
        try:
            self.gyro.set_yaw(0)
            self.gyro_yaw = 0.0
            print("Gyro reset to 0°")
        except Exception as e:
            self.handleError("resetGyro", e)
    
    def getPose(self) -> Pose2d:
        """Get the current estimated pose of the robot."""
        return self.robot_pose
    
    def getGyroYaw(self) -> float:
        """Get the current yaw angle from the gyro."""
        return self.gyro_yaw
    
    def toggleFieldOriented(self):
        """Toggle between field-oriented and robot-oriented driving."""
        self.field_oriented = not self.field_oriented
        wpilib.SmartDashboard.putBoolean("SwerveDrive/FieldOriented", self.field_oriented)
        print(f"Field-oriented driving: {self.field_oriented}")
    
    def setFieldOriented(self, field_oriented: bool):
        """Set field-oriented driving mode."""
        self.field_oriented = field_oriented
        wpilib.SmartDashboard.putBoolean("SwerveDrive/FieldOriented", self.field_oriented)
    
    def drive(self, x_speed: float, y_speed: float, rot_speed: float, field_relative: bool = None):
        """
        Drive the robot with the given speeds.
        
        Args:
            x_speed (float): Speed in the x direction (forward/backward) in m/s
            y_speed (float): Speed in the y direction (left/right) in m/s
            rot_speed (float): Rotational speed in rad/s
            field_relative (bool, optional): Whether to use field-relative control
                                           If None, uses the current field_oriented setting
        """
        try:
            # Determine if we should use field-relative control
            if field_relative is None:
                field_relative = self.field_oriented
            
            # Create chassis speeds
            if field_relative:
                # For field-relative, we need the gyro angle
                rotation = deg2Rot2d(self.gyro_yaw)
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed, y_speed, rot_speed, rotation
                )
            else:
                # For robot-relative, just use the speeds directly
                chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot_speed)
            
            # Apply acceleration limiting
            chassis_speeds = self.limitAcceleration(self.last_chassis_speed, chassis_speeds)
            
            # Store for next iteration
            self.last_chassis_speed = chassis_speeds
            
            # Convert chassis speeds to module states
            module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            
            # Normalize wheel speeds if any exceeds the maximum speed
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, driveConsts.MAX_SPEED
            )
            
            # Set module states
            self.frontLeftModule.setDesiredState(module_states[0])
            self.frontRightModule.setDesiredState(module_states[1])
            self.backLeftModule.setDesiredState(module_states[2])
            self.backRightModule.setDesiredState(module_states[3])
            
        except Exception as e:
            self.handleError("drive", e)
            self.stopMotors()
    
    def driveWithChassisSpeeds(self, chassis_speeds: ChassisSpeeds):
        """
        Drive the robot with the given ChassisSpeeds.
        
        Args:
            chassis_speeds (ChassisSpeeds): The desired chassis speeds
        """
        try:
            # Apply acceleration limiting
            chassis_speeds = self.limitAcceleration(self.last_chassis_speed, chassis_speeds)
            
            # Store for next iteration
            self.last_chassis_speed = chassis_speeds
            
            # Convert chassis speeds to module states
            module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            
            # Normalize wheel speeds if any exceeds the maximum speed
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, driveConsts.MAX_SPEED
            )
            
            # Set module states
            self.frontLeftModule.setDesiredState(module_states[0])
            self.frontRightModule.setDesiredState(module_states[1])
            self.backLeftModule.setDesiredState(module_states[2])
            self.backRightModule.setDesiredState(module_states[3])
            
        except Exception as e:
            self.handleError("driveWithChassisSpeeds", e)
            self.stopMotors()
    
    def limitAcceleration(self, current_speeds: ChassisSpeeds, target_speeds: ChassisSpeeds, dt: float = 0.02) -> ChassisSpeeds:
        """
        Limit acceleration rates to prevent jerky movements.
        
        Args:
            current_speeds: Current chassis speeds
            target_speeds: Target chassis speeds
            dt: Time difference since last update (default: 20ms)
            
        Returns:
            ChassisSpeeds with limited acceleration
        """
        try:
            # Maximum acceleration rates
            max_linear_accel = driveConsts.MAX_LINEAR_ACCELERATION  # m/s²
            max_angular_accel = driveConsts.MAX_ANGULAR_ACCELERATION  # rad/s²
            
            # Calculate maximum speed changes for this time step
            max_vx_change = max_linear_accel * dt
            max_vy_change = max_linear_accel * dt
            max_omega_change = max_angular_accel * dt
            
            # Limit vx acceleration
            vx_error = target_speeds.vx - current_speeds.vx
            vx_change = max(-max_vx_change, min(vx_error, max_vx_change))
            
            # Limit vy acceleration
            vy_error = target_speeds.vy - current_speeds.vy
            vy_change = max(-max_vy_change, min(vy_error, max_vy_change))
            
            # Limit omega acceleration
            omega_error = target_speeds.omega - current_speeds.omega
            omega_change = max(-max_omega_change, min(omega_error, max_omega_change))
        
            # Create new chassis speeds with limited acceleration
            return ChassisSpeeds(
                current_speeds.vx + vx_change,
                current_speeds.vy + vy_change,
                current_speeds.omega + omega_change
            )
        
        except Exception as e:
            self.handleError("limitAcceleration", e)
            return current_speeds  # Return current speeds in case of error
    
    def stopMotors(self):
        """Stop all motors in the swerve drive."""
        try:
            for module in self.modules:
                module.stop()
            self.last_chassis_speed = ChassisSpeeds(0, 0, 0)
        except Exception as e:
            self.handleError("stopMotors", e)
    
    def getChassisSpeed(self) -> ChassisSpeeds:
        """Get the current chassis speeds."""
        return self.last_chassis_speed
    
    def shouldFlipPath(self) -> bool:
        """
        Determine if paths should be flipped for the red alliance.
        
        Returns:
            bool: True if on red alliance, False otherwise
        """
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def subsystemSimulationPeriodic(self):
        """Periodic simulation code for the swerve drive."""
        if self.is_simulation:
            # In simulation, we would update the simulated hardware
            # This would typically include updating the simulated gyro,
            # odometry, and other sensors based on the commanded speeds
            pass