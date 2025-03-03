# src/subsystems/SwerveDriveSubsystem.py
import rev
import math
import commands2
import wpilib
import ntcore

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath import controller
from wpilib import DriverStation, RobotBase

from constants import CANIDs, driveConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory
from subsystems.BaseSubsystem import BaseSubsystem
from phoenix6.hardware import CANcoder, Pigeon2

if RobotBase.isSimulation():
    from sim.swerve_drive_sim import SwerveDriveSim


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
        self.is_simulation = RobotBase.isSimulation()
        
        # Create motor configurations
        drive_config = SparkMaxFactory.Configuration()
        if self.is_simulation:
            drive_config.idle_mode = rev.SparkMax.IdleMode.kBrake
        else:
            drive_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        drive_config.current_limit = driveConsts.currentLimit
        drive_config.voltage_comp_enabled = True
        drive_config.voltage_comp_saturation = 12.0
        drive_config.inverted = drive_inverted
        
        rotation_config = SparkMaxFactory.Configuration()
        if self.is_simulation:
            rotation_config.idle_mode = rev.SparkMax.IdleMode.kBrake
        else:
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
            # Create simulation object if in simulation mode
            self.is_simulation = RobotBase.isSimulation()
            if self.is_simulation:
                self.sim = SwerveDriveSim()
                # Initialize simulation-specific attributes
                self.gyro = self.sim
                self.pose_x_pub = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive/pose_x").publish()
                self.pose_y_pub = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive/pose_y").publish()
                self.pose_rot_pub = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive/pose_rot").publish()
                self.gyro_yaw_pub = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive/gyro_yaw").publish()
                self.robot_pose = Pose2d()
                self.last_chassis_speed = ChassisSpeeds(0, 0, 0)
                self.field_oriented = True
            else:
                # Initialize hardware-specific attributes
                self.gyro = Pigeon2(CANIDs.PigeonID)
                self.pose_x_pub = None
                self.pose_y_pub = None
                self.pose_rot_pub = None
                self.gyro_yaw_pub = None
                self.robot_pose = Pose2d()
                self.last_chassis_speed = ChassisSpeeds(0, 0, 0)
                self.field_oriented = True
            
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
            
        except Exception as e:
            print(f"Error initializing SwerveDrive: {e}")
            raise

    def resetGyro(self):
        """Reset the gyro to zero heading."""
        try:
            if self.is_simulation:
                self.sim.reset_pose(Pose2d())
            else:
                self.gyro.set_yaw(0)
        except Exception as e:
            print(f"Error resetting gyro: {e}")

    def getGyroYaw(self) -> float:
        """Get the current yaw angle from the gyro in radians."""
        try:
            if self.is_simulation:
                return self.sim.get_gyro_angle()
            else:
                return math.radians(self.gyro.get_yaw().value)
        except Exception as e:
            print(f"Error getting gyro yaw: {e}")
            return 0.0

    def getGyroPitch(self) -> float:
        """Get the current pitch angle from the gyro in radians."""
        try:
            if self.is_simulation:
                return 0.0  # Simulation doesn't track pitch
            else:
                return math.radians(self.gyro.get_pitch().value)
        except Exception as e:
            print(f"Error getting gyro pitch: {e}")
            return 0.0

    def cacheSensors(self):
        """Cache sensor values to reduce CAN traffic."""
        try:
            if self.is_simulation:
                self.sim.update()
            else:
                # Cache module sensors
                self.frontLeftModule.cacheSensors()
                self.frontRightModule.cacheSensors()
                self.backLeftModule.cacheSensors()
                self.backRightModule.cacheSensors()
                
                # Cache module currents
                self.frontLeftModule.cacheCurrents()
                self.frontRightModule.cacheCurrents()
                self.backLeftModule.cacheCurrents()
                self.backRightModule.cacheCurrents()
        except Exception as e:
            print(f"Error caching SwerveDrive sensors: {e}")

    def updateOdometry(self):
        """Update the robot's odometry."""
        try:
            if self.is_simulation:
                self.robot_pose = self.sim.get_pose()
            else:
                self.robot_pose = self.odometry.update(
                    Rotation2d(self.getGyroYaw()),
                    (
                        self.frontLeftModule.getPosition(),
                        self.frontRightModule.getPosition(),
                        self.backLeftModule.getPosition(),
                        self.backRightModule.getPosition()
                    )
                )
        except Exception as e:
            print(f"Error updating SwerveDrive odometry: {e}")

    def updateNetworkTables(self):
        """Update NetworkTables with current values."""
        try:
            if self.is_simulation:
                self.pose_x_pub.set(self.robot_pose.X())
                self.pose_y_pub.set(self.robot_pose.Y())
                self.pose_rot_pub.set(self.robot_pose.rotation().radians())
                self.gyro_yaw_pub.set(self.getGyroYaw())
        except Exception as e:
            print(f"Error updating SwerveDrive NetworkTables: {e}")

    def updateDashboard(self):
        """Update the dashboard with current values."""
        try:
            if self.is_simulation:
                self.updateNetworkTables()
            else:
                # Update NetworkTables if they exist
                if self.pose_x_pub is not None:
                    self.updateNetworkTables()
        except Exception as e:
            print(f"Error updating SwerveDrive dashboard: {e}")

    def drive(self, xSpeed: float, ySpeed: float, rotSpeed: float):
        """Drive the robot using field-oriented or robot-oriented control."""
        try:
            # Create chassis speeds
            if self.field_oriented:
                # Convert field-oriented speeds to robot-oriented speeds
                gyro_angle = self.getGyroYaw()
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, Rotation2d(gyro_angle)
                )
            else:
                chassis_speeds = ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
            
            # Store chassis speeds for odometry
            self.last_chassis_speed = chassis_speeds
            
            # Convert chassis speeds to module states
            module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            
            # Set module states
            if self.is_simulation:
                self.sim.set_module_states(module_states)
            else:
                self.frontLeftModule.setDesiredState(module_states[0])
                self.frontRightModule.setDesiredState(module_states[1])
                self.backLeftModule.setDesiredState(module_states[2])
                self.backRightModule.setDesiredState(module_states[3])
        except Exception as e:
            print(f"Error in SwerveDrive.drive: {e}")
            self.stop()

    def stop(self):
        """Stop all motors."""
        try:
            if self.is_simulation:
                self.sim.set_module_states([
                    SwerveModuleState(0, Rotation2d()),
                    SwerveModuleState(0, Rotation2d()),
                    SwerveModuleState(0, Rotation2d()),
                    SwerveModuleState(0, Rotation2d())
                ])
            else:
                self.frontLeftModule.stop()
                self.frontRightModule.stop()
                self.backLeftModule.stop()
                self.backRightModule.stop()
        except Exception as e:
            print(f"Error stopping SwerveDrive: {e}")

    def periodic(self):
        """Called periodically to update subsystem state."""
        try:
            self.cacheSensors()
            self.updateOdometry()
            self.updateDashboard()
        except Exception as e:
            print(f"Error in SwerveDrive.periodic: {e}")
            
    def getChassisSpeed(self) -> ChassisSpeeds:
        """Get the current chassis speeds."""
        try:
            if self.is_simulation:
                return self.sim.get_chassis_speeds()
            else:
                # Get current module states
                states = [
                    SwerveModuleState(
                        self.frontLeftModule.drive_encoder.getVelocity(),
                        Rotation2d(ticks2rad(self.frontLeftModule.rotation_position))
                    ),
                    SwerveModuleState(
                        self.frontRightModule.drive_encoder.getVelocity(),
                        Rotation2d(ticks2rad(self.frontRightModule.rotation_position))
                    ),
                    SwerveModuleState(
                        self.backLeftModule.drive_encoder.getVelocity(),
                        Rotation2d(ticks2rad(self.backLeftModule.rotation_position))
                    ),
                    SwerveModuleState(
                        self.backRightModule.drive_encoder.getVelocity(),
                        Rotation2d(ticks2rad(self.backRightModule.rotation_position))
                    )
                ]
                # Convert module states to chassis speeds
                return self.kinematics.toChassisSpeeds(states)
        except Exception as e:
            print(f"Error getting chassis speeds: {e}")
            return ChassisSpeeds(0, 0, 0)