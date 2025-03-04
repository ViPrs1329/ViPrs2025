import math
import wpilib
import commands2
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Odometry
)
from wpimath.controller import PIDController
from phoenix6.hardware import Pigeon2
from constants.constants import DriveConstants
from subsystems.swerve_module import SwerveModule
from utils.caching import CachingSubsystemBase

class DriveSubsystem(CachingSubsystemBase):
    """
    The drive subsystem, controlling the robot's swerve drive.
    """
    
    class Cache(CachingSubsystemBase.Cache):
        """Cache specific to drive subsystem"""
        def __init__(self):
            super().__init__()
            # Initialize with default values
            self.set_cached("gyro_angle", 0.0)
            self.set_cached("robot_x", 0.0)
            self.set_cached("robot_y", 0.0)
            self.set_cached("robot_heading", 0.0)
            self.set_cached("roll", 0.0)
            self.set_cached("pitch", 0.0)
            
            # Drive mode
            self.set_cached("field_relative", True)
            self.set_cached("speed_mode", DriveConstants.NORMAL_SPEED_MULTIPLIER)
            
            # Setpoints
            self.set_setpoint("x_speed", 0.0)
            self.set_setpoint("y_speed", 0.0)
            self.set_setpoint("rot_speed", 0.0)

    def __init__(self):
        super().__init__()

        # Define the locations of the swerve modules relative to the center of the robot
        self.front_left_location = Translation2d(0.381, 0.381)  # 15 inches forward, 15 inches left
        self.front_right_location = Translation2d(0.381, -0.381)
        self.back_left_location = Translation2d(-0.381, 0.381)
        self.back_right_location = Translation2d(-0.381, -0.381)

        # Create the kinematics object
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location
        )

        # Create the swerve modules
        self.front_left_module = SwerveModule(
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.FRONT_LEFT_TURN_MOTOR,
            DriveConstants.FRONT_LEFT_CANCODER,
            False,  # drive motor inverted
            False,  # turn motor inverted
            DriveConstants.FRONT_LEFT_OFFSET,
            "Front Left"
        )

        self.front_right_module = SwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR,
            DriveConstants.FRONT_RIGHT_CANCODER,
            True,   # drive motor inverted
            False,  # turn motor inverted
            DriveConstants.FRONT_RIGHT_OFFSET,
            "Front Right"
        )

        self.back_left_module = SwerveModule(
            DriveConstants.BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_TURN_MOTOR,
            DriveConstants.BACK_LEFT_CANCODER,
            False,  # drive motor inverted
            False,  # turn motor inverted
            DriveConstants.BACK_LEFT_OFFSET,
            "Back Left"
        )

        self.back_right_module = SwerveModule(
            DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_TURN_MOTOR,
            DriveConstants.BACK_RIGHT_CANCODER,
            True,   # drive motor inverted
            False,  # turn motor inverted
            DriveConstants.BACK_RIGHT_OFFSET,
            "Back Right"
        )

        # Create the Pigeon 2.0 gyro
        self.gyro = Pigeon2(DriveConstants.PIGEON_ID)
        self.gyro.reset()

        # Create the odometry object
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position()
            ),
            Pose2d()
        )

    def cache_sensors(self) -> None:
        """Cache all sensor values."""
        # Cache gyro values
        self.cache.set_cached("gyro_angle", self.gyro.get_yaw().value)
        self.cache.set_cached("roll", self.gyro.get_roll().value)
        self.cache.set_cached("pitch", self.gyro.get_pitch().value)
        
        # Cache odometry values
        pose = self.odometry.getPose()
        self.cache.set_cached("robot_x", pose.X())
        self.cache.set_cached("robot_y", pose.Y())
        self.cache.set_cached("robot_heading", pose.rotation().degrees())
        
        # Update odometry with cached values
        self.odometry.update(
            Rotation2d.fromDegrees(self.cache.get_cached("gyro_angle")),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position()
            )
        )

    def update_hardware(self) -> None:
        """Update hardware with cached setpoints."""
        # Get drive setpoints
        x_speed = self.cache.get_setpoint("x_speed")
        y_speed = self.cache.get_setpoint("y_speed")
        rot = self.cache.get_setpoint("rot_speed")
        
        # Apply speed multiplier
        speed_mode = self.cache.get_cached("speed_mode")
        x_speed *= speed_mode
        y_speed *= speed_mode
        rot *= speed_mode
        
        # Calculate chassis speeds
        if self.cache.get_cached("field_relative"):
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, 
                Rotation2d.fromDegrees(self.cache.get_cached("gyro_angle"))
            )
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)

        # Calculate module states
        swerve_module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        # Normalize wheel speeds if any speed is greater than the max speed
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, 4.0)  # 4 m/s max speed

        # Set each module state
        self.front_left_module.set_desired_state(swerve_module_states[0])
        self.front_right_module.set_desired_state(swerve_module_states[1])
        self.back_left_module.set_desired_state(swerve_module_states[2])
        self.back_right_module.set_desired_state(swerve_module_states[3])

    def periodic_logic(self) -> None:
        """Update SmartDashboard with cached values."""
        wpilib.SmartDashboard.putNumber("Robot X", self.cache.get_cached("robot_x"))
        wpilib.SmartDashboard.putNumber("Robot Y", self.cache.get_cached("robot_y"))
        wpilib.SmartDashboard.putNumber("Robot Heading", self.cache.get_cached("robot_heading"))
        wpilib.SmartDashboard.putNumber("Robot Roll", self.cache.get_cached("roll"))
        wpilib.SmartDashboard.putNumber("Robot Pitch", self.cache.get_cached("pitch"))
        wpilib.SmartDashboard.putBoolean("Field Relative", self.cache.get_cached("field_relative"))

    def drive(self, x_speed: float, y_speed: float, rot: float, period: float = None):
        """
        Drive the robot with given speeds.
        
        :param x_speed: Speed of the robot in the x direction (forward) in m/s
        :param y_speed: Speed of the robot in the y direction (sideways) in m/s
        :param rot: Angular rate of the robot in rad/s
        :param period: Time between calls for velocity calculations
        """
        self.cache.set_setpoint("x_speed", x_speed)
        self.cache.set_setpoint("y_speed", y_speed)
        self.cache.set_setpoint("rot_speed", rot)

    def set_module_states(self, desired_states: list[SwerveModuleState]):
        """
        Set the swerve module states.
        
        :param desired_states: List of desired states for each module
        """
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, 4.0)
        
        self.front_left_module.set_desired_state(desired_states[0])
        self.front_right_module.set_desired_state(desired_states[1])
        self.back_left_module.set_desired_state(desired_states[2])
        self.back_right_module.set_desired_state(desired_states[3])

    def reset_odometry(self, pose: Pose2d):
        """
        Reset the robot's odometry to the given pose.
        
        :param pose: The pose to reset to
        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.cache.get_cached("gyro_angle")),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position()
            ),
            pose
        )

    def zero_heading(self):
        """Reset the gyro heading to zero."""
        self.gyro.reset()
        self.cache_sensors()  # Update cached values
    
    # Alias for zero_heading to maintain consistent naming convention
    def zeroHeading(self):
        """Reset the gyro heading to zero."""
        self.zero_heading()

    def get_heading(self) -> float:
        """
        Get the robot's heading.
        
        :return: The robot's heading in degrees
        """
        return math.remainder(self.cache.get_cached("gyro_angle"), 360.0)

    def get_pose(self) -> Pose2d:
        """
        Get the robot's current pose.
        
        :return: The robot's current pose
        """
        return Pose2d(
            self.cache.get_cached("robot_x"),
            self.cache.get_cached("robot_y"),
            Rotation2d.fromDegrees(self.cache.get_cached("robot_heading"))
        )

    def toggle_field_relative(self):
        """Toggle between field-relative and robot-relative control."""
        self.cache.set_cached("field_relative", not self.cache.get_cached("field_relative"))
    
    # Alias for toggle_field_relative to maintain consistent naming convention
    def toggleFieldRelative(self):
        """Toggle between field-relative and robot-relative control."""
        self.toggle_field_relative()

    def set_speed_mode(self, mode: float):
        """Set the speed mode multiplier."""
        self.cache.set_cached("speed_mode", mode)
    
    def setX(self):
        """Set the modules in an X configuration to prevent movement."""
        states = [
            SwerveModuleState(0, Rotation2d.fromDegrees(45)),   # Front Left
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Front Right
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Back Left
            SwerveModuleState(0, Rotation2d.fromDegrees(45))    # Back Right
        ]
        self.set_module_states(states) 