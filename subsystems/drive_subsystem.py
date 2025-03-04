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
from constants.constants import DriveConstants
from subsystems.swerve_module import SwerveModule

class DriveSubsystem(commands2.SubsystemBase):
    """
    The drive subsystem, controlling the robot's swerve drive.
    """

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

        # Create the gyro
        self.gyro = wpilib.ADIS16470_IMU()
        self.gyro.calibrate()
        self.gyro.reset()

        # Create the odometry object
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position()
            ),
            Pose2d()
        )

        # Initialize drive mode
        self.field_relative = True
        self.speed_mode = DriveConstants.NORMAL_SPEED_MULTIPLIER

    def periodic(self):
        """Update odometry and dashboard values."""
        # Update odometry
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position()
            )
        )

        # Update SmartDashboard
        pose = self.odometry.getPose()
        wpilib.SmartDashboard.putNumber("Robot X", pose.X())
        wpilib.SmartDashboard.putNumber("Robot Y", pose.Y())
        wpilib.SmartDashboard.putNumber("Robot Heading", pose.rotation().degrees())
        wpilib.SmartDashboard.putBoolean("Field Relative", self.field_relative)

    def drive(self, x_speed: float, y_speed: float, rot: float, period: float = None):
        """
        Drive the robot with given speeds.
        
        :param x_speed: Speed of the robot in the x direction (forward) in m/s
        :param y_speed: Speed of the robot in the y direction (sideways) in m/s
        :param rot: Angular rate of the robot in rad/s
        :param period: Time between calls for velocity calculations
        """
        # Apply speed multiplier
        x_speed *= self.speed_mode
        y_speed *= self.speed_mode
        rot *= self.speed_mode

        if self.field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, Rotation2d.fromDegrees(self.gyro.getAngle())
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
            Rotation2d.fromDegrees(self.gyro.getAngle()),
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
    
    # Alias for zero_heading to maintain consistent naming convention
    def zeroHeading(self):
        """Reset the gyro heading to zero."""
        self.zero_heading()

    def get_heading(self) -> float:
        """
        Get the robot's heading.
        
        :return: The robot's heading in degrees
        """
        return math.remainder(self.gyro.getAngle(), 360.0)

    def get_pose(self) -> Pose2d:
        """
        Get the robot's current pose.
        
        :return: The robot's current pose
        """
        return self.odometry.getPose()

    def toggle_field_relative(self):
        """Toggle between field-relative and robot-relative control."""
        self.field_relative = not self.field_relative
        wpilib.SmartDashboard.putBoolean("Field Relative", self.field_relative)
    
    # Alias for toggle_field_relative to maintain consistent naming convention
    def toggleFieldRelative(self):
        """Toggle between field-relative and robot-relative control."""
        self.toggle_field_relative()

    def set_speed_mode(self, mode: float):
        """Set the speed mode multiplier."""
        self.speed_mode = mode
    
    def setX(self):
        """Set the modules in an X configuration to prevent movement."""
        states = [
            SwerveModuleState(0, Rotation2d.fromDegrees(45)),   # Front Left
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Front Right
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Back Left
            SwerveModuleState(0, Rotation2d.fromDegrees(45))    # Back Right
        ]
        self.set_module_states(states) 