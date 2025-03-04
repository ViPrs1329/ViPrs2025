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
from utils.telemetry_subsystem_base import TelemetrySubsystemBase
from utils.telemetry import TelemetryManager
from typing import List

class DriveSubsystem(TelemetrySubsystemBase):
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

    def __init__(self, telemetry: TelemetryManager):
        """Initialize drive subsystem with telemetry."""
        super().__init__(telemetry, "Drive")
        
        # Initialize Pigeon 2.0
        self.gyro = Pigeon2(DriveConstants.PIGEON_ID)
        self.gyro.setYaw(0)
        
        # Initialize swerve modules
        self.front_left = SwerveModule(
            "FrontLeft",
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DriveConstants.FRONT_LEFT_ENCODER_ID,
            DriveConstants.FRONT_LEFT_ENCODER_OFFSET
        )
        # ... initialize other modules ...
        
        self.modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right
        ]
        
        # Initialize odometry
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(self.gyro.getYaw().value),
            self._get_module_positions(),
            Pose2d()
        )
        
        # Set up health monitoring
        self._setup_health_monitoring()
        
        # Log initialization
        self.log_event("Initialized")

    def _setup_health_monitoring(self) -> None:
        """Set up health monitoring thresholds."""
        # Monitor gyro connection
        self.register_health_threshold("gyro_connected", min_value=1, max_value=1)
        
        # Monitor voltage levels
        self.register_health_threshold(
            "voltage",
            min_value=10.0,  # Error if below 10V
            max_value=13.5,  # Error if above 13.5V
            warning_min=11.0,  # Warning if below 11V
            warning_max=13.0   # Warning if above 13V
        )
        
        # Monitor temperature
        self.register_health_threshold(
            "temperature",
            max_value=80.0,    # Error if above 80°C
            warning_max=70.0   # Warning if above 70°C
        )
    
    def cache_sensors(self) -> None:
        """Cache all sensor readings."""
        # Cache gyro readings
        self.set_cached("gyro_yaw", self.gyro.getYaw().value)
        self.set_cached("gyro_pitch", self.gyro.getPitch().value)
        self.set_cached("gyro_roll", self.gyro.getRoll().value)
        self.set_cached("gyro_connected", self.gyro.isConnected())
        
        # Cache module states
        module_states = {}
        for module in self.modules:
            state = module.get_state()
            module_states[module.name] = {
                "drive_position": state.speed,
                "turn_position": state.angle.degrees(),
                "drive_velocity": module.get_velocity(),
                "turn_velocity": module.get_turn_velocity(),
                "drive_current": module.get_drive_current(),
                "turn_current": module.get_turn_current(),
                "temperature": module.get_temperature()
            }
        self.set_cached("module_states", module_states)
        
        # Cache odometry
        pose = self.odometry.getPose()
        self.set_cached("pose_x", pose.X())
        self.set_cached("pose_y", pose.Y())
        self.set_cached("pose_rotation", pose.rotation().degrees())
    
    def periodic_logic(self) -> None:
        """Update odometry and log states."""
        # Update odometry
        self.odometry.update(
            Rotation2d.fromDegrees(self.get_cached("gyro_yaw")),
            self._get_module_positions()
        )
        
        # Log swerve states for AdvantageScope visualization
        self.telemetry.log_swerve_state(self.get_cached("module_states"))
        
        # Log robot pose for field visualization
        self.telemetry.log_odometry(self.get_pose())
    
    def update_hardware(self) -> None:
        """Update hardware with cached values."""
        # Hardware updates are handled by individual set methods
        pass
    
    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """Drive the robot."""
        if field_relative:
            # Use cached gyro reading for field-relative calculations
            current_rotation = Rotation2d.fromDegrees(self.get_cached("gyro_yaw"))
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, current_rotation
            )
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)
        
        # Convert chassis speeds to module states
        swerve_module_states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            chassis_speeds
        )
        
        # Normalize wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, DriveConstants.MAX_SPEED
        )
        
        # Set module states
        for i, module in enumerate(self.modules):
            module.set_desired_state(swerve_module_states[i])
        
        # Log drive command
        self.log_event("Drive Command", 
                      f"x:{x_speed:.2f} y:{y_speed:.2f} rot:{rot:.2f} field:{field_relative}")
    
    def get_pose(self) -> Pose2d:
        """Get the current robot pose."""
        return Pose2d(
            self.get_cached("pose_x"),
            self.get_cached("pose_y"),
            Rotation2d.fromDegrees(self.get_cached("pose_rotation"))
        )
    
    def reset_odometry(self, pose: Pose2d) -> None:
        """Reset odometry to the given pose."""
        self.gyro.setYaw(pose.rotation().degrees())
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getYaw().value),
            self._get_module_positions(),
            pose
        )
        self.log_event("Odometry Reset", f"x:{pose.X():.2f} y:{pose.Y():.2f} rot:{pose.rotation().degrees():.2f}")
    
    def _get_module_positions(self) -> List[SwerveModulePosition]:
        """Get the positions of all swerve modules."""
        return [module.get_position() for module in self.modules]

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
        return math.remainder(self.get_cached("gyro_yaw"), 360.0)

    def toggle_field_relative(self):
        """Toggle between field-relative and robot-relative control."""
        self.set_cached("field_relative", not self.get_cached("field_relative"))
    
    # Alias for toggle_field_relative to maintain consistent naming convention
    def toggleFieldRelative(self):
        """Toggle between field-relative and robot-relative control."""
        self.toggle_field_relative()

    def set_speed_mode(self, mode: float):
        """Set the speed mode multiplier."""
        self.set_cached("speed_mode", mode)
    
    def setX(self):
        """Set the modules in an X configuration to prevent movement."""
        states = [
            SwerveModuleState(0, Rotation2d.fromDegrees(45)),   # Front Left
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Front Right
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  # Back Left
            SwerveModuleState(0, Rotation2d.fromDegrees(45))    # Back Right
        ]
        self.set_module_states(states) 