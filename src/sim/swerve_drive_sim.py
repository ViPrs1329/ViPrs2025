# swerve_drive_sim.py
"""Simulation implementation for the complete swerve drive"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from .swerve_module_sim import SwerveModuleSim
from .swerve_sim_config import *
from constants import driveConsts

class SwerveDriveSim:
    """
    Simulation class for the complete swerve drive system.
    Manages four swerve modules and simulates the robot's motion.
    """
    
    def __init__(self):
        """Initialize the swerve drive simulation."""
        # Calculate module positions
        half_width = ROBOT_WIDTH / 2.0
        half_length = ROBOT_LENGTH / 2.0
        
        # Create swerve modules
        self.front_left = SwerveModuleSim(
            "Front Left",
            wpimath.geometry.Translation2d(half_length, half_width)
        )
        
        self.front_right = SwerveModuleSim(
            "Front Right",
            wpimath.geometry.Translation2d(half_length, -half_width)
        )
        
        self.back_left = SwerveModuleSim(
            "Back Left",
            wpimath.geometry.Translation2d(-half_length, half_width)
        )
        
        self.back_right = SwerveModuleSim(
            "Back Right",
            wpimath.geometry.Translation2d(-half_length, -half_width)
        )
        
        # Store modules in a list for easy iteration
        self.modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right
        ]
        
        # Create kinematics object
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left.location,
            self.front_right.location,
            self.back_left.location,
            self.back_right.location
        )
        
        # Robot pose tracking
        self.pose = wpimath.geometry.Pose2d()
        self.gyro_angle = 0.0
        
        # Create odometry
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d(0),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ),
            wpimath.geometry.Pose2d()
        )
        
        # Last update time
        self.last_time = wpilib.Timer.getFPGATimestamp()
    
    def update(self):
        """Update the simulation state."""
        # Update each module
        for module in self.modules:
            module.update()
        
        # Calculate time difference
        current_time = wpilib.Timer.getFPGATimestamp()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get current chassis speeds from module states
        chassis_speeds = self.kinematics.toChassisSpeeds(
            self.front_left.get_state(),
            self.front_right.get_state(),
            self.back_left.get_state(),
            self.back_right.get_state()
        )
        
        # Update robot pose
        self.gyro_angle += chassis_speeds.omega * dt
        
        # Normalize gyro angle to [-pi, pi]
        self.gyro_angle = math.atan2(
            math.sin(self.gyro_angle),
            math.cos(self.gyro_angle)
        )
        
        # Update odometry
        self.pose = self.odometry.update(
            wpimath.geometry.Rotation2d(self.gyro_angle),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            )
        )
    
    def set_module_states(self, states: list[wpimath.kinematics.SwerveModuleState]):
        """
        Set the desired states for all modules.
        
        Args:
            states: List of desired states for [FL, FR, BL, BR] modules
        """
        if len(states) != 4:
            raise ValueError("Must provide exactly 4 module states")
        
        # Optimize states to minimize rotation
        optimized_states = []
        for module, state in zip(self.modules, states):
            current_rotation = wpimath.geometry.Rotation2d(module.rotation_position)
            optimized_state = wpimath.kinematics.SwerveModuleState.optimize(state, current_rotation)
            optimized_states.append(optimized_state)
        
        # Calculate and set voltages for each module
        for module, state in zip(self.modules, optimized_states):
            # Calculate drive voltage
            drive_voltage = state.speed / driveConsts.MAX_SPEED * DRIVE_MAX_VOLTAGE
            
            # Calculate rotation voltage
            current_angle = module.rotation_position
            target_angle = state.angle.radians()
            angle_error = math.atan2(
                math.sin(target_angle - current_angle),
                math.cos(target_angle - current_angle)
            )
            rotation_voltage = angle_error * 5.0  # Simple P controller
            
            # Set module voltages
            module.set_drive_voltage(drive_voltage)
            module.set_rotation_voltage(rotation_voltage)
    
    def get_pose(self) -> wpimath.geometry.Pose2d:
        """Get the current robot pose."""
        return self.pose
    
    def get_gyro_angle(self) -> float:
        """Get the current gyro angle in radians."""
        return self.gyro_angle
    
    def reset_pose(self, pose: wpimath.geometry.Pose2d):
        """Reset the robot's pose."""
        self.pose = pose
        self.gyro_angle = pose.rotation().radians()
        
        # Reset odometry
        self.odometry.resetPosition(
            pose.rotation(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ),
            pose
        )
    
    def reset_encoders(self):
        """Reset all module encoders."""
        for module in self.modules:
            module.reset_encoders()
            
    def get_module_states(self) -> list[wpimath.kinematics.SwerveModuleState]:
        """Get the current states of all modules."""
        return [module.get_state() for module in self.modules]
    
    def get_module_positions(self) -> list[wpimath.kinematics.SwerveModulePosition]:
        """Get the current positions of all modules."""
        return [module.get_position() for module in self.modules] 