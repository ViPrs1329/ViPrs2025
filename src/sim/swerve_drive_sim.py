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
    
    def __init__(self, swerve_drive):
        """Initialize the swerve drive simulation."""
        self.swerve_drive = swerve_drive
        self.swerve_modules = []
        self.kinematics = None
        self.odometry = None
        self.gyro = None
        self.field = None
        self.robot_pose = wpimath.geometry.Pose2d()
        self.last_chassis_speed = wpimath.kinematics.ChassisSpeeds()
        
        # Initialize simulation components
        self._init_simulation()
        
    def _init_simulation(self):
        """Initialize simulation components"""
        # Create simulated swerve modules
        for module in self.swerve_drive.modules:
            sim_module = SwerveModuleSim(module)
            self.swerve_modules.append(sim_module)
            
        # Create kinematics using simulated module positions
        module_positions = [module.get_position() for module in self.swerve_modules]
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(*module_positions)
        
        # Create odometry using simulated module positions
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d(0),
            (
                module_positions[0].x,
                module_positions[1].x,
                module_positions[2].x,
                module_positions[3].x
            ),
            self.robot_pose
        )
        
        # Create simulated gyro
        self.gyro = wpilib.ADXRS450_Gyro()
        
        # Create field
        self.field = wpilib.Field2d()
        
    def update(self, dt):
        """Update simulation state"""
        # Update module states
        for module in self.swerve_modules:
            module.update(dt)
            
        # Update odometry
        self.odometry.update(
            self.swerve_drive.getGyroYaw(),
            *[module.get_state() for module in self.swerve_modules]
        )
        
        # Update robot pose
        self.robot_pose = self.odometry.getPose()
        
        # Update field
        self.field.setRobotPose(self.robot_pose)
        
    def setModuleStates(self, states):
        """Set states for all swerve modules"""
        if states is None:
            print("Warning: Received None states in setModuleStates, using default states")
            # Create default states for all modules
            states = []
            for module in self.swerve_modules:
                current_state = module.get_state()
                if current_state is not None:
                    states.append(wpimath.kinematics.SwerveModuleState(0, current_state.angle))
                else:
                    states.append(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d()))
            
        # Ensure we have enough states for all modules
        if len(states) < len(self.swerve_modules):
            print(f"Warning: Received {len(states)} states for {len(self.swerve_modules)} modules, using default states")
            # Pad with default states
            while len(states) < len(self.swerve_modules):
                states.append(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d()))
            
        for module, state in zip(self.swerve_modules, states):
            try:
                if state is None:
                    print(f"Warning: Received None state for module {module.name}, using default state")
                    # Use a default state with zero speed and current angle
                    current_state = module.get_state()
                    if current_state is not None:
                        module.set_state(wpimath.kinematics.SwerveModuleState(0, current_state.angle))
                    else:
                        module.set_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d()))
                else:
                    # Check if state attributes are None
                    if state.speed is None:
                        print(f"Warning: Received None speed for module {module.name}, using zero speed")
                        state = wpimath.kinematics.SwerveModuleState(0, state.angle)
                    if state.angle is None:
                        print(f"Warning: Received None angle for module {module.name}, using current angle")
                        current_state = module.get_state()
                        if current_state is not None:
                            state = wpimath.kinematics.SwerveModuleState(state.speed, current_state.angle)
                        else:
                            state = wpimath.kinematics.SwerveModuleState(state.speed, wpimath.geometry.Rotation2d())
                    module.set_state(state)
            except Exception as e:
                print(f"Error setting state for module {module.name}: {e}")
                # Try to set a safe default state
                try:
                    module.set_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d()))
                except Exception as e2:
                    print(f"Error setting default state for module {module.name}: {e2}")
                
    def getPose(self):
        """Get current robot pose"""
        return self.robot_pose
        
    def getGyroAngle(self):
        """Get current gyro angle"""
        return self.gyro.getAngle()
        
    def getChassisSpeeds(self):
        """Get current chassis speeds"""
        # Get module states
        module_states = []
        for module in self.swerve_modules:
            state = module.get_state()
            if state is not None:
                module_states.append(state)
            else:
                print(f"Warning: Module {module.name} returned None state")
                # Use a safe default state
                module_states.append(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d()))
        
        # Convert to chassis speeds using kinematics
        chassis_speeds = self.kinematics.toChassisSpeeds(module_states)
        
        # Store for next drive command
        self.last_chassis_speed = chassis_speeds
        
        return chassis_speeds
        
    def drive(self, chassis_speeds: wpimath.kinematics.ChassisSpeeds):
        """
        Drive the robot using chassis speeds.
        
        Args:
            chassis_speeds: Desired chassis speeds
        """
        if chassis_speeds is None:
            print("Warning: Received None chassis speeds in drive, using zero speeds")
            chassis_speeds = wpimath.kinematics.ChassisSpeeds()
            
        try:
            # Convert chassis speeds to module states
            module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            
            # Set module states
            self.setModuleStates(module_states)
        except Exception as e:
            print(f"Error in drive method: {e}")
            # Try to stop the robot
            try:
                self.stop()
            except Exception as e2:
                print(f"Error stopping robot: {e2}")
        
    def stop(self):
        """Stop all motors"""
        self.drive(wpimath.kinematics.ChassisSpeeds())
    
    def reset_pose(self, pose: wpimath.geometry.Pose2d):
        """Reset the robot's pose."""
        self.robot_pose = pose
        self.gyro.reset()
        
        # Reset odometry
        self.odometry.resetPosition(
            pose.rotation(),
            (
                self.swerve_modules[0].get_position().x,
                self.swerve_modules[1].get_position().x,
                self.swerve_modules[2].get_position().x,
                self.swerve_modules[3].get_position().x
            ),
            pose
        )
    
    def reset_encoders(self):
        """Reset all module encoders."""
        for module in self.swerve_modules:
            module.reset_encoders()
            
    def get_module_states(self) -> list[wpimath.kinematics.SwerveModuleState]:
        """Get the current states of all modules."""
        return [module.get_state() for module in self.swerve_modules]
    
    def get_module_positions(self) -> list[wpimath.kinematics.SwerveModulePosition]:
        """Get the current positions of all modules."""
        return [module.get_position() for module in self.swerve_modules] 