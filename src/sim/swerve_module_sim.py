# swerve_module_sim.py
"""Simulation implementation for a swerve module"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from wpimath.system.plant import DCMotor
from wpimath.units import meters_per_second
from .swerve_sim_config import *

class SwerveModuleSim:
    """
    Simulation class for a single swerve module.
    This simulates both the drive and rotation motors.
    """
    
    def __init__(self, module):
        """
        Initialize the swerve module simulation.
        
        Args:
            module: The actual swerve module to simulate
        """
        self.module = module
        self.name = module.name
        
        # Initialize state variables
        self.drive_position = 0.0  # meters
        self.drive_velocity = 0.0  # meters per second
        self.rotation_position = 0.0  # radians
        self.rotation_velocity = 0.0  # radians per second
        
        # Current draw simulation
        self.drive_current = 0.0
        self.rotation_current = 0.0
        
        # Control inputs
        self.drive_voltage = 0.0
        self.rotation_voltage = 0.0
        
        # Last update time
        self.last_time = wpilib.Timer.getFPGATimestamp()
        
        # Initialize the actual module's encoders
        if hasattr(self.module, 'drive_encoder') and self.module.drive_encoder is not None:
            self.module.drive_encoder.setPosition(self.drive_position)
            self.module.drive_encoder.setVelocity(self.drive_velocity)
            
        if hasattr(self.module, 'rotation_encoder') and self.module.rotation_encoder is not None:
            self.module.rotation_encoder.setPosition(self.rotation_position)
            self.module.rotation_encoder.setVelocity(self.rotation_velocity)
        
        # Initialize the actual module's state
        initial_state = wpimath.kinematics.SwerveModuleState(
            self.drive_velocity,
            wpimath.geometry.Rotation2d(self.rotation_position)
        )
        if hasattr(self.module, 'setDesiredState') and self.module is not None:
            try:
                self.module.setDesiredState(initial_state)
            except Exception as e:
                print(f"Warning: Could not set initial state for {self.name}: {e}")
    
    def update(self, dt):
        """Update the simulation state."""
        # Simulate drive motor
        drive_acceleration = self._simulate_drive_motor(
            self.drive_voltage,
            self.drive_velocity
        )
        
        # Update drive state
        self.drive_velocity += drive_acceleration * dt
        self.drive_position += self.drive_velocity * dt
        
        # Simulate rotation motor
        rotation_acceleration = self._simulate_rotation_motor(
            self.rotation_voltage,
            self.rotation_velocity
        )
        
        # Update rotation state
        self.rotation_velocity += rotation_acceleration * dt
        self.rotation_position += self.rotation_velocity * dt
        
        # Normalize rotation position to [-pi, pi]
        self.rotation_position = math.atan2(
            math.sin(self.rotation_position),
            math.cos(self.rotation_position)
        )
        
        # Calculate motor currents (simplified model)
        self.drive_current = abs(self.drive_voltage / 12.0) * 40.0  # Approximate current draw
        self.rotation_current = abs(self.rotation_voltage / 12.0) * 40.0
        
        # Update the actual module's state
        if hasattr(self.module, 'drive_encoder') and self.module.drive_encoder is not None:
            self.module.drive_encoder.setPosition(self.drive_position)
            self.module.drive_encoder.setVelocity(self.drive_velocity)
        if hasattr(self.module, 'rotation_encoder') and self.module.rotation_encoder is not None:
            self.module.rotation_encoder.setPosition(self.rotation_position)
            self.module.rotation_encoder.setVelocity(self.rotation_velocity)
    
    def set_state(self, state: wpimath.kinematics.SwerveModuleState):
        """Set the desired state of the module."""
        if state is None:
            print(f"Warning: Received None state for {self.name}, using default state")
            # Create a default state with zero speed and current angle
            state = wpimath.kinematics.SwerveModuleState(
                0.0,
                wpimath.geometry.Rotation2d(self.rotation_position)
            )
            
        # Check if state attributes are None
        if state.speed is None:
            print(f"Warning: Received None speed for {self.name}, using zero speed")
            state = wpimath.kinematics.SwerveModuleState(
                0.0,
                state.angle
            )
            
        if state.angle is None:
            print(f"Warning: Received None angle for {self.name}, using current angle")
            state = wpimath.kinematics.SwerveModuleState(
                state.speed,
                wpimath.geometry.Rotation2d(self.rotation_position)
            )
            
        # Get current rotation
        current_rotation = wpimath.geometry.Rotation2d(self.rotation_position)
        
        # Optimize the state to minimize rotation
        optimized_state = wpimath.kinematics.SwerveModuleState.optimize(state, current_rotation)
        
        # Calculate drive voltage
        drive_voltage = optimized_state.speed / driveConsts.MAX_SPEED * DRIVE_MAX_VOLTAGE
        
        # Calculate rotation voltage
        current_angle = self.rotation_position
        target_angle = optimized_state.angle.radians()
        angle_error = math.atan2(
            math.sin(target_angle - current_angle),
            math.cos(target_angle - current_angle)
        )
        rotation_voltage = angle_error * 5.0  # Simple P controller
        
        # Set voltages
        self.set_drive_voltage(drive_voltage)
        self.set_rotation_voltage(rotation_voltage)
        
        # Update the actual module's state
        if hasattr(self.module, 'setDesiredState'):
            try:
                self.module.setDesiredState(optimized_state)
            except Exception as e:
                print(f"Warning: Could not set desired state for {self.name}: {e}")
    
    def _simulate_drive_motor(self, voltage: float, velocity: float) -> float:
        """
        Simulate the drive motor using a simplified model.
        
        Args:
            voltage: Applied voltage
            velocity: Current velocity
            
        Returns:
            float: Acceleration in m/s^2
        """
        # Apply voltage limits
        voltage = max(-DRIVE_MAX_VOLTAGE, min(voltage, DRIVE_MAX_VOLTAGE))
        
        # Calculate friction compensation
        friction = math.copysign(DRIVE_KS, voltage) if abs(voltage) > 0 else 0
        friction += DRIVE_KV * velocity
        
        # Calculate effective voltage
        effective_voltage = voltage - friction
        
        # Calculate acceleration using simplified motor model
        acceleration = (effective_voltage * DRIVE_MOTOR_KV - velocity) * DRIVE_MOTOR_KA
        
        return acceleration
    
    def _simulate_rotation_motor(self, voltage: float, velocity: float) -> float:
        """
        Simulate the rotation motor using a simplified model.
        
        Args:
            voltage: Applied voltage
            velocity: Current angular velocity
            
        Returns:
            float: Angular acceleration in rad/s^2
        """
        # Apply voltage limits
        voltage = max(-ROTATION_MAX_VOLTAGE, min(voltage, ROTATION_MAX_VOLTAGE))
        
        # Calculate friction compensation
        friction = math.copysign(ROTATION_KS, voltage) if abs(voltage) > 0 else 0
        friction += ROTATION_KV * velocity
        
        # Calculate effective voltage
        effective_voltage = voltage - friction
        
        # Calculate acceleration using simplified motor model
        acceleration = (effective_voltage * ROTATION_MOTOR_KV - velocity) * ROTATION_MOTOR_KA
        
        return acceleration
    
    def set_drive_voltage(self, voltage: float):
        """Set the drive motor voltage."""
        self.drive_voltage = voltage
        self.module.drive_motor.setVoltage(voltage)
    
    def set_rotation_voltage(self, voltage: float):
        """Set the rotation motor voltage."""
        self.rotation_voltage = voltage
        self.module.rotation_motor.setVoltage(voltage)
    
    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        """Get the current state of the module."""
        return wpimath.kinematics.SwerveModuleState(
            self.drive_velocity,
            wpimath.geometry.Rotation2d(self.rotation_position)
        )
    
    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        """Get the current position of the module."""
        return wpimath.kinematics.SwerveModulePosition(
            self.drive_position,
            wpimath.geometry.Rotation2d(self.rotation_position)
        )
    
    def reset_encoders(self):
        """Reset the encoder positions to zero."""
        self.drive_position = 0.0
        self.rotation_position = 0.0
        if hasattr(self.module, 'drive_encoder') and self.module.drive_encoder is not None:
            self.module.drive_encoder.setPosition(0)
        if hasattr(self.module, 'rotation_encoder') and self.module.rotation_encoder is not None:
            self.module.rotation_encoder.setPosition(0)
    
    def get_drive_current(self) -> float:
        """Get the drive motor current draw."""
        return self.drive_current
    
    def get_rotation_current(self) -> float:
        """Get the rotation motor current draw."""
        return self.rotation_current 