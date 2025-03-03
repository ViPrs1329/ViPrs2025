# swerve_sim_config.py
"""Configuration parameters for swerve drive simulation"""

import math
from wpimath.units import meters_per_second

# Motor simulation parameters
DRIVE_KS = 0.1  # Static friction compensation (V)
DRIVE_KV = 2.0  # Velocity-based friction compensation (V*s/m)
DRIVE_KA = 0.2  # Acceleration compensation (V*s²/m)
DRIVE_MAX_VOLTAGE = 12.0  # Maximum voltage for motors (V)

# Rotation motor simulation parameters
ROTATION_KS = 0.1  # Static friction (V)
ROTATION_KV = 0.5  # Velocity friction (V*s/rad)
ROTATION_KA = 0.1  # Acceleration compensation (V*s²/rad)
ROTATION_MAX_VOLTAGE = 12.0  # Maximum voltage (V)

# Motor constants (NEO characteristics)
DRIVE_MOTOR_KV = 473  # RPM per volt
DRIVE_MOTOR_KA = 0.8  # RPM per volt per second
ROTATION_MOTOR_KV = 473  # RPM per volt
ROTATION_MOTOR_KA = 0.8  # RPM per volt per second

# Simulation update rate
SIM_SAMPLE_RATE_MS = 20  # 50Hz update rate

# Visualization parameters
ROBOT_WIDTH = 0.6  # meters
ROBOT_LENGTH = 0.6  # meters
MODULE_RADIUS = 0.05  # meters for visualization
WHEEL_WIDTH = 0.04  # meters for visualization

# Testing modes
TEST_MODES = {
    'FULL_DRIVE': 'Full Swerve Drive Control',
    'SINGLE_MODULE_DRIVE': 'Single Module Drive Test',
    'SINGLE_MODULE_ROTATION': 'Single Module Rotation Test',
    'MODULE_CHARACTERIZATION': 'Module Characterization',
    'TRAJECTORY_FOLLOWING': 'Trajectory Following Test'
}

# Module test parameters
MAX_TEST_VELOCITY = 2.0  # meters per second
MAX_TEST_ACCELERATION = 1.0  # meters per second squared
MAX_TEST_ROTATION_VELOCITY = math.pi  # radians per second 