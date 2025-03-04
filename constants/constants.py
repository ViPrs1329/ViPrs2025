"""
Constants for the robot.
"""
from dataclasses import dataclass
from typing import Final
from math import pi

@dataclass
class DriveConstants:
    # Controller ports
    DRIVER_CONTROLLER_PORT: Final[int] = 0
    OPERATOR_CONTROLLER_PORT: Final[int] = 1

    # Motor CAN IDs
    FRONT_LEFT_DRIVE_MOTOR: Final[int] = 1
    FRONT_LEFT_TURN_MOTOR: Final[int] = 2
    FRONT_RIGHT_DRIVE_MOTOR: Final[int] = 3
    FRONT_RIGHT_TURN_MOTOR: Final[int] = 4
    BACK_LEFT_DRIVE_MOTOR: Final[int] = 5
    BACK_LEFT_TURN_MOTOR: Final[int] = 6
    BACK_RIGHT_DRIVE_MOTOR: Final[int] = 7
    BACK_RIGHT_TURN_MOTOR: Final[int] = 8

    # CANcoder IDs
    FRONT_LEFT_CANCODER: Final[int] = 21
    FRONT_RIGHT_CANCODER: Final[int] = 22
    BACK_LEFT_CANCODER: Final[int] = 23
    BACK_RIGHT_CANCODER: Final[int] = 24

    # Pigeon 2.0 ID
    PIGEON_ID: Final[int] = 25  # Update with actual CAN ID

    # CANcoder offsets (in rotations)
    FRONT_LEFT_OFFSET: Final[float] = 0.0  # Replace with actual measured offset
    FRONT_RIGHT_OFFSET: Final[float] = 0.0  # Replace with actual measured offset
    BACK_LEFT_OFFSET: Final[float] = 0.0  # Replace with actual measured offset
    BACK_RIGHT_OFFSET: Final[float] = 0.0  # Replace with actual measured offset

    # Swerve Drive Characteristics
    WHEEL_DIAMETER_METERS: Final[float] = 0.1016  # 4 inches
    DRIVE_GEAR_RATIO: Final[float] = 8.14  # SDS MK4i L2
    TURN_GEAR_RATIO: Final[float] = 150/7  # SDS MK4i

    # Drive Speed Multipliers
    NORMAL_SPEED_MULTIPLIER: Final[float] = 1.0
    PRECISION_SPEED_MULTIPLIER: Final[float] = 0.5
    BOOST_SPEED_MULTIPLIER: Final[float] = 1.5

@dataclass
class ElevatorConstants:
    # Motor IDs
    LEFT_MOTOR_ID: Final[int] = 20  # Update with actual CAN ID
    RIGHT_MOTOR_ID: Final[int] = 21  # Update with actual CAN ID
    
    # Encoder
    THROUGH_BORE_ENCODER_ID: Final[int] = 0  # Update with actual CAN ID
    
    # Gear ratios and mechanical constants
    GEAR_RATIO: Final[float] = 16.0  # 16:1 planetary gearbox
    DRUM_DIAMETER_METERS: Final[float] = 0.0508  # 2 inch diameter drum (update with actual)
    DRUM_CIRCUMFERENCE: Final[float] = DRUM_DIAMETER_METERS * pi
    
    # Conversion factors
    POSITION_CONVERSION_FACTOR: Final[float] = DRUM_CIRCUMFERENCE / GEAR_RATIO  # meters per motor rotation
    VELOCITY_CONVERSION_FACTOR: Final[float] = POSITION_CONVERSION_FACTOR / 60.0  # meters per second
    
    # PID Values
    kP: Final[float] = 5.0
    kI: Final[float] = 0.0
    kD: Final[float] = 0.0
    kFF: Final[float] = 0.0
    
    # Motion Profile Constraints
    MAX_VELOCITY: Final[float] = 2.0  # meters per second
    MAX_ACCELERATION: Final[float] = 2.0  # meters per second squared
    
    # Current Limits
    CURRENT_LIMIT: Final[int] = 40  # amps
    TRIGGER_THRESHOLD_CURRENT: Final[int] = 35  # amps
    TRIGGER_THRESHOLD_TIME: Final[float] = 0.1  # seconds
    
    # Position Setpoints (in meters from base)
    BASE_HEIGHT: Final[float] = 0.0
    L1_HEIGHT: Final[float] = 0.5
    L2_HEIGHT: Final[float] = 1.0
    L3_HEIGHT: Final[float] = 1.5
    L4_HEIGHT: Final[float] = 2.0  # TBD
    
    # Soft Limits (in meters)
    MIN_HEIGHT: Final[float] = -0.05  # Slightly below 0 to ensure we can reach base
    MAX_HEIGHT: Final[float] = 2.1  # Slightly above max height
    
    # Tolerance for position control
    POSITION_TOLERANCE: Final[float] = 0.02  # meters
    VELOCITY_TOLERANCE: Final[float] = 0.05  # meters per second

@dataclass
class CoralManipulatorConstants:
    # Motor IDs
    LEFT_MOTOR: Final[int] = 30  # Update with actual CAN ID
    RIGHT_MOTOR: Final[int] = 31  # Update with actual CAN ID
    
    # Sensor IDs
    LEFT_SENSOR: Final[int] = 40  # Update with actual CAN ID
    RIGHT_SENSOR: Final[int] = 41  # Update with actual CAN ID
    
    # Detection threshold (4 inches = ~100mm)
    DETECTION_THRESHOLD_MM: Final[int] = 100

    # Motor Characteristics
    GEAR_RATIO: Final[float] = 4.0
    INTAKE_SPEED: Final[float] = 0.7
    OUTTAKE_SPEED: Final[float] = -0.7

@dataclass
class AlgaeManipulatorConstants:
    # Motor IDs
    ROTATION_MOTOR: Final[int] = 32  # Update with actual CAN ID
    INTAKE_MOTOR: Final[int] = 33    # Update with actual CAN ID

    # Encoder Port
    ABSOLUTE_ENCODER: Final[int] = 2

    # Arm Positions (in radians)
    RETRACTED_POSITION: Final[float] = 0.0
    TOP_PICKUP_POSITION: Final[float] = 2.1  # ~120 degrees
    BOTTOM_PICKUP_POSITION: Final[float] = -0.52  # ~-30 degrees

    # Motor Characteristics
    ROTATION_GEAR_RATIO: Final[float] = 49.0
    INTAKE_GEAR_RATIO: Final[float] = 4.0
    INTAKE_SPEED: Final[float] = 0.8

@dataclass
class OIConstants:
    # Controller Ports (already defined in DriveConstants, but repeated here for clarity)
    DRIVER_CONTROLLER_PORT: Final[int] = 0
    OPERATOR_CONTROLLER_PORT: Final[int] = 1
    
    # Driver Controller (Drive)
    # Left stick: Translation control (x/y movement)
    # Right stick X-axis: Rotation control
    DRIVE_DEADBAND: Final[float] = 0.05
    
    # Driver Buttons
    FIELD_RELATIVE_TOGGLE_BUTTON: Final[int] = 8  # Start button
    PRECISION_MODE_BUTTON: Final[int] = 5  # Left bumper
    BOOST_MODE_BUTTON: Final[int] = 6  # Right bumper
    RESET_GYRO_BUTTON: Final[int] = 7  # Back button
    X_FORMATION_BUTTON: Final[int] = 1  # A button - wheels in X for stability
    
    # Operator Controller (Mechanisms)
    # Elevator Controls
    ELEVATOR_BASE_BUTTON: Final[int] = 1  # A button
    ELEVATOR_L1_BUTTON: Final[int] = 2  # B button
    ELEVATOR_L2_BUTTON: Final[int] = 3  # X button
    ELEVATOR_L3_BUTTON: Final[int] = 4  # Y button
    ELEVATOR_L4_BUTTON: Final[int] = 10  # Right stick button (less common position)
    
    # Coral Manipulator Controls
    CORAL_INTAKE_BUTTON: Final[int] = 5  # Left bumper
    CORAL_OUTTAKE_BUTTON: Final[int] = 6  # Right bumper
    
    # Algae Manipulator Controls
    ALGAE_RETRACTED_BUTTON: Final[int] = 7  # Back button
    ALGAE_TOP_PICKUP_BUTTON: Final[int] = 8  # Start button
    ALGAE_BOTTOM_PICKUP_BUTTON: Final[int] = 9  # Left stick button
    ALGAE_INTAKE_AXIS: Final[int] = 2  # Left trigger
    ALGAE_OUTTAKE_AXIS: Final[int] = 3  # Right trigger
    AXIS_THRESHOLD: Final[float] = 0.5  # Threshold for trigger activation 