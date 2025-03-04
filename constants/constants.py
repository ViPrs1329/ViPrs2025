"""
Constants for the robot.
"""
from dataclasses import dataclass
from typing import Final

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
    # Motor CAN IDs
    LEFT_MOTOR: Final[int] = 9
    RIGHT_MOTOR: Final[int] = 10

    # Elevator Positions (in meters)
    RETRACTED_POSITION: Final[float] = 0.0
    LOW_POSITION: Final[float] = 0.5
    MEDIUM_POSITION: Final[float] = 1.2
    HIGH_POSITION: Final[float] = 2.0

    # Elevator Characteristics
    GEAR_RATIO: Final[float] = 16.0
    DRUM_DIAMETER_METERS: Final[float] = 0.0508  # 2 inches

@dataclass
class CoralManipulatorConstants:
    # Motor CAN IDs
    LEFT_MOTOR: Final[int] = 11
    RIGHT_MOTOR: Final[int] = 12

    # Sensor DIO Ports
    LEFT_SENSOR: Final[int] = 0
    RIGHT_SENSOR: Final[int] = 1

    # Motor Characteristics
    GEAR_RATIO: Final[float] = 4.0
    INTAKE_SPEED: Final[float] = 0.7
    OUTTAKE_SPEED: Final[float] = -0.7

@dataclass
class AlgaeManipulatorConstants:
    # Motor CAN IDs
    ROTATION_MOTOR: Final[int] = 13
    INTAKE_MOTOR: Final[int] = 14

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