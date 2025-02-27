# EndEffector.py
#
# This file defines the EndEffector subsystem, which controls the
# Algae Intake and Coral Intake mechanisms of the robot.
# 
# TODO:
# - Fill out CANIDs
# - Review code
# - 

import rev
import commands2
from constants import CANIDs  # Assuming CANIDs are defined in constants.py

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # 1. Algae Intake Rotation Motor
        self.algae_rotation_motor = rev.CANSparkMax(
            CANIDs.AlgaeIntakeRotation, rev.CANSparkMax.MotorType.kBrushless
        )
        self.algae_rotation_motor.setInverted(False)  # Adjust if needed
        self.algae_rotation_motor_config = rev.SparkMaxConfig()
        self.algae_rotation_motor_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        self.algae_rotation_motor_config.smart_current_limit_amps = 20 #limit current
        self.algae_rotation_motor.apply(self.algae_rotation_motor_config)

        # 2. Algae Intake Motor
        self.algae_intake_motor = rev.CANSparkMax(
            CANIDs.AlgaeIntake, rev.CANSparkMax.MotorType.kBrushless
        )
        self.algae_intake_motor.setInverted(False)  # Adjust if needed
        self.algae_intake_motor_config = rev.SparkMaxConfig()
        self.algae_intake_motor_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast #can change to brake if needed
        self.algae_intake_motor_config.smart_current_limit_amps = 20 #limit current
        self.algae_intake_motor.apply(self.algae_intake_motor_config)

        # 3. Coral Intake Left Motor
        self.coral_intake_left_motor = rev.CANSparkMax(
            CANIDs.CoralIntakeLeft, rev.CANSparkMax.MotorType.kBrushless
        )
        self.coral_intake_left_motor.setInverted(False)  # Adjust if needed
        self.coral_intake_left_motor_config = rev.SparkMaxConfig()
        self.coral_intake_left_motor_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast #can change to brake if needed
        self.coral_intake_left_motor_config.smart_current_limit_amps = 20 #limit current
        self.coral_intake_left_motor.apply(self.coral_intake_left_motor_config)

        # 4. Coral Intake Right Motor
        self.coral_intake_right_motor = rev.CANSparkMax(
            CANIDs.CoralIntakeRight, rev.CANSparkMax.MotorType.kBrushless
        )
        self.coral_intake_right_motor.setInverted(True)  # Adjust if needed, may need to be inverted
        self.coral_intake_right_motor_config = rev.SparkMaxConfig()
        self.coral_intake_right_motor_config.idle_mode = rev.CANSparkMax.IdleMode.kCoast #can change to brake if needed
        self.coral_intake_right_motor_config.smart_current_limit_amps = 20 #limit current
        self.coral_intake_right_motor.apply(self.coral_intake_right_motor_config)

    def setAlgaeRotationSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake rotation motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.algae_rotation_motor.set(speed)

    def setAlgaeIntakeSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.algae_intake_motor.set(speed)

    def setCoralIntakeLeftSpeed(self, speed: float) -> None:
        """Sets the speed of the left coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.coral_intake_left_motor.set(speed)

    def setCoralIntakeRightSpeed(self, speed: float) -> None:
        """Sets the speed of the right coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.coral_intake_right_motor.set(speed)

    def stopAllMotors(self) -> None:
        """Stops all motors in the end effector subsystem."""
        self.algae_rotation_motor.set(0)
        self.algae_intake_motor.set(0)
        self.coral_intake_left_motor.set(0)
        self.coral_intake_right_motor.set(0)

