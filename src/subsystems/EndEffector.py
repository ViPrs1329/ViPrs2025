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
from constants import CANIDs, endEffectorConsts  # Assuming CANIDs are defined in constants.py

import grapple.LaserCAN as LC

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # 1. Algae Intake Rotation Motor
        self.algae_rotation_motor = rev.SparkMax(
            CANIDs.AlgaeIntakeRotation, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_rotation_motor.setInverted(False)  # Adjust if needed
        self.algae_rotation_motor_config = rev.SparkMaxConfig()
        self.algae_rotation_motor_config.idle_mode = rev.SparkMax.IdleMode.kBrake
        self.algae_rotation_motor_config.smart_current_limit_amps = endEffectorConsts.algaeRotCurrentLimit #limit current
        self.algae_rotation_motor.apply(self.algae_rotation_motor_config)

        # 2. Algae Intake Motor
        self.algae_intake_motor = rev.SparkMax(
            CANIDs.AlgaeIntake, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_intake_motor.setInverted(False)  # Adjust if needed
        self.algae_intake_motor_config = rev.SparkMaxConfig()
        self.algae_intake_motor_config.idle_mode = rev.SparkMax.IdleMode.kCoast #can change to brake if needed
        self.algae_intake_motor_config.smart_current_limit_amps = endEffectorConsts.algaeIntakeCurrentLimit #limit current
        self.algae_intake_motor.apply(self.algae_intake_motor_config)

        # 3. Coral Intake Left Motor
        self.coral_intake_left_motor = rev.SparkMax(
            CANIDs.CoralIntakeLeft, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_left_motor.setInverted(False)  # Adjust if needed
        self.coral_intake_left_motor_config = rev.SparkMaxConfig()
        self.coral_intake_left_motor_config.idle_mode = rev.SparkMax.IdleMode.kCoast #can change to brake if needed
        self.coral_intake_left_motor_config.smart_current_limit_amps = endEffectorConsts.coralCurrentLimit #limit current
        self.coral_intake_left_motor.apply(self.coral_intake_left_motor_config)

        # 4. Coral Intake Right Motor
        self.coral_intake_right_motor = rev.SparkMax(
            CANIDs.CoralIntakeRight, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_right_motor.setInverted(True)  # Adjust if needed, may need to be inverted
        self.coral_intake_right_motor_config = rev.SparkMaxConfig()
        self.coral_intake_right_motor_config.idle_mode = rev.SparkMax.IdleMode.kCoast #can change to brake if needed
        self.coral_intake_right_motor_config.smart_current_limit_amps = endEffectorConsts.coralCurrentLimit #limit current
        self.coral_intake_right_motor.apply(self.coral_intake_right_motor_config)

        # 5. Coral LaserCAN setups
        self.coral_intake_LC = LC.LaserCAN(CANIDs.EECoralInSensorID)
        self.coral_stop_LC = LC.LaserCAN(CANIDs.EECoralStopSensorID)

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

    def isCoralDetected(self) -> bool:
        """Check if coral is detected at entrance of intake."""
        measurement = self.coral_intake_LC.get_measurement()
        if measurement:
            distance, status = measurement
            # Use a threshold distance defined in constants.py
            return status == 0 and distance < endEffectorConsts.CORAL_DETECTION_THRESHOLD
        return False
    
    def isCoralPositioned(self) -> bool:
        """Check if coral has reached correct position inside intake."""
        measurement = self.coral_stop_LC.get_measurement()
        if measurement:
            distance, status = measurement
            return status == 0 and distance < endEffectorConsts.CORAL_STOP_THRESHOLD
        return False

    def intakeCoral(self, speed=0.7):
        """Intake coral at given speed until properly positioned."""
        if not self.isCoralPositioned():
            self.setCoralIntakeLeftSpeed(speed)
            self.setCoralIntakeRightSpeed(speed)
            return False  # Not finished
        else:
            self.stopCoralIntake()
            return True  # Finished
        
    def stopCoralIntake(self):
        """Stop the coral intake motors."""
        self.setCoralIntakeLeftSpeed(0)
        self.setCoralIntakeRightSpeed(0)
