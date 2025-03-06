# EndEffector.py
#
# This file defines the EndEffector subsystem, which controls the
# Algae Intake and Coral Intake mechanisms of the robot.
# 
# Fill out CANIDs

import rev
import commands2
from constants import CANIDs, intakeConsts  # Assuming CANIDs are defined in constants.py
from wpimath.controller import PIDController
class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # 1. Algae Intake Rotation Motor
        self.algaeRotationMotor = rev.SparkFlex(
            CANIDs.AlgaeArm, rev.SparkFlex.MotorType.kBrushless
        )
        self.algaeRotationMotor.setInverted(False)  # Adjust if needed
        self.algaeRotationMotorConfig = rev.SparkFlexConfig()
        self.algaeRotationMotorConfig.setIdleMode(rev.SparkFlexConfig.IdleMode.kBrake)
        self.algaeRotationMotorConfig.smartCurrentLimit(10) #limit current
        self.algaeRotationMotor.configure(self.algaeRotationMotorConfig)

        # 2. Algae Intake Motor
        self.algaeIntakeMotor = rev.SparkFlex(
            CANIDs.AlgaeIntake, rev.SparkFlex.MotorType.kBrushless
        )
        self.algaeIntakeMotor.setInverted(False)  # Adjust if needed
        self.algaeIntakeMotorConfig = rev.SparkFlexConfig()
        self.algaeIntakeMotorConfig.setIdleMode(rev.SparkFlexConfig.IdleMode.kCoast) #can change to brake if needed
        self.algaeIntakeMotorConfig.smartCurrentLimit(10) #limit current
        self.algaeIntakeMotor.configure(self.algaeIntakeMotorConfig)

        # 3. Coral Intake Left Motor
        self.coralIntakeLeftMotor = rev.SparkFlex(
            CANIDs.CoralLeft, rev.SparkFlex.MotorType.kBrushless
        )
        self.coralIntakeLeftMotor.setInverted(False)  # Adjust if needed
        self.coralIntakeLeftMotorConfig = rev.SparkFlexConfig()
        self.coralIntakeLeftMotorConfig.setIdleMode(rev.SparkFlexConfig.IdleMode.kCoast) #can change to brake if needed
        self.coralIntakeLeftMotorConfig.smartCurrentLimit(10) #limit current
        self.coralIntakeLeftMotor.configure(self.coralIntakeLeftMotorConfig)

        # 4. Coral Intake Right Motor
        self.coralIntakeRightMotor = rev.SparkFlex(
            CANIDs.CoralRight, rev.SparkFlex.MotorType.kBrushless
        )
        self.coralIntakeRightMotor.setInverted(True)  # Adjust if needed, may need to be inverted
        self.coralIntakeRightMotorConfig = rev.SparkFlexConfig()
        self.coralIntakeRightMotorConfig.setIdleMode(rev.SparkFlexConfig.IdleMode.kCoast) #can change to brake if needed
        self.coralIntakeRightMotorConfig.smartCurrentLimit(10) #limit current
        self.coralIntakeRightMotor.configure(self.coralIntakeRightMotorConfig)

        Kp = 1
        Ki = 0
        Kd = 0
        self.algaePID = PIDController(Kp, Ki, Kd)
        self.algaePID.enableContinuousInput(-0.5, 0.5)
        self.algaePID.setSetpoint(0)
    def setAlgaeRotationSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake rotation motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.algaeRotationMotor.set(speed)

    def setAlgaeIntakeSpeed(self, speed: float) -> None:
        """Sets the speed of the algae intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.algaeIntakeMotor.set(speed)

    def setCoralIntakeLeftSpeed(self, speed: float) -> None:
        """Sets the speed of the left coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.coralIntakeLeftMotor.set(speed)

    def setCoralIntakeRightSpeed(self, speed: float) -> None:
        """Sets the speed of the right coral intake motor.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.coralIntakeRightMotor.set(speed)

    def stopAllMotors(self) -> None:
        """Stops all motors in the end effector subsystem."""
        self.algaeRotationMotor.set(0)
        self.algaeIntakeMotor.set(0)
        self.coralIntakeLeftMotor.set(0)
        self.coralIntakeRightMotor.set(0)

    def stopCoralMotors(self):
        self.coralIntakeLeftMotor.set(0)
        self.coralIntakeRightMotor.set(0)

    def startCoralMotors(self):
        self.coralIntakeLeftMotor.set(intakeConsts.intakeSpeed)
        self.coralIntakeRightMotor.set(intakeConsts.intakeSpeed)
    def setAlgaeArmAngle(self, angle):
        self.algaePID.setSetpoint(angle)

    def startAlgaeIntake(self):
        self.algaeIntakeMotor.set(intakeConsts.algaeIntakeSpeed)

    def stopAlgaeIntake(self):
        self.algaeIntakeMotor.set(0)