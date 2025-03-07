# EndEffector.py
#
# This file defines the EndEffector subsystem, which controls the
# Algae Intake and Coral Intake mechanisms of the robot.
# 
# Fill out CANIDs

import rev
import commands2
from constants import CANIDs, intakeConsts, convert  # Assuming CANIDs are defined in constants.py
from wpimath.controller import PIDController
from wpilib import MotorControllerGroup
from phoenix6.hardware import CANrange

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.canRange0 = CANrange(CANIDs.canRange0)
        self.canRange1 = CANrange(CANIDs.canRange1)

        # 1. Algae Intake Rotation Motor
        self.algaeRotationMotor = rev.SparkMax(
            CANIDs.AlgaeArm, rev.SparkMax.MotorType.kBrushless
        )
        self.algaeRotationMotor.setInverted(False)  # Adjust if needed
        self.algaeRotationMotorConfig = rev.SparkMaxConfig()
        self.algaeRotationMotorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        self.algaeRotationMotorConfig.smartCurrentLimit(10) #limit current
        self.algaeRotationMotor.configure(self.algaeRotationMotorConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # 2. Algae Intake Motor
        self.algaeIntakeMotor = rev.SparkMax(
            CANIDs.AlgaeIntake, rev.SparkMax.MotorType.kBrushless
        )
        self.algaeIntakeMotor.setInverted(False)  # Adjust if needed
        self.algaeIntakeMotorConfig = rev.SparkMaxConfig()
        self.algaeIntakeMotorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast) #can change to brake if needed
        self.algaeIntakeMotorConfig.smartCurrentLimit(10) #limit current
        self.algaeIntakeMotor.configure(self.algaeIntakeMotorConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # 3. Coral Intake Left Motor
        self.coralIntakeLeftMotor = rev.SparkMax(
            CANIDs.CoralLeft, rev.SparkMax.MotorType.kBrushless
        )
        self.coralIntakeLeftMotor.setInverted(False)  # Adjust if needed
        self.coralIntakeLeftMotorConfig = rev.SparkMaxConfig()
        self.coralIntakeLeftMotorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast) #can change to brake if needed
        self.coralIntakeLeftMotorConfig.smartCurrentLimit(10) #limit current
        self.coralIntakeLeftMotor.configure(self.coralIntakeLeftMotorConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # 4. Coral Intake Right Motor
        self.coralIntakeRightMotor = rev.SparkMax(
            CANIDs.CoralRight, rev.SparkMax.MotorType.kBrushless
        )
        self.coralIntakeRightMotor.setInverted(True)  # Adjust if needed, may need to be inverted
        self.coralIntakeRightMotorConfig = rev.SparkMaxConfig()
        self.coralIntakeRightMotorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast) #can change to brake if needed
        self.coralIntakeRightMotorConfig.smartCurrentLimit(10) #limit current
        self.coralIntakeRightMotor.configure(self.coralIntakeRightMotorConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self.coralIntakeMotor = MotorControllerGroup(self.coralIntakeLeftMotor, self.coralIntakeRightMotor)

        Kp = 0
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

    def setCoralIntakeSpeed(self, speed: float) -> None:
        """Sets the speed of the coral intake motors.

        Args:
            speed (float): The desired speed (-1.0 to 1.0).
        """
        self.coralIntakeMotor.set(speed)

    def stopAllMotors(self) -> None:
        """Stops all motors in the end effector subsystem."""
        self.algaeRotationMotor.set(0)
        self.algaeIntakeMotor.set(0)
        self.coralIntakeMotor.set(0)

    def stopCoralMotors(self):
        self.coralIntakeMotor.set(0)

    def startCoralMotors(self):
        self.coralIntakeMotor.set(intakeConsts.intakeSpeed)
        print('startCoralMotors()')

    def setAlgaeArmAngle(self, angle):
        self.algaePID.setSetpoint(angle)

    def startAlgaeIntake(self):
        self.algaeIntakeMotor.set(intakeConsts.algaeIntakeSpeed)

    def getRange(self): #returns inches
        return convert.m2in(self.canRange0.get_distance())

    def hasCoral(self):
        return self.canRange1.get_distance() <= intakeConsts.coralDetectionThreshold

    def stopAlgaeIntake(self):
        self.algaeIntakeMotor.set(0)