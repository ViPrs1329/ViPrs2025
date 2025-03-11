# EndEffector.py
#
# This file defines the EndEffector subsystem, which controls the
# Algae Intake and Coral Intake mechanisms of the robot.
# 
# Fill out CANIDs

import rev
import commands2
from constants import CANIDs, intakeConsts, convert  # Assuming CANIDs are defined in constants.py
from wpimath.controller import PIDController, ArmFeedforward
import ntcore
import wpilib

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable("EE Table")
        self.EEEXboxController = wpilib.XboxController(1)
        self.i = 0

        # 1. Algae Intake Rotation Motor
        self.algae_rotation_motor = rev.SparkMax(
            CANIDs.AlgaeArm, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_rotation_motor.setInverted(False)  # Adjust if needed
        self.algae_rotation_motor_config = rev.SparkBaseConfig()
        self.algae_rotation_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.algae_rotation_motor_config.smartCurrentLimit(20) #limit current
        self.algae_rotation_motor.configure(self.algae_rotation_motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # 2. Algae Intake Motor
        self.algae_intake_motor = rev.SparkMax(
            CANIDs.AlgaeIntake, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_intake_motor_config = rev.SparkBaseConfig()
        self.algae_intake_motor_config.inverted(True)  # Adjust if needed
        self.algae_intake_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.algae_intake_motor_config.smartCurrentLimit(20) #limit current
        self.algae_intake_motor.configure(self.algae_intake_motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self.algaeEncoder = self.algae_rotation_motor.getEncoder()

        # 3. Coral Intake Left Motor
        self.coral_intake_left_motor = rev.SparkMax(
            CANIDs.CoralLeft, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_left_motor_config = rev.SparkMaxConfig()
        self.coral_intake_left_motor_config.inverted(False)  # Adjust if needed
        self.coral_intake_left_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.coral_intake_left_motor_config.smartCurrentLimit(20) #limit current
        self.coral_intake_left_motor.configure(self.coral_intake_left_motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # 4. Coral Intake Right Motor
        self.coral_intake_right_motor = rev.SparkMax(
            CANIDs.CoralRight, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_right_motor_config = rev.SparkMaxConfig()
        self.coral_intake_right_motor_config.inverted(True)  # Adjust if needed, may need to be inverted
        self.coral_intake_right_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.coral_intake_right_motor_config.smartCurrentLimit(20) #limit current
        self.coral_intake_right_motor.configure(self.coral_intake_right_motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        Kp = 1.5
        Ki = 0.1
        Kd = 0.2
        self.algaePID = PIDController(Kp, Ki, Kd)
        self.algaePID.enableContinuousInput(-0.5, 0.5)
        self.algaePID.setSetpoint(0)

        kS = 0
        kG = 0.01
        kV = 0.2
        kA = 0

        self.destination = -0.3

        self.algaeFF = ArmFeedforward(kS, kG, kV, kA)

        self.rotationOffset = self.algaeEncoder.getPosition()

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

    def periodic(self):
        if True:
            desiredVelocity = self.algaePID.calculate(self.getAlgaeArmAngle(), self.destination)
        else:
            desiredVelocity = -self.EEEXboxController.getRightY()
        elevatorVelocity = self.algaeFF.calculate(self.getAlgaeArmAngle(), desiredVelocity)
        self.algae_rotation_motor.set(elevatorVelocity)
        print(f"periodic() desiredVelocity={desiredVelocity} | elevatorVelocity={elevatorVelocity}")

    def getEEEControllerRightJoystick(self):
        return self.EEEXboxController.getRightX(), self.EEEXboxController.getRightY()

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

    def stopCoralMotors(self):
        self.coral_intake_left_motor.set(0)
        self.coral_intake_right_motor.set(0)

    def startCoralMotors(self):
        self.coral_intake_left_motor.set(intakeConsts.intakeSpeed)
        self.coral_intake_right_motor.set(intakeConsts.intakeSpeed)
    
    def setAlgaeArmAngle(self, angle):
        self.algaePID.setSetpoint(angle)

    def getAlgaeArmRotations(self):
        return self.algaeEncoder.getPosition() - self.rotationOffset

    def getAlgaeArmAngle(self): # should return arm value in from standard position (0 is horizontally forward) 
        return convert.rot2angAlgae(self.getAlgaeArmRotations())

    def getAlgaeArmVelocity(self):
        return self.algaeEncoder.getVelocity()

    def zeroAlgaeArm(self):
        self.destination = 0

    def startAlgaeIntake(self):
        self.algae_intake_motor.set(intakeConsts.algaeIntakeSpeed)

    def stopAlgaeIntake(self):
        self.algae_intake_motor.set(0)