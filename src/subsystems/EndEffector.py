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
import math

class EndEffector(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        inst = ntcore.NetworkTableInstance.getDefault()
        self.table = inst.getTable("EE Table")
        self.EEEXboxController = wpilib.XboxController(2)
        self.i = 0

        # 1. Algae Intake Rotation Motor
        self.algae_rotation_motor = rev.SparkMax(
            CANIDs.AlgaeArm, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_rotation_motor.setInverted(False)  # Adjust if needed
        self.algae_rotation_motor_config = rev.SparkBaseConfig()
        self.algae_rotation_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.algae_rotation_motor_config.smartCurrentLimit(10) #limit current
        self.algae_rotation_motor_config.inverted(False)
        self.algae_rotation_motor.configure(self.algae_rotation_motor_config, 
                                            rev.SparkBase.ResetMode.kNoResetSafeParameters,  # THIS PARAMETER COST ME 3 HOURS OF MY LIFE!!!
                                            rev.SparkBase.PersistMode.kPersistParameters)

        # 2. Algae Intake Motor
        self.algae_intake_motor = rev.SparkMax(
            CANIDs.AlgaeIntake, rev.SparkMax.MotorType.kBrushless
        )
        self.algae_intake_motor_config = rev.SparkBaseConfig()
        self.algae_intake_motor_config.inverted(True)  # Adjust if needed
        self.algae_intake_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.algae_intake_motor_config.smartCurrentLimit(20) #limit current
        self.algae_intake_motor.configure(self.algae_intake_motor_config, 
                                          rev.SparkBase.ResetMode.kNoResetSafeParameters, 
                                          rev.SparkBase.PersistMode.kPersistParameters)

        self.algaeEncoder = self.algae_rotation_motor.getAbsoluteEncoder()

        # 3. Coral Intake Left Motor
        self.coral_intake_left_motor = rev.SparkMax(
            CANIDs.CoralLeft, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_left_motor_config = rev.SparkMaxConfig()
        self.coral_intake_left_motor_config.inverted(False)  # Adjust if needed
        self.coral_intake_left_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.coral_intake_left_motor_config.smartCurrentLimit(20) #limit current
        self.coral_intake_left_motor.configure(self.coral_intake_left_motor_config, 
                                               rev.SparkBase.ResetMode.kNoResetSafeParameters, 
                                               rev.SparkBase.PersistMode.kPersistParameters)

        # 4. Coral Intake Right Motor
        self.coral_intake_right_motor = rev.SparkMax(
            CANIDs.CoralRight, rev.SparkMax.MotorType.kBrushless
        )
        self.coral_intake_right_motor_config = rev.SparkMaxConfig()
        self.coral_intake_right_motor_config.inverted(True)  # Adjust if needed, may need to be inverted
        self.coral_intake_right_motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake) #can change to brake if needed
        self.coral_intake_right_motor_config.smartCurrentLimit(20) #limit current
        self.coral_intake_right_motor.configure(self.coral_intake_right_motor_config, 
                                                rev.SparkBase.ResetMode.kNoResetSafeParameters, 
                                                rev.SparkBase.PersistMode.kPersistParameters)

        Kp = 0.5
        Ki = 0.0
        Kd = 0.0
        self.algaePID = PIDController(Kp, Ki, Kd)
        self.algaePID.setSetpoint(0)

        kS = 0
        kG = 0  # This is not necessary and actually causes issues even when at it's starting position
        kV = 0
        kA = 0

        self.algaeDestination = 3.1

        self.algaeFF = ArmFeedforward(kS, kG, kV, kA)

        self.rotationOffset = self.algaeEncoder.getPosition()

        self.flopArm = False
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
        current_angle = self.getAlgaeArmAngle()

        error = self.algaeDestination - current_angle
        dest = self.algaeDestination
        if self.flopArm:
            dest = intakeConsts.algaeStoredSetpoint

        pid_output = self.algaePID.calculate(current_angle, dest)

        ff_output = self.algaeFF.calculate(current_angle, 0)

        motor_output = pid_output + ff_output

        motor_output = max(-1, min(1, motor_output))
        # print(motor_output)
        
        self.algae_rotation_motor.set(motor_output)
        
        # Debug output - convert back to degrees for easier reading
        # print(f"Arm: caR={current_angle:.2f} dest={dest:.2f} pidO={pid_output:.2f} ffO={ff_output:.2f} mO={motor_output:.2f} ic={self.algae_intake_motor.getOutputCurrent():.2f} flop={self.flopArm}")

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

        self.algaeDestination = 3.1

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

    '''
    def getAlgaeArmAngle(self): # should return arm value in from standard position (0 is horizontally forward) 
        return convert.rot2angAlgae(self.getAlgaeArmRotations())
    '''

    def getAlgaeArmAngle(self): 
        """
        Returns arm angle in radians where:
        0 rad = straight down
        π/2 rad (≈1.57) = horizontal
        2π/3 rad (≈2.09) = max upward position (120°)
        """
        
        raw_position = self.algaeEncoder.getPosition()   
        
        angle_radians = (raw_position - 0.45) * (2 * math.pi)

        angle_radians = max(0, min(angle_radians, math.pi))

        # print(f"getAlgaeArmAngle() - rP={raw_position:.3f} aR={angle_radians:.3f} dest={self.algaeDestination:.2f}")
        
        return angle_radians

    def getAlgaeArmVelocity(self):
        return self.algaeEncoder.getVelocity()

    def zeroAlgaeArm(self):
        self.algaeDestination = 3.1

    def startAlgaeIntake(self):
        self.algae_intake_motor.set(intakeConsts.algaeIntakeSpeed)

    def stopAlgaeIntake(self):
        self.algae_intake_motor.set(0)