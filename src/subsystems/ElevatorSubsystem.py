# ElevatorSubsystem.py
#
import rev
import math
import commands2

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation, MotorControllerGroup
from wpimath import controller

from constants import CANIDs

from phoenix6.hardware import CANcoder, Pigeon2

class Elevator(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()

    self.currentLevel = 0

    # Motor initiation

    self.LEM = rev.SparkFlex(CANIDs.ElevatorLeft, rev.SparkFlex.MotorType.kBrushless)
    self.REM = rev.SparkFlex(CANIDs.ElevatorRight, rev.SparkFlex.MotorType.kBrushless)
    self.LEM.setInverted(False)
    self.REM.setInverted(True)
    self.motorGroup = MotorControllerGroup(self.LEM, self.REM)

    self.LEE = self.LEM.getEncoder()
    self.REE = self.REM.getEncoder()

    # Set configs

    self.LEMConfig = rev.SparkBaseConfig()
    self.LEMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.LEMConfig.smartCurrentLimit(10)
    self.REMConfig = rev.SparkBaseConfig()
    self.REMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.REMConfig.smartCurrentLimit(10)

    # actually configure

    self.LEM.configure(self.LEMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.REM.configure(self.REMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # set up the pid controllers
    Kp = 1
    Ki = 0
    Kd = 0
    self.elevatorPID = controller.PIDController(Kp, Ki, Kd)
    self.elevatorPID.enableContinuousInput(-.5,.5)
    self.elevatorPID.setSetpoint(0.0)

    self.destination = 0

  def getElevatorPosition(self):
    return self.REE.getPosition()
  
  def periodic(self):
    elevatorVelocity = self.elevatorPID.calculate(self.getElevatorPosition(), self.destination)
    self.motorGroup.set(elevatorVelocity)

  def gotoPosition(self, position):
    self.destination = position

  def stopMotors(self):
    self.motorGroup.set(0)