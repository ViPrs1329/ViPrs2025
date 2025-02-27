# ElevatorSubsystem.py
#
import rev
import math
import commands2

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpimath import controller

from constants import CANIDs

from phoenix6.hardware import CANcoder, Pigeon2

class Elevator(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()

    # Motor initiation

    self.LEM = rev.SparkFlex(CANIDs.ElevatorLeft, rev.SparkFlex.MotorType.kBrushless)
    self.REM = rev.SparkFlex(CANIDs.ElevatorRight, rev.SparkFlex.MotorType.kBrushless)

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

  def drive(self, v):
    self.LEM.set(v)
    self.LEM.set(-v)

  def stopMotors(self):
    self.LEM.set(0)
    self.REM.set(0)