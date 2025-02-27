# ElevatorSubsystem.py
#
import rev
import math
import commands2

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpimath import controller

import constants

from phoenix6.hardware import CANcoder, Pigeon2

class Elevator(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()

    # Motor initiation

    self.LEM = rev.SparkFlex(constants.CANIDs.ElevatorLeftID, rev.SparkFlex.MotorType.kBrushless)
    self.REM = rev.SparkFlex(constants.CANIDs.ElevatorRightID, rev.SparkFlex.MotorType.kBrushless)

    # Set configs

    self.LEMConfig = rev.SparkBaseConfig()
    self.LEMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.LEMConfig.smartCurrentLimit(constants.elevatorConsts.currentLimit)
    self.REMConfig = rev.SparkBaseConfig()
    self.REMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.REMConfig.smartCurrentLimit(constants.elevatorConsts.currentLimit)

    # actually configure

    self.LEM.configure(self.LEMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.REM.configure(self.REMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

  def stopMotors(self):
    self.LEM.set(0)
    self.REM.set(0)