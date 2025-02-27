# IntakeSubsystem.py
#
# 
import rev
import math
import commands2

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpimath import controller

from constants import CANIDs

from phoenix6.hardware import CANcoder, Pigeon2

class Intake(commands2.subsystem):
  def __init__(self) -> None:
    super().__init__()

    # Motor initiation

    self.CoralRight = rev.SparkMax(CANIDs.CoralLeft, rev.SparkMax.MotorType.kBrushless)
    self.CoralLeft =  rev.SparkMax(CANIDs.CoralRight, rev.SparkMax.MotorType.kBrushless)
    self.AlgaeIntake = rev.SparkMax(CANIDs.AlgaeIntake, rev.SparkMax.MotorType.kBrushless)
    self.AlgaeArm = rev.SparkMax(CANIDs.AlgaeArm, rev.SparkMax.MotorType.kBrushless)

