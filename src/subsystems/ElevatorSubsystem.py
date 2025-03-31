# ElevatorSubsystem.py
#
import rev
import math
import commands2

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation, MotorControllerGroup
from wpimath import controller


from constants import CANIDs, convert, elevatorConsts

from phoenix6.hardware import CANcoder, Pigeon2

class Elevator(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()

    self.currentLevel = 1

    # Motor initiation

    self.LEM = rev.SparkFlex(CANIDs.ElevatorLeft, rev.SparkFlex.MotorType.kBrushless)
    self.REM = rev.SparkFlex(CANIDs.ElevatorRight, rev.SparkFlex.MotorType.kBrushless)
    # self.LEM.setInverted(True)
    self.motorGroup = MotorControllerGroup(self.LEM, self.REM)

    self.LEE = self.LEM.getEncoder()
    self.REE = self.REM.getEncoder()

    # Set configs

    self.LEMConfig = rev.SparkBaseConfig()
    self.LEMConfig.inverted(True)
    self.LEMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.LEMConfig.smartCurrentLimit(30)
    self.REMConfig = rev.SparkBaseConfig()
    self.REMConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.REMConfig.smartCurrentLimit(30)

    # actually configure

    self.LEM.configure(self.LEMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.REM.configure(self.REMConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # set up the pid controllers
    Kp = 0.6
    Ki = 0
    Kd = 0.0
    self.elevatorPID = controller.PIDController(Kp, Ki, Kd)
    self.elevatorPID.setSetpoint(0.0)
    self.destination = 0

    # feed forward 
    kS = 0.0
    kG = 0.04
    kV = 0.1
    kA = 0.0

    self.elevatorFF = controller.ElevatorFeedforward(kS, kG, kV, kA)

  def getElevatorPosition(self):
    return self.REE.getPosition()
  
  def getElevatorVelocity(self):
    return self.REE.getVelocity()
  
  def periodic(self):
    desiredVelocity = self.elevatorPID.calculate(self.getElevatorPosition(), self.destination)
    elevatorVelocity = self.elevatorFF.calculate(self.getElevatorVelocity(), desiredVelocity)
    self.motorGroup.set(elevatorVelocity)
    # print((convert.rot2in(self.destination)) * 2 + elevatorConsts.verticalOffset)

  def gotoPosition(self, position):
    self.destination = max(position, 0)

  def stopMotors(self):
    self.motorGroup.set(0)

  def zeroElevator(self):
    self.currentLevel = 1
    self.destination = 0
    self.gotoPosition(0)