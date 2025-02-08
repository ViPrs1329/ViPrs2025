from commands2 import Subsystem
import rev
import constants
from wpimath import controller

class Elevator(Subsystem):
  def __init__(self):
    super().__init__()

    # init motors
    self.elevator1 = rev.SparkMax(constants.CANIDs.elevator1, rev.SparkMax.MotorType.kBrushless)
    self.elevator2 = rev.SparkMax(constants.CANIDs.elevator2, rev.SparkMax.MotorType.kBrushless)
    # set motor configs
    self.elevator1Config = rev.SparkBaseConfig()
    self.elevator1Config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.elevator1Config.smartCurrentLimit(10)
    self.elevator2Config = rev.SparkBaseConfig()
    self.elevator2Config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.elevator2Config.smartCurrentLimit(10)

    # configure motors to their configs
    self.elevator1.configure(self.elevator1Config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.elevator2.configure(self.elevator2Config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # PID Setup
    Kp = 4
    self.elevator1PID = controller.PIDController(Kp, 0, 0)
    self.elevator1PID.enableContinuousInput(-0.5, 0.5)
    self.elevator1PID.setSetpoint(0)
    self.elevator2PID = controller.PIDController(Kp, 0, 0)
    self.elevator2PID.enableContinuousInput(-0.5, 0.5)
    self.elevator2PID.setSetpoint(0)