from commands2 import Subsystem
import rev
import constants
from wpimath import controller

class EndEffector(Subsystem):
  def __init__(self):
    super().__init__()

    # init motors
    self.arm = rev.SparkMax(constants.CANIDs.arm, rev.SparkMax.MotorType.kBrushless)
    self.intake1 = rev.SparkMax(constants.CANIDs.intake1, rev.SparkMax.MotorType.kBrushless)
    self.intake2 = rev.SparkMax(constants.CANIDs.intake2, rev.SparkMax.MotorType.kBrushless)
    self.algaeIn = rev.SparkMax(constants.CANIDs.algae, rev.SparkMax.MotorType.kBrushless)

    # set motor configs
    self.armConfig = rev.SparkBaseConfig()
    self.armConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.armConfig.smartCurrentLimit(10)
    self.intake1Config = rev.SparkBaseConfig()
    self.intake1Config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.intake1Config.smartCurrentLimit(10)
    self.intake2Config = rev.SparkBaseConfig()
    self.intake2Config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.intake2Config.smartCurrentLimit(10)
    self.algaeInConfig = rev.SparkBaseConfig()
    self.algaeInConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    self.algaeInConfig.smartCurrentLimit(10)

    # configure motors to their configs
    self.arm.configure(self.armConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.intake1.configure(self.intake1Config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.intake2.configure(self.intake2Config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self.algaeIn.configure(self.algaeInConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    # PID Setup
    Kp = 4
    self.armPID = controller.PIDController(Kp, 0, 0)
    self.armPID.enableContinuousInput(-0.5, 0.5)
    self.armPID.setSetpoint(0)
    