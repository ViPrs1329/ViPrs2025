# IntakeSubsystem.py
#
# 
from rev import SparkMax
from rev import SparkBaseConfig
from rev import SparkBase
from rev import SparkClosedLoopController
from rev import ClosedLoopConfig
from rev import ClosedLoopSlot

from commands2 import Subsystem
from constants import CANIDs
from constants import Arm

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        
        # initialise motors
        self.intakeMotor: SparkMax = SparkMax(CANIDs.intakeMotor, SparkMax.MotorType.kBrushless)
        intakeConfig: SparkBaseConfig = SparkBaseConfig()
        intakeConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        intakeConfig.smartCurrentLimit(Arm.Consts.intakeCurrentLimit)
        self.intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.leftArm: SparkMax = SparkMax(CANIDs.leftArmMotor, SparkMax.MotorType.kBrushless)
        self.rightArm: SparkMax = SparkMax(CANIDs.rightArmMotor, SparkMax.MotorType.kBrushless)

        # configure motors
        self.slot: ClosedLoopSlot = ClosedLoopSlot(0)

        leftArmConfig: SparkBaseConfig = SparkBaseConfig()
        leftArmConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        leftArmConfig.smartCurrentLimit(Arm.Consts.armCurrentLimit)
        leftArmConfig.inverted(True)
        leftArmConfig.closedLoop.pidf(Arm.Consts.armP, Arm.Consts.armI, Arm.Consts.armD, Arm.Consts.armFF, self.slot)
        leftArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        leftArmConfig.closedLoop.positionWrappingEnabled(False)
        self.leftArm.configure(leftArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        rightArmConfig: SparkBaseConfig = SparkBaseConfig()
        rightArmConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rightArmConfig.smartCurrentLimit(Arm.Consts.armCurrentLimit)
        rightArmConfig.inverted(False)
        rightArmConfig.closedLoop.pidf(Arm.Consts.armP, Arm.Consts.armI, Arm.Consts.armD, Arm.Consts.armFF, self.slot)
        rightArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rightArmConfig.closedLoop.positionWrappingEnabled(False)
        self.rightArm.configure(rightArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.leftArmController: SparkClosedLoopController = self.leftArm.getClosedLoopController()
        self.rightArmController: SparkClosedLoopController = self.rightArm.getClosedLoopController()

        # initialise other variables
        self.targetRevs: float = Arm.Consts.default

    def initialize(self) -> None:
        """
        This function is called once when the command is initialized.
        """
        self.targetRevs = Arm.Consts.default
        self.leftArmController.setReference(self.targetRevs, SparkBase.ControlType.kPosition, self.slot)
        self.rightArmController.setReference(self.targetRevs, SparkBase.ControlType.kPosition, self.slot)
        self.intakeMotor.set(Arm.Consts.intakeSpeed)
        
    def periodic(self) -> None:
        pass
    
    def moveTo(self, target: Arm.States) -> None:
        """
        Move the arm to the target state.
        """
        match target:
            case Arm.States.groundIntakeAlgae:
                self.targetRevs = Arm.Consts.groundIntakeAlgae
            case Arm.States.l2IntakeAlgae:
                self.targetRevs = Arm.Consts.l2IntakeAlgae
            case Arm.States.l3IntakeAlgae:
                self.targetRevs = Arm.Consts.l3IntakeAlgae
            case Arm.States.groundIntakeCoral:  
                self.targetRevs = Arm.Consts.groundIntakeCoral
            case Arm.States.feederIntakeCoral:
                self.targetRevs = Arm.Consts.feederIntakeCoral
            case Arm.States.scoreAlgaeNet:
                self.targetRevs = Arm.Consts.scoreAlgaeNet
            case Arm.States.scoreAlgaeProcessor:
                self.targetRevs = Arm.Consts.scoreAlgaeProcessor
            case Arm.States.scoreCoralL1:
                self.targetRevs = Arm.Consts.scoreCoralL1
            case Arm.States.scoreCoralL2:
                self.targetRevs = Arm.Consts.scoreCoralL2
            case Arm.States.scoreCoralL3:
                self.targetRevs = Arm.Consts.scoreCoralL3
            case Arm.States.scoreCoralL4:
                self.targetRevs = Arm.Consts.scoreCoralL4
            case Arm.States.default:
                self.targetRevs = Arm.Consts.default
        self.leftArmController.setReference(self.targetRevs, SparkBase.ControlType.kPosition, self.slot)
        self.rightArmController.setReference(self.targetRevs, SparkBase.ControlType.kPosition, self.slot)