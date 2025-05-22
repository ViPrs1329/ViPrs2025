# ElevatorSubsystem.py
#
# 

from rev import SparkFlex
from rev import SparkBaseConfig
from rev import SparkBase
from rev import SparkClosedLoopController
from rev import ClosedLoopConfig
from rev import ClosedLoopSlot

from commands2 import Subsystem
from constants import CANIDs
from constants import Elevator

class ElevatorSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # initialise motors
        self.slot: ClosedLoopSlot = ClosedLoopSlot(0)
        self.left: SparkFlex = SparkFlex(CANIDs.leftElevatorMotor, SparkFlex.MotorType.kBrushless)
        self.right: SparkFlex = SparkFlex(CANIDs.rightElevatorMotor, SparkFlex.MotorType.kBrushless)
        
        # configure controllers
        self.leftController: SparkClosedLoopController = self.left.getClosedLoopController()
        self.rightController: SparkClosedLoopController = self.right.getClosedLoopController()
        
        self.leftController.setReference(0, SparkBase.ControlType.kPosition, self.slot)
        self.rightController.setReference(0, SparkBase.ControlType.kPosition, self.slot)

        # configure motors
        leftConfig: SparkBaseConfig = SparkBaseConfig()
        leftConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        leftConfig.smartCurrentLimit(Elevator.Consts.currentLimit)

        leftConfig.closedLoop.pidf(Elevator.Consts.p, Elevator.Consts.i, Elevator.Consts.d, Elevator.Consts.ff, self.slot)
        leftConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        leftConfig.closedLoop.positionWrappingEnabled(False)
        leftConfig.inverted(False)

        rightConfig: SparkBaseConfig = SparkBaseConfig()
        rightConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rightConfig.smartCurrentLimit(Elevator.Consts.currentLimit)

        rightConfig.closedLoop.pidf(Elevator.Consts.p, Elevator.Consts.i, Elevator.Consts.d, Elevator.Consts.ff, self.slot)
        rightConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rightConfig.closedLoop.positionWrappingEnabled(False)
        rightConfig.inverted(True)

        self.left.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.right.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

    def periodic(self) -> None:
        pass

    def moveTo(self, state: Elevator.States) -> None:
        match state:
            case Elevator.States.groundIntakeAlgae:
                self.leftController.setReference(Elevator.Consts.groundIntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.groundIntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.l2IntakeAlgae:
                self.leftController.setReference(Elevator.Consts.l2IntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.l2IntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.l3IntakeAlgae:
                self.leftController.setReference(Elevator.Consts.l3IntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.l3IntakeAlgae, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.groundIntakeCoral:
                self.leftController.setReference(Elevator.Consts.groundIntakeCoral, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.groundIntakeCoral, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.feederIntakeCoral:
                self.leftController.setReference(Elevator.Consts.feederIntakeCoral, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.feederIntakeCoral, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreAlgaeNet:
                self.leftController.setReference(Elevator.Consts.scoreAlgaeNet, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreAlgaeNet, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreAlgaeProcessor:
                self.leftController.setReference(Elevator.Consts.scoreAlgaeProcessor, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreAlgaeProcessor, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreCoralL1:
                self.leftController.setReference(Elevator.Consts.scoreCoralL1, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreCoralL1, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreCoralL2:
                self.leftController.setReference(Elevator.Consts.scoreCoralL2, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreCoralL2, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreCoralL3:
                self.leftController.setReference(Elevator.Consts.scoreCoralL3, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreCoralL3, SparkBase.ControlType.kPosition, self.slot)
            case Elevator.States.scoreCoralL4:
                self.leftController.setReference(Elevator.Consts.scoreCoralL4, SparkBase.ControlType.kPosition, self.slot)
                self.rightController.setReference(Elevator.Consts.scoreCoralL4, SparkBase.ControlType.kPosition, self.slot)
