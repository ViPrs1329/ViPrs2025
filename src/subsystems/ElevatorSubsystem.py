# ElevatorSubsystem.py
#
# 

from rev import SparkFlex
from rev import SparkBaseConfig
from rev import SparkBase
from rev import SparkClosedLoopController
from rev import ClosedLoopConfig
from rev import ClosedLoopSlot
from rev import SparkRelativeEncoder

from commands2 import Subsystem
from commands2 import InstantCommand
from commands2 import ParallelCommandGroup
from commands2 import SequentialCommandGroup
from commands2 import WaitCommand
from constants import CANIDs
from constants import Elevator

class ElevatorSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # initialise motors
        self.slot: ClosedLoopSlot = ClosedLoopSlot(0)
        self.left: SparkFlex = SparkFlex(CANIDs.leftElevatorMotor, SparkFlex.MotorType.kBrushless)
        self.right: SparkFlex = SparkFlex(CANIDs.rightElevatorMotor, SparkFlex.MotorType.kBrushless)
        
        # configure encoder
        self.leftEncoder: SparkRelativeEncoder = self.left.getEncoder()
        self.rightEncoder: SparkRelativeEncoder = self.right.getEncoder()

        # configure controllers
        self.leftController: SparkClosedLoopController = self.left.getClosedLoopController()
        self.rightController: SparkClosedLoopController = self.right.getClosedLoopController()

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

        # initialise other variables
        self.targetRevs: float = Elevator.Consts.default
        self.targetState: int = Elevator.States.default

        self.scoreCoralCommand: SequentialCommandGroup = InstantCommand(
            lambda: self.startScoringCoral(),
            self
        ).andThen(
            WaitCommand(Elevator.Consts.coralScoringTime)
        ).andThen(
            InstantCommand(
                lambda: self.stopScoringCoral(),
                self
            )
        )

        self.scoreAlgaeCommand: SequentialCommandGroup = WaitCommand(Elevator.Consts.algaeScoringTime).andThen(
            InstantCommand(
                lambda: self.moveTo(Elevator.States.default),
                self
            )
        )

    def startScoringCoral(self) -> None:
        match self.targetState:
            case Elevator.States.scoreCoralL1:
                self.moveTo(Elevator.States.scoringCoralL1)
            case Elevator.States.scoreCoralL2:
                self.moveTo(Elevator.States.scoringCoralL2)
            case Elevator.States.scoreCoralL3:
                self.moveTo(Elevator.States.scoringCoralL3)
            case Elevator.States.scoreCoralL4:
                self.moveTo(Elevator.States.scoringCoralL4)

    def stopScoringCoral(self) -> None:
        self.moveTo(Elevator.States.default)

    def initialize(self) -> None:
        self.leftController.setReference(0, SparkBase.ControlType.kPosition, self.slot)
        self.rightController.setReference(0, SparkBase.ControlType.kPosition, self.slot)

    def periodic(self) -> None:
        pass

    def isInTollerance(self) -> bool:
        # get the current position of the elevator
        leftPosition: float = self.leftEncoder.getPosition()
        rightPosition: float = self.rightEncoder.getPosition()
        distance: float = (leftPosition + rightPosition) / 2

        # get the target position
        target: float = self.targetRevs
        # check if the elevator is within the tolerance
        if abs(distance - target) < Elevator.Consts.tollerance:
            return True
        else:
            return False
        
    def moveTo(self, state: int) -> None:
        self.targetState = state
        # set the target revolutions based on the state
        match state:
            case Elevator.States.groundIntakeAlgae:
                self.targetRevs = Elevator.Consts.groundIntakeAlgae
            case Elevator.States.l2IntakeAlgae:
                self.targetRevs = Elevator.Consts.l2IntakeAlgae
            case Elevator.States.l3IntakeAlgae:
                self.targetRevs = Elevator.Consts.l3IntakeAlgae
            case Elevator.States.groundIntakeCoral:
                self.targetRevs = Elevator.Consts.groundIntakeCoral
            case Elevator.States.feederIntakeCoral:
                self.targetRevs = Elevator.Consts.feederIntakeCoral
            case Elevator.States.scoreAlgaeNet:
                self.targetRevs = Elevator.Consts.scoreAlgaeNet
            case Elevator.States.scoreAlgaeProcessor:
                self.targetRevs = Elevator.Consts.scoreAlgaeProcessor
            case Elevator.States.scoreCoralL1:
                self.targetRevs = Elevator.Consts.scoreCoralL1
            case Elevator.States.scoreCoralL2:
                self.targetRevs = Elevator.Consts.scoreCoralL2
            case Elevator.States.scoreCoralL3:
                self.targetRevs = Elevator.Consts.scoreCoralL3
            case Elevator.States.scoreCoralL4:
                self.targetRevs = Elevator.Consts.scoreCoralL4
            case Elevator.States.default:
                self.targetRevs = Elevator.Consts.default
        self.leftController.setReference(Elevator.Consts.default, SparkBase.ControlType.kPosition, self.slot)
        self.rightController.setReference(Elevator.Consts.default, SparkBase.ControlType.kPosition, self.slot)