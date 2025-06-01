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

from math import pi

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
        leftConfig.softLimit.forwardSoftLimitEnabled(True)
        leftConfig.softLimit.reverseSoftLimitEnabled(True)
        leftConfig.softLimit.forwardSoftLimit(self.metersToRevolutions(Elevator.Consts.maxHeightMeters))  # Convert meters to revolutions
        leftConfig.softLimit.reverseSoftLimit(self.metersToRevolutions(Elevator.Consts.minHeightMeters))  # Convert meters to revolutions

        leftConfig.encoder.positionConversionFactor(self.metersToRevolutions(1))  # Set conversion factor for encoder
        leftConfig.encoder.velocityConversionFactor(self.metersToRevolutions(1) / 60)  # Set conversion factor for velocity

        leftConfig.closedLoop.pidf(Elevator.Consts.p, Elevator.Consts.i, Elevator.Consts.d, Elevator.Consts.ff, self.slot)
        leftConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        leftConfig.closedLoop.positionWrappingEnabled(False)
        leftConfig.inverted(False)

        rightConfig: SparkBaseConfig = SparkBaseConfig()
        rightConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rightConfig.smartCurrentLimit(Elevator.Consts.currentLimit)
        rightConfig.softLimit.forwardSoftLimitEnabled(True)
        rightConfig.softLimit.reverseSoftLimitEnabled(True)
        rightConfig.softLimit.forwardSoftLimit(self.metersToRevolutions(Elevator.Consts.maxHeightMeters))  # Convert meters to revolutions
        rightConfig.softLimit.reverseSoftLimit(self.metersToRevolutions(Elevator.Consts.minHeightMeters))  # Convert meters to revolutions

        rightConfig.encoder.positionConversionFactor(self.metersToRevolutions(1))  # Set conversion factor for encoder
        rightConfig.encoder.velocityConversionFactor(self.metersToRevolutions(1) / 60)  # Set conversion factor for velocity

        rightConfig.closedLoop.pidf(Elevator.Consts.p, Elevator.Consts.i, Elevator.Consts.d, Elevator.Consts.ff, self.slot)
        rightConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rightConfig.closedLoop.positionWrappingEnabled(False)
        rightConfig.inverted(True)

        self.left.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.right.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # initialise other variables
        self.targetHeight: float = Elevator.Consts.default
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

        # don't need to move elevator, since for algae scoring it is done solely 
        # by the intake, as the elevator is already in position
        self.scoreAlgaeCommand: SequentialCommandGroup = WaitCommand(Elevator.Consts.algaeScoringTime).andThen(
            InstantCommand(
                lambda: self.moveTo(Elevator.States.default),
                self
            )
        )

    def metersToRevolutions(self, meters: float) -> float:
        """
        Convert meters of elevator travel to motor revolutions
        For two-stage telescoping: 
        - If we want the end effector to move 2 meters
        - The motor needs to move the outer stage 1 meter
        - So divide desired height by 2
        """
        sprocketCircumference = pi * Elevator.Consts.sprocketDiameterMeters
        outer_stage_travel = meters / 2.0  # Convert end effector height to outer stage movement
        return (outer_stage_travel / sprocketCircumference) * Elevator.Consts.gearRatio

    def revolutionsToMeters(self, revolutions: float) -> float:
        """
        Convert motor revolutions to meters of elevator travel
        For two-stage telescoping:
        - If motor moves outer stage 1 meter
        - End effector moves 2 meters
        - So multiply final height by 2
        """
        sprocketCircumference = pi * Elevator.Consts.sprocketDiameterMeters
        outer_stage_travel = (revolutions / Elevator.Consts.gearRatio) * sprocketCircumference
        return outer_stage_travel * 2.0  # Convert outer stage movement to end effector height

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

    def isInTolerance(self) -> bool:
        # get the current position of the elevator
        leftPosition: float = self.leftEncoder.getPosition()
        rightPosition: float = self.rightEncoder.getPosition()
        distance: float = (leftPosition + rightPosition) / 2

        # get the target position
        target: float = self.targetHeight
        # check if the elevator is within the tolerance
        if abs(distance - target) < Elevator.Consts.tolerance:
            return True
        else:
            return False
        
    def moveTo(self, state: int) -> None:
        self.targetState = state
        # set the target revolutions based on the state
        match state:
            case Elevator.States.groundIntakeAlgae:
                self.targetHeight = Elevator.Consts.groundIntakeAlgae
            case Elevator.States.l2IntakeAlgae:
                self.targetHeight = Elevator.Consts.l2IntakeAlgae
            case Elevator.States.l3IntakeAlgae:
                self.targetHeight = Elevator.Consts.l3IntakeAlgae
            case Elevator.States.groundIntakeCoral:
                self.targetHeight = Elevator.Consts.groundIntakeCoral
            case Elevator.States.feederIntakeCoral:
                self.targetHeight = Elevator.Consts.feederIntakeCoral
            case Elevator.States.scoreAlgaeNet:
                self.targetHeight = Elevator.Consts.scoreAlgaeNet
            case Elevator.States.scoreAlgaeProcessor:
                self.targetHeight = Elevator.Consts.scoreAlgaeProcessor
            case Elevator.States.scoreCoralL1:
                self.targetHeight = Elevator.Consts.scoreCoralL1
            case Elevator.States.scoreCoralL2:
                self.targetHeight = Elevator.Consts.scoreCoralL2
            case Elevator.States.scoreCoralL3:
                self.targetHeight = Elevator.Consts.scoreCoralL3
            case Elevator.States.scoreCoralL4:
                self.targetHeight = Elevator.Consts.scoreCoralL4
            case Elevator.States.default:
                self.targetHeight = Elevator.Consts.default

        if self.targetHeight > Elevator.Consts.maxHeightMeters or self.targetHeight < Elevator.Consts.minHeightMeters:
            raise ValueError("Target height is out of bounds")
        
        self.leftController.setReference(self.targetHeight, SparkBase.ControlType.kPosition, self.slot)
        self.rightController.setReference(self.targetHeight, SparkBase.ControlType.kPosition, self.slot)