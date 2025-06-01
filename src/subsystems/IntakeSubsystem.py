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
from commands2 import InstantCommand
from commands2 import ParallelCommandGroup
from commands2 import SequentialCommandGroup
from commands2 import WaitCommand
from constants import CANIDs
from constants import Intake
from armUtils import ArmAngle

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        
        # initialise motors
        self.intakeMotor: SparkMax = SparkMax(CANIDs.intakeMotor, SparkMax.MotorType.kBrushless)
        intakeConfig: SparkBaseConfig = SparkBaseConfig()
        intakeConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        intakeConfig.smartCurrentLimit(Intake.Consts.intakeCurrentLimit)
        self.intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.leftArm: SparkMax = SparkMax(CANIDs.leftArmMotor, SparkMax.MotorType.kBrushless)
        self.rightArm: SparkMax = SparkMax(CANIDs.rightArmMotor, SparkMax.MotorType.kBrushless)

        # configure motors
        self.slot: ClosedLoopSlot = ClosedLoopSlot(0)

        leftArmConfig: SparkBaseConfig = SparkBaseConfig()
        leftArmConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        leftArmConfig.smartCurrentLimit(Intake.Consts.armCurrentLimit)
        leftArmConfig.inverted(True)
        leftArmConfig.closedLoop.pidf(Intake.Consts.armP, Intake.Consts.armI, Intake.Consts.armD, Intake.Consts.armFF, self.slot)
        leftArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        leftArmConfig.closedLoop.positionWrappingEnabled(False)
        self.leftArm.configure(leftArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        rightArmConfig: SparkBaseConfig = SparkBaseConfig()
        rightArmConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rightArmConfig.smartCurrentLimit(Intake.Consts.armCurrentLimit)
        rightArmConfig.inverted(False)
        rightArmConfig.closedLoop.pidf(Intake.Consts.armP, Intake.Consts.armI, Intake.Consts.armD, Intake.Consts.armFF, self.slot)
        rightArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rightArmConfig.closedLoop.positionWrappingEnabled(False)
        self.rightArm.configure(rightArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.leftArmController: SparkClosedLoopController = self.leftArm.getClosedLoopController()
        self.rightArmController: SparkClosedLoopController = self.rightArm.getClosedLoopController()

        # initialise other variables
        self.targetArmAngle: ArmAngle = Intake.Consts.default
        self.targetState: int = Intake.States.default

        self.scoreCoralCommand: SequentialCommandGroup = InstantCommand(
            lambda: self.startScoringCoral(),
            self
        ).andThen(
            WaitCommand(Intake.Consts.coralScoringTime)
        ).andThen(
            InstantCommand(
                lambda: self.stopScoringCoral(),
                self
            )
        )

        self.scoreAlgaeCommand: SequentialCommandGroup = InstantCommand(
            lambda: self.startScoringAlgae(),
            self
        ).andThen(
            WaitCommand(Intake.Consts.algaeScoringTime)
        ).andThen(
            InstantCommand(
                lambda: self.stopScoringAlgae(),
                self
            )
        )

    def startScoringCoral(self) -> None:
        self.intakeMotor.set(-Intake.Consts.intakeSpeed)

        # move to the scoring position based on the target state
        match self.targetState:
            case Intake.States.scoreCoralL1:
                self.moveTo(Intake.States.scoringCoralL1)
            case Intake.States.scoreCoralL2:
                self.moveTo(Intake.States.scoringCoralL2)
            case Intake.States.scoreCoralL3:
                self.moveTo(Intake.States.scoringCoralL3)
            case Intake.States.scoreCoralL4:
                self.moveTo(Intake.States.scoringCoralL4)

    def stopScoringCoral(self) -> None:
        self.intakeMotor.set(Intake.Consts.intakeSpeed)
        self.moveTo(Intake.States.default)

    def startScoringAlgae(self) -> None:
        self.intakeMotor.set(-Intake.Consts.intakeSpeed)

    def stopScoringAlgae(self) -> None:
        self.intakeMotor.set(Intake.Consts.intakeSpeed)
        self.moveTo(Intake.States.default)

    def initialize(self) -> None:
        """
        This function is called once when the subsystem is initialized.
        """
        self.targetArmAngle: ArmAngle = Intake.Consts.default
        self.leftArmController.setReference(self.targetArmAngle.leftRot, SparkBase.ControlType.kPosition, self.slot)
        self.rightArmController.setReference(self.targetArmAngle.rightRot, SparkBase.ControlType.kPosition, self.slot)
        self.intakeMotor.set(Intake.Consts.intakeSpeed)
        
    def periodic(self) -> None:
        pass
    
    def moveTo(self, target: int) -> None:
        """
        Move the arm to the target state.
        """

        self.targetState = target

        # set the target position based on the target state
        match target:
            case Intake.States.groundIntakeAlgae:
                self.targetArmAngle = Intake.Consts.groundIntakeAlgae
            case Intake.States.l2IntakeAlgae:
                self.targetArmAngle = Intake.Consts.l2IntakeAlgae
            case Intake.States.l3IntakeAlgae:
                self.targetArmAngle = Intake.Consts.l3IntakeAlgae
            case Intake.States.groundIntakeCoral:  
                self.targetArmAngle = Intake.Consts.groundIntakeCoral
            case Intake.States.feederIntakeCoral:
                self.targetArmAngle = Intake.Consts.feederIntakeCoral
            case Intake.States.scoreAlgaeNet:
                self.targetArmAngle = Intake.Consts.scoreAlgaeNet
            case Intake.States.scoreAlgaeProcessor:
                self.targetArmAngle = Intake.Consts.scoreAlgaeProcessor
            case Intake.States.scoreCoralL1:
                self.targetArmAngle = Intake.Consts.scoreCoralL1
            case Intake.States.scoreCoralL2:
                self.targetArmAngle = Intake.Consts.scoreCoralL2
            case Intake.States.scoreCoralL3:
                self.targetArmAngle = Intake.Consts.scoreCoralL3
            case Intake.States.scoreCoralL4:
                self.targetArmAngle = Intake.Consts.scoreCoralL4
            case Intake.States.default:
                self.targetArmAngle = Intake.Consts.default
            case Intake.States.scoringCoralL1:
                self.targetArmAngle = Intake.Consts.scoringCoralL1
            case Intake.States.scoringCoralL2:
                self.targetArmAngle = Intake.Consts.scoringCoralL2
            case Intake.States.scoringCoralL3:
                self.targetArmAngle = Intake.Consts.scoringCoralL3
            case Intake.States.scoringCoralL4:
                self.targetArmAngle = Intake.Consts.scoringCoralL4

        # set the target position for the arm motors
        self.leftArmController.setReference(self.targetArmAngle.leftRot, SparkBase.ControlType.kPosition, self.slot)
        self.rightArmController.setReference(self.targetArmAngle.rightRot, SparkBase.ControlType.kPosition, self.slot)