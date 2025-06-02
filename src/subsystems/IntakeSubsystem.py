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

from math import cos, pi

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
        leftArmConfig.encoder.positionConversionFactor(2 * pi / Intake.Consts.gearRatio)  # Set conversion factor for encoder
        leftArmConfig.encoder.velocityConversionFactor(2 * pi / (Intake.Consts.gearRatio * 60))  # Set conversion factor for velocity
        self.leftArm.configure(leftArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        rightArmConfig: SparkBaseConfig = SparkBaseConfig()
        rightArmConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rightArmConfig.smartCurrentLimit(Intake.Consts.armCurrentLimit)
        rightArmConfig.inverted(False)
        rightArmConfig.closedLoop.pidf(Intake.Consts.armP, Intake.Consts.armI, Intake.Consts.armD, Intake.Consts.armFF, self.slot)
        rightArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rightArmConfig.closedLoop.positionWrappingEnabled(False)
        rightArmConfig.encoder.positionConversionFactor(2 * pi / Intake.Consts.gearRatio)  # Set conversion factor for encoder
        rightArmConfig.encoder.velocityConversionFactor(2 * pi / (Intake.Consts.gearRatio * 60))  # Set conversion factor for velocity
        self.rightArm.configure(rightArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.leftArmController: SparkClosedLoopController = self.leftArm.getClosedLoopController()
        self.rightArmController: SparkClosedLoopController = self.rightArm.getClosedLoopController()

        # initialise other variables
        self.targetArmAngle: ArmAngle = Intake.Consts.default
        self.targetState: int = Intake.States.default

        self.flipState: bool = False

        self.flipCommand: InstantCommand = InstantCommand(
            lambda: self.flipEndEffector(),
            self
        )

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

    def flipEndEffector(self) -> None:
        self.flipState = not self.flipState
        # flip the end effector angle

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

    def getArmAngle(self) -> float:
        """
        Get the current elevation angle of the arm.
        For a Hero's differential:
        - Average of both motors gives elevation angle
        - Difference between motors gives end effector rotation
        Returns angle in radians from horizontal
        """
        leftAngle: float = self.leftArm.getEncoder().getPosition()
        rightAngle: float = self.rightArm.getEncoder().getPosition()
        # Same direction motion = elevation
        return (leftAngle + rightAngle) / 2.0

    def getEndEffectorAngle(self) -> float:
        """
        Get the current rotation angle of the end effector.
        For a Hero's differential:
        - Difference between motors gives end effector rotation
        Returns angle in radians
        """
        leftAngle: float = self.leftArm.getEncoder().getPosition()
        rightAngle: float = self.rightArm.getEncoder().getPosition()
        # Differential motion = end effector rotation
        return (leftAngle - rightAngle) / 2.0

    def calculateFF(self) -> float:
        """
        Calculate the feedforward value for the arm motors.
        Uses cosine compensation for gravity.
        
        Returns:
            float: Feedforward voltage to apply to motors
            kG * cos(theta) where:
            - kG is the gravity compensation constant
            - theta is the arm angle from horizontal
        """
        kG = Intake.Consts.armFF

        return kG * cos(self.getArmAngle())

    def initialize(self) -> None:
        """
        This function is called once when the subsystem is initialized.
        """
        # reset the arm encoders
        self.leftArm.getEncoder().setPosition(0)
        self.rightArm.getEncoder().setPosition(0)

        # reset the arm motors
        self.targetArmAngle: ArmAngle = Intake.Consts.default
        self.targetState: int = Intake.States.default

        # set the initial target position for the arm motors
        self.leftArmController.setReference(self.targetArmAngle.leftRot, SparkBase.ControlType.kPosition, self.slot)
        self.rightArmController.setReference(self.targetArmAngle.rightRot, SparkBase.ControlType.kPosition, self.slot)
        self.intakeMotor.set(Intake.Consts.intakeSpeed)

        self.flipState = False
        
    def periodic(self) -> None:
        """Updates every robot loop (~50hz)"""
        # update the motors to the target position
        self.updateMotors()

    def updateMotors(self) -> None:
        # Calculate and apply FF based on current angle
        feedForward: float = self.calculateFF()

        # Set the target position for the arm motors
        # If the flip state is active, flip the end effector angle
        target: ArmAngle

        if self.flipState:
            if self.targetState in (
                Intake.States.scoreCoralL1, 
                Intake.States.scoreCoralL2, 
                Intake.States.scoreCoralL3, 
                Intake.States.scoreCoralL4, 
                Intake.States.scoringCoralL1, 
                Intake.States.scoringCoralL2, 
                Intake.States.scoringCoralL3, 
                Intake.States.scoringCoralL4
            ):
                # Flip the end effector angle for scoring coral
                target = self.targetArmAngle.withEndEffectorAngle(-self.targetArmAngle.endEffectorAngle)
            else:
                # Set the flipState to false
                self.flipState = False

                # Don't flip the end effector angle for other states
                target = self.targetArmAngle.withEndEffectorAngle(self.targetArmAngle.endEffectorAngle)
        else:
            target = self.targetArmAngle

        self.leftArmController.setReference(
            target.leftRot, 
            SparkBase.ControlType.kPosition, 
            self.slot,
            arbFeedforward=feedForward
        )
        self.rightArmController.setReference(
            target.rightRot, 
            SparkBase.ControlType.kPosition, 
            self.slot,
            arbFeedforward=feedForward
        )
    
    def moveTo(self, target: int) -> None:
        """
        Move the arm to the target state.
        """
        if target not in Intake.States.__dict__.values():
            raise ValueError(f"Invalid target state: {target}")

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
