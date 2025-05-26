from subsystems.ElevatorSubsystem import ElevatorSubsystem
from subsystems.IntakeSubsystem import IntakeSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
import constants as Consts
from commands2 import InstantCommand
from commands2 import SequentialCommandGroup
from commands2 import ParallelCommandGroup

class SubsystemWrapper:
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem, drivetrain: DriveSubsystem, limelight):
        self.elevator = elevator
        self.intake = intake
        self.drivetrain = drivetrain
        self.limelight = limelight

        self.groundIntakeAlgaeCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.groundIntakeAlgae),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.groundIntakeAlgae),
                self.intake
            )
        )

        self.l2IntakeAlgaeCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.l2IntakeAlgae),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.l2IntakeAlgae),
                self.intake
            )
        )

        self.l3IntakeAlgaeCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.l3IntakeAlgae),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.l3IntakeAlgae),
                self.intake
            )
        )

        self.groundIntakeCoralCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.groundIntakeCoral),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.groundIntakeCoral),
                self.intake
            )
        )

        self.feederIntakeCoralCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.feederIntakeCoral),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.feederIntakeCoral),
                self.intake
            )
        )

        self.scoreAlgaeNetCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreAlgaeNet),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreAlgaeNet),
                self.intake
            )
        )

        self.scoreAlgaeProcessorCommand = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreAlgaeProcessor),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreAlgaeProcessor),
                self.intake
            )
        )

        self.scoreCoralL1Command = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreCoralL1),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreCoralL1),
                self.intake
            )
        )

        self.scoreCoralL2Command = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreCoralL2),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreCoralL2),
                self.intake
            )
        )

        self.scoreCoralL3Command = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreCoralL3),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreCoralL3),
                self.intake
            )
        )

        self.scoreCoralL4Command = InstantCommand(
            lambda: self.elevator.moveTo(Consts.Elevator.States.scoreCoralL4),
            self.elevator
        ).alongWith(
            InstantCommand(
                lambda: self.intake.moveTo(Consts.Intake.States.scoreCoralL4),
                self.intake
            )
        )

        self.scoreCoralCommand: ParallelCommandGroup = ParallelCommandGroup(
            self.intake.scoreCoralCommand,
            self.elevator.scoreCoralCommand,
            self.drivetrain.scoreCoralCommand
        )

        self.scoreAlgaeCommand: ParallelCommandGroup = ParallelCommandGroup(
            self.intake.scoreAlgaeCommand,
            self.elevator.scoreAlgaeCommand,
            self.drivetrain.scoreAlgaeCommand
        )

    def groundIntakeAlgae(self):
        """
        Move the elevator and intake to the ground intake algae position.
        """
        self.groundIntakeAlgaeCommand.schedule()

    def l2IntakeAlgae(self):
        """
        Move the elevator and intake to the L2 intake algae position.
        """
        self.l2IntakeAlgaeCommand.schedule()

    def l3IntakeAlgae(self):
        """
        Move the elevator and intake to the L3 intake algae position.
        """
        self.l3IntakeAlgaeCommand.schedule()
    
    def groundIntakeCoral(self):
        """
        Move the elevator and intake to the ground intake coral position.
        """
        self.groundIntakeCoralCommand.schedule()

    def feederIntakeCoral(self):
        """
        Move the elevator and intake to the feeder intake coral position.
        """
        self.feederIntakeCoralCommand.schedule()

    def scoreAlgaeNet(self):
        """
        Move the elevator and intake to the score algae net position.
        """
        self.scoreAlgaeNetCommand.schedule()

    def scoreAlgaeProcessor(self):
        """
        Move the elevator and intake to the score algae processor position.
        """
        self.scoreAlgaeProcessorCommand.schedule()

    def scoreCoralL1(self):
        """
        Move the elevator and intake to the score coral L1 position.
        """
        self.scoreCoralL1Command.schedule()
    
    def scoreCoralL2(self):
        """
        Move the elevator and intake to the score coral L2 position.
        """
        self.scoreCoralL2Command.schedule()

    def scoreCoralL3(self):
        """
        Move the elevator and intake to the score coral L3 position.
        """
        self.scoreCoralL3Command.schedule()

    def scoreCoralL4(self):
        """
        Move the elevator and intake to the score coral L4 position.
        """
        self.scoreCoralL4Command.schedule()

    def scoreCoral(self):
        """
        Score the coral onto the branch
        """
        self.scoreCoralCommand.schedule()
        
    def scoreAlgae(self):
        """
        Score the algae onto the net and processor
        """
        self.scoreAlgaeCommand.schedule()
