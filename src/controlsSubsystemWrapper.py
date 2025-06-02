from subsystems.ElevatorSubsystem import ElevatorSubsystem
from subsystems.IntakeSubsystem import IntakeSubsystem
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
import constants as Consts
from commands2 import InstantCommand
from commands2 import SequentialCommandGroup
from commands2 import ParallelCommandGroup
from commands2 import Subsystem
from ntcore import NetworkTableInstance

from wpimath.geometry import Pose2d

class SubsystemWrapper(Subsystem):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem, drivetrain: DriveSubsystem, limelight: LimelightSubsystem):
        """
        Wrapper class that coordinates multiple subsystems to perform complex robot actions.

        This class provides a simplified interface for common robot operations by combining
        movements from multiple subsystems into single method calls.
        """        
        
        self.elevator = elevator
        self.intake = intake
        self.drivetrain = drivetrain
        self.limelight = limelight

        self.resetBeforeTeleopCommand = SequentialCommandGroup(
            # Safety first - stop all motion
            InstantCommand(
                lambda: self.drivetrain.stopDrive(),
                self.drivetrain
            ),
            # Move mechanisms to safe starting positions
            InstantCommand(
                lambda: self.elevator.moveTo(Consts.Elevator.States.default),
                self.elevator
            ).alongWith(
                InstantCommand(
                    lambda: self.intake.moveTo(Consts.Intake.States.default),
                    self.intake
                )
            ),
            # Configure intake for teleop operation
            InstantCommand(
                lambda: self.intake.intakeMotor.set(Consts.Intake.Consts.intakeSpeed),
                self.intake
            )
        )

        self.resetSubsystemsCommand = SequentialCommandGroup(
            # Stop all motion first for safety
            InstantCommand(
                lambda: self.drivetrain.stopDrive(),
                self.drivetrain
            ),
            # Reset intake roller to default speed
            InstantCommand(
                lambda: self.intake.intakeMotor.set(Consts.Intake.Consts.intakeSpeed),
                self.intake
            ),
            # Zero encoders
            InstantCommand(
                lambda: self.elevator.initialize(),
                self.elevator
            ).alongWith(
                InstantCommand(
                    lambda: self.intake.initialize(),
                    self.intake
                )
            ),
            # Zero the gyro last (after motion has stopped)
            InstantCommand(
                lambda: self.drivetrain.rezeroGyro(),
                self.drivetrain
            ),
            # Move to default positions after zeroing
            InstantCommand(
                lambda: self.elevator.moveTo(Consts.Elevator.States.default),
                self.elevator
            ).alongWith(
                InstantCommand(
                    lambda: self.intake.moveTo(Consts.Intake.States.default),
                    self.intake
                )
            )
        )

        self.flipEndEffectorCommand = intake.flipCommand

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

        # Initialize NetworkTables logging
        self.nt = NetworkTableInstance.getDefault()
        self.logTable = self.nt.getTable("SubsystemWrapper")
        
        # Create persistent publishers
        self.elevatorHeightPub = self.logTable.getDoubleTopic("elevator/height").publish()
        self.elevatorStatePub = self.logTable.getStringTopic("elevator/state").publish()
        self.intakeArmAnglePub = self.logTable.getDoubleTopic("intake/armAngle").publish()
        self.intakeEndEffectorPub = self.logTable.getDoubleTopic("intake/endEffector").publish()
        self.intakeFlipStatePub = self.logTable.getBooleanTopic("intake/flipState").publish()
        self.driveRotationPub = self.logTable.getStructTopic("drive/odometry", Pose2d).publish()
        self.targetVisiblePub = self.logTable.getBooleanTopic("limelight/targetVisible").publish()
        
    def updateNetworkTables(self) -> None:
        """Update NetworkTables with current subsystem states"""
        # Elevator logging
        self.elevatorHeightPub.set(self.elevator.targetHeight)
        self.elevatorStatePub.set(str(self.elevator.targetState))
        
        # Intake logging
        self.intakeArmAnglePub.set(self.intake.getArmAngle())
        self.intakeEndEffectorPub.set(self.intake.getEndEffectorAngle())
        self.intakeFlipStatePub.set(self.intake.flipState)
        
        # Drivetrain logging
        self.driveRotationPub.set(self.drivetrain.getPose())
        
        # Limelight logging
        self.targetVisiblePub.set(self.limelight.canSeeTarget())
        
        # Log command states
        commandsTable = self.logTable.getSubTable("commands")
        commandsTable.putBoolean("resetCommand", self.resetSubsystemsCommand.isScheduled())
        commandsTable.putBoolean("scoreCoralCommand", self.scoreCoralCommand.isScheduled())
        commandsTable.putBoolean("scoreAlgaeCommand", self.scoreAlgaeCommand.isScheduled())

    def periodic(self) -> None:
        """Called periodically, use for updating NetworkTables"""
        self.updateNetworkTables()

    def resetSubsystems(self) -> None:
        """
        Reset all subsystems to their default states.
        This includes stopping all motion, zeroing encoders, and moving to default positions.
        """
        self.resetSubsystemsCommand.schedule()

    def flipEndEffector(self) -> None:
        """
        Flip the end effector of the intake.
        """
        self.flipEndEffectorCommand.schedule()

    def groundIntakeAlgae(self) -> None:
        """
        Move the elevator and intake to the ground intake algae position.
        """
        self.groundIntakeAlgaeCommand.schedule()

    def l2IntakeAlgae(self) -> None:
        """
        Move the elevator and intake to the L2 intake algae position.
        """
        self.l2IntakeAlgaeCommand.schedule()

    def l3IntakeAlgae(self) -> None:
        """
        Move the elevator and intake to the L3 intake algae position.
        """
        self.l3IntakeAlgaeCommand.schedule()
    
    def groundIntakeCoral(self) -> None:
        """
        Move the elevator and intake to the ground intake coral position.
        """
        self.groundIntakeCoralCommand.schedule()

    def feederIntakeCoral(self) -> None:
        """
        Move the elevator and intake to the feeder intake coral position.
        """
        self.feederIntakeCoralCommand.schedule()

    def scoreAlgaeNet(self) -> None:
        """
        Move the elevator and intake to the score algae net position.
        """
        self.scoreAlgaeNetCommand.schedule()

    def scoreAlgaeProcessor(self) -> None:
        """
        Move the elevator and intake to the score algae processor position.
        """
        self.scoreAlgaeProcessorCommand.schedule()

    def scoreCoralL1(self) -> None:
        """
        Move the elevator and intake to the score coral L1 position.
        """
        self.scoreCoralL1Command.schedule()
    
    def scoreCoralL2(self) -> None:
        """
        Move the elevator and intake to the score coral L2 position.
        """
        self.scoreCoralL2Command.schedule()

    def scoreCoralL3(self) -> None:
        """
        Move the elevator and intake to the score coral L3 position.
        """
        self.scoreCoralL3Command.schedule()

    def scoreCoralL4(self) -> None:
        """
        Move the elevator and intake to the score coral L4 position.
        """
        self.scoreCoralL4Command.schedule()

    def scoreCoral(self) -> None:
        """
        Score the coral onto the branch
        """
        self.scoreCoralCommand.schedule()
        
    def scoreAlgae(self) -> None:
        """
        Score the algae onto the net and processor
        """
        self.scoreAlgaeCommand.schedule()

    def resetBeforeTeleop(self) -> None:
        """
        Prepare robot for teleop operation.
        Moves to safe positions without zeroing sensors.
        """
        self.resetBeforeTeleopCommand.schedule()
