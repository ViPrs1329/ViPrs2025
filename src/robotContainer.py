import commands2
from commands2.button import CommandXboxController, Trigger
from commands2 import button
from commands2 import SequentialCommandGroup

from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.EndEffector import EndEffector
from commands.IntakeCommands import IntakeCoralCommand

class RobotContainer:
    """
    This class is where the bulk of the robot's resources are declared. Here, subsystems
    are instantiated and commands and button bindings are configured.
    """
    def __init__(self) -> None:
        # Create controllers
        self.driverController = CommandXboxController(0)
        self.operatorController = CommandXboxController(1)
        
        # Initialize subsystems
        self.drivetrain = DriveTrain()
        self.endEffector = EndEffector()
        
        # Initialize commands
        self.initCommands()
        
        # Configure button bindings
        self.configureButtonBindings()

    def initCommands(self):
        """Instantiate the robot's commands."""
        # Create commands
        self.intakeCoralCommand = IntakeCoralCommand(self.endEffector)
        
        # Set default commands
        # No default commands needed at this time

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
        # Driver controls
        # Example: self.driverController.a().onTrue(commands2.InstantCommand(lambda: self.drivetrain.resetGyro()))
        
        # Operator controls - Intake Coral with the X button
        self.operatorController.x().onTrue(self.intakeCoralCommand)
        
        # Additional controls can be added here

    def getAutonomousCommand(self):
        """Return the command to run in autonomous mode."""
        # Return the auto command here
        return commands2.InstantCommand()  # Placeholder
        
    def updateHardware(self):
        """Call the update methods of each subsystem."""
        # Used for updating hardware state if needed
        pass

    def cacheSensors(self):
        """Retrieve and cache sensor data from each subsystem."""
        # Used for caching sensor data to minimize CAN bus traffic
        pass