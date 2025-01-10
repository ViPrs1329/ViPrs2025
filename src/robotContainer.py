
import commands2.button
from commands2 import SequentialCommandGroup
from commands2.button import CommandXboxController
from subsystems.SwerveDriveSubsystem import DriveTrain

class RobotContainer:
    """
    This class is where the bulk of the robot's resources are declared. Here, subsystems
    are instantiated and commands and button bindings are configured.
    """
    def __init__(self) -> None:
        self.drivetrain = DriveTrain()
        self.initSubsystems()
        self.initControls()
        self.initCommands()
        self.configureButtonBindings()

    def initSubsystems(self):
        """Instantiate the robot's subsystems."""
        
        pass

    def initControls(self):
        """Instantiate the robot's control objects"""
        
        pass

    def initCommands(self):
        """Instantiate the robot's commands."""
        
        pass

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
               
        pass

    def updateHardware(self):
        """Call the update methods of each subsystem."""
        pass

    def cacheSensors(self):
        """Retrieve and cache sensor data from each subsystem."""
        pass
