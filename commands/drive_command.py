from commands2.command import Command
from wpilib import XboxController
from subsystems.drive_subsystem import DriveSubsystem
from constants.constants import DriveConstants, OIConstants

class DriveCommand(Command):
    """
    A command to drive the robot with joystick input.
    """
    
    def __init__(
        self,
        drive_subsystem: DriveSubsystem,
        controller: XboxController,
    ):
        """
        Creates a new DriveCommand.
        
        :param drive_subsystem: The drive subsystem to use
        :param controller: The controller to use
        """
        super().__init__()
        
        self.drive = drive_subsystem
        self.controller = controller
        
        self.addRequirements(drive_subsystem)
        
        # Initialize button states
        self.field_relative_pressed = False
        self.precision_pressed = False
        self.boost_pressed = False
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        pass
    
    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        # Get joystick inputs
        x_speed = -self.controller.getLeftY()  # Forward/backward
        y_speed = -self.controller.getLeftX()  # Left/right
        rot = -self.controller.getRightX()     # Rotation
        
        # Apply deadband
        if abs(x_speed) < OIConstants.DRIVE_DEADBAND:
            x_speed = 0
        if abs(y_speed) < OIConstants.DRIVE_DEADBAND:
            y_speed = 0
        if abs(rot) < OIConstants.DRIVE_DEADBAND:
            rot = 0
        
        # Check speed mode buttons
        if self.controller.getRawButton(OIConstants.PRECISION_MODE_BUTTON):
            self.drive.set_speed_mode(DriveConstants.PRECISION_SPEED_MULTIPLIER)
        elif self.controller.getRawButton(OIConstants.BOOST_MODE_BUTTON):
            self.drive.set_speed_mode(DriveConstants.BOOST_SPEED_MULTIPLIER)
        else:
            self.drive.set_speed_mode(DriveConstants.NORMAL_SPEED_MULTIPLIER)
        
        # Drive
        self.drive.drive(x_speed * 4.0, y_speed * 4.0, rot * 4.0)  # Scale to max speed of 4 m/s
    
    def end(self, interrupted: bool):
        """
        Called once the command ends or is interrupted.
        
        :param interrupted: whether the command was interrupted
        """
        self.drive.drive(0, 0, 0)
    
    def isFinished(self) -> bool:
        """
        Returns true when the command should end.
        
        :return: whether the command should end
        """
        return False  # Command never ends on its own 