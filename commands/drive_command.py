from commands2.command import Command
from wpilib import XboxController
from subsystems.drive_subsystem import DriveSubsystem
from constants.constants import DriveConstants

class DriveCommand(Command):
    """
    A command to drive the robot with joystick input.
    """
    
    def __init__(
        self,
        drive_subsystem: DriveSubsystem,
        controller: XboxController,
        precision_button: int = XboxController.Button.kLeftBumper,
        boost_button: int = XboxController.Button.kRightBumper,
        field_relative_toggle: int = XboxController.Button.kStart
    ):
        """
        Creates a new DriveCommand.
        
        :param drive_subsystem: The drive subsystem to use
        :param controller: The controller to use
        :param precision_button: The button that enables precision mode
        :param boost_button: The button that enables boost mode
        :param field_relative_toggle: The button that toggles field-relative control
        """
        super().__init__()
        
        self.drive = drive_subsystem
        self.controller = controller
        self.precision_button = precision_button
        self.boost_button = boost_button
        self.field_relative_toggle = field_relative_toggle
        
        self.addRequirements(drive_subsystem)
        
        # Initialize button states
        self.field_relative_pressed = False
    
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
        if abs(x_speed) < 0.1:
            x_speed = 0
        if abs(y_speed) < 0.1:
            y_speed = 0
        if abs(rot) < 0.1:
            rot = 0
        
        # Check speed mode buttons
        if self.controller.getRawButton(self.precision_button):
            self.drive.set_speed_mode(DriveConstants.PRECISION_SPEED_MULTIPLIER)
        elif self.controller.getRawButton(self.boost_button):
            self.drive.set_speed_mode(DriveConstants.BOOST_SPEED_MULTIPLIER)
        else:
            self.drive.set_speed_mode(DriveConstants.NORMAL_SPEED_MULTIPLIER)
        
        # Check field relative toggle
        if (self.controller.getRawButton(self.field_relative_toggle) 
            and not self.field_relative_pressed):
            self.drive.toggle_field_relative()
        self.field_relative_pressed = self.controller.getRawButton(self.field_relative_toggle)
        
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