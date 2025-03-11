import commands2
import wpilib

class ToggleDebugMode(commands2.Command):
    """
    Command that toggles the debug mode flag when executed.
    This is a one-shot command that completes immediately after toggling the flag.
    """
    def __init__(self, debug_mode_ref):
        """
        Initialize the toggle debug mode command.
        
        Args:
            debug_mode_ref: A list containing a single boolean value that represents
                           the current debug mode state [is_debug_mode]
        """
        super().__init__()
        self.debug_mode_ref = debug_mode_ref
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Toggle the debug mode
        self.debug_mode_ref[0] = not self.debug_mode_ref[0]
        mode_str = "ENABLED" if self.debug_mode_ref[0] else "DISABLED"
        print(f"Debug mode {mode_str}")
        
        # Give haptic feedback on the controller
        if wpilib.DriverStation.isJoystickConnected(0):
            controller = wpilib.XboxController(0)
            feedback_intensity = 0.5
            controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, feedback_intensity)
            # The rumble will be reset in end()
            
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        pass
        
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Turn off rumble
        if wpilib.DriverStation.isJoystickConnected(0):
            controller = wpilib.XboxController(0)
            controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)
        
    def isFinished(self):
        """Returns true when the command should end."""
        return True  # This is a one-shot command