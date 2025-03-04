from commands2.command import Command
from subsystems.end_effector import EndEffector
from constants.constants import AlgaeManipulatorConstants

class AlgaeManipulateCommand(Command):
    """
    A command to handle the Algae manipulator operations.
    This command can:
    1. Move the arm to specific positions
    2. Run the intake at specific speeds
    """
    
    def __init__(
        self,
        end_effector: EndEffector,
        target_position: float = None,
        intake_speed: float = None
    ):
        """
        Create a new AlgaeManipulateCommand.
        
        :param end_effector: The EndEffector subsystem to use
        :param target_position: Target position for the arm in radians (optional)
        :param intake_speed: Speed for the intake from -1 to 1 (optional)
        """
        super().__init__()
        
        self.end_effector = end_effector
        self.target_position = target_position
        self.intake_speed = intake_speed
        
        self.addRequirements(end_effector)
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        if self.target_position is not None:
            self.end_effector.set_algae_arm_position(self.target_position)
        if self.intake_speed is not None:
            self.end_effector.set_algae_intake_speed(self.intake_speed)
    
    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        # No continuous updates needed
        pass
    
    def end(self, interrupted: bool):
        """
        Called once the command ends or is interrupted.
        
        :param interrupted: whether the command was interrupted
        """
        if interrupted:
            self.end_effector.stop_all()
    
    def isFinished(self) -> bool:
        """
        Returns true when the command should end.
        
        :return: whether the command should end
        """
        if self.target_position is not None:
            # Check if we're close enough to the target position
            current_position = self.end_effector.get_algae_arm_position()
            return abs(current_position - self.target_position) < 0.1  # 0.1 rad tolerance
        return True  # If no position specified, command ends immediately 