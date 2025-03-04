from commands2.command import Command
from subsystems.end_effector import EndEffector
from constants.constants import CoralManipulatorConstants

class CoralIntakeCommand(Command):
    """
    A command to handle the Coral intake sequence.
    The sequence is:
    1. Run intake wheels until intake sensor is triggered
    2. When intake sensor is triggered, keep running slowly until both:
       - Outlet sensor is triggered
       - Intake sensor is not triggered
    """
    
    def __init__(self, end_effector: EndEffector):
        """
        Create a new CoralIntakeCommand.
        
        :param end_effector: The EndEffector subsystem to use
        """
        super().__init__()
        
        self.end_effector = end_effector
        self.addRequirements(end_effector)
        
        # State variables
        self.intake_sensor_triggered = False
        self.outlet_sensor_triggered = False
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.intake_sensor_triggered = False
        self.outlet_sensor_triggered = False
        self.end_effector.set_coral_intaking(True)
        self.end_effector.set_coral_fully_intaken(False)
    
    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        # Get current sensor states
        intake_sensor = self.end_effector.get_coral_intake_sensor()
        outlet_sensor = self.end_effector.get_coral_outlet_sensor()
        
        # Update state variables
        if intake_sensor and not self.intake_sensor_triggered:
            self.intake_sensor_triggered = True
        
        if outlet_sensor and not self.outlet_sensor_triggered:
            self.outlet_sensor_triggered = True
        
        # Set intake speed based on state
        if not self.intake_sensor_triggered:
            # Full speed until intake sensor is triggered
            self.end_effector.set_coral_intake_speed(CoralManipulatorConstants.INTAKE_SPEED)
        elif not self.outlet_sensor_triggered or intake_sensor:
            # Slow speed until fully intaken
            self.end_effector.set_coral_intake_speed(CoralManipulatorConstants.INTAKE_SPEED * 0.3)
        else:
            # Stop when fully intaken
            self.end_effector.set_coral_intake_speed(0)
            self.end_effector.set_coral_fully_intaken(True)
    
    def end(self, interrupted: bool):
        """
        Called once the command ends or is interrupted.
        
        :param interrupted: whether the command was interrupted
        """
        self.end_effector.set_coral_intake_speed(0)
        self.end_effector.set_coral_intaking(False)
    
    def isFinished(self) -> bool:
        """
        Returns true when the command should end.
        
        :return: whether the command should end
        """
        return self.end_effector.is_coral_fully_intaken() 