import commands2
from subsystems.EndEffector import EndEffector

class AlgaeArmCyclePositions(commands2.Command):
    """
    Command that cycles through preset algae arm positions with each button press.
    """
    
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.EE = endEffector
        self.addRequirements(endEffector)
        
        # Define the preset positions in degrees (0° = straight down)
        self.positions = [0, 30, 45, 90, 120]
        
        # Initialize position index
        # Store this as a class variable so it persists between command instances
        if not hasattr(AlgaeArmCyclePositions, 'current_index'):
            AlgaeArmCyclePositions.current_index = 0
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Move to the next position in the cycle
        AlgaeArmCyclePositions.current_index = (AlgaeArmCyclePositions.current_index + 1) % len(self.positions)
        target_position = self.positions[AlgaeArmCyclePositions.current_index]
        
        # Set the arm to the new target position
        self.EE.destination = target_position
        
        # Print debug information
        print(f"Moving algae arm to position: {target_position}° (index {AlgaeArmCyclePositions.current_index})")
    
    def execute(self):
        """Called repeatedly while the command is scheduled."""
        # The periodic method in EndEffector handles the actual movement
        pass
    
    def end(self, interrupted: bool):
        """Called when the command ends or is interrupted."""
        if interrupted:
            print("Algae arm position cycling was interrupted")
    
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # End immediately after setting the new position
        return True