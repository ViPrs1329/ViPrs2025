import commands2
import math
from subsystems.EndEffector import EndEffector

class AlgaeArmCyclePositions(commands2.Command):
    """
    Command that cycles through preset algae arm positions with each button press.
    Positions are now defined in radians instead of degrees.
    """
    
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.EE = endEffector
        self.addRequirements(endEffector)
        
        # Define the preset positions in radians
        # Previous values: [0, 30, 45, 90, 120] degrees
        self.positions = [
            0,                # 0 deg - straight down
            math.pi/4,        # 45 deg - down and forward
            math.pi/2,        # 90 deg - horizontal
            3*math.pi/4,      # 135 deg - up and forward
            math.pi           # 180 deg -straight up
        ]
        
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
        
        # Print debug information - convert to degrees for more readable output
        print(f"Moving algae arm to position: {math.degrees(target_position):.1f}° (index {AlgaeArmCyclePositions.current_index})")
    
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