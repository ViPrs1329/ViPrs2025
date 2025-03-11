# MoveAlgaeArmToPosition.py
import commands2
import math
from subsystems.EndEffector import EndEffector

class MoveAlgaeArmToPosition(commands2.Command):
    """
    Command that moves the algae arm to a specific position in radians.
    """
    
    def __init__(self, endEffector: EndEffector, position_radians: float):
        super().__init__()
        self.EE = endEffector
        self.position = position_radians
        self.addRequirements(endEffector)
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.EE.algaeDestination = self.position
        # Convert to degrees for more readable debug output
        print(f"Moving algae arm to position: {math.degrees(self.position):.1f}°")
    
    def execute(self):
        """Called repeatedly while the command is scheduled."""
        # The periodic method in EndEffector handles the actual movement
        pass
    
    def end(self, interrupted: bool):
        """Called when the command ends or is interrupted."""
        if interrupted:
            print("Algae arm movement was interrupted")
    
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # End immediately after setting the new position
        return True