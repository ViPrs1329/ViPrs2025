# In a new file called commands/IntakeCommands.py
import commands2
from subsystems.EndEffector import EndEffector

class IntakeCoralCommand(commands2.CommandBase):
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.endEffector = endEffector
        self.addRequirements(endEffector)
        
    def execute(self):
        return self.endEffector.intakeCoral()
        
    def isFinished(self):
        return self.endEffector.isCoralPositioned()
        
    def end(self, interrupted):
        self.endEffector.stopCoralIntake()