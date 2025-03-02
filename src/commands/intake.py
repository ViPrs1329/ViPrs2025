import commands2
import wpilib
# Can't import RobotContainer
from subsystems.EndEffector import EndEffector
import constants

class Intake(commands2.Command):
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.EE = endEffector

    def initialize(self):
        self.EE.stopCoralMotors()

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.EE.startCoralMotors()
        
    def isFinished(self) -> bool:
        return True