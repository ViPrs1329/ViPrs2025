import commands2
import wpilib
# Can't import RobotContainer
from subsystems.EndEffector import EndEffector
import constants

class AlgaeIntake(commands2.Command):
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.EE = endEffector

    def initialize(self):
        self.EE.startAlgaeIntake()

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.EE.stopAlgaeIntake()
        
    def isFinished(self) -> bool:
        return self.EE.algae_intake_motor.getOutputCurrent() > constants.intakeConsts.algaeThresholdCurrent