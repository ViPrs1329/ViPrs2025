import commands2
import wpilib
# Can't import RobotContainer

import constants

class Slow(commands2.Command):
    def __init__(self, inputScalerValue):  #
        super().__init__()
        self.s = inputScalerValue

    def initialize(self):
        self.s = constants.controller.slowDriveScale

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.s = 1
        
    def isFinished(self) -> bool:
        return False