import commands2
import wpilib
# Can't import RobotContainer

import constants

class LT(commands2.Command):
    def __init__(self, pressed):
        super().__init__()
        self.buttons = pressed

    def initialize(self):
        self.buttons[0] = True

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.buttons[0] = False
        
    def isFinished(self) -> bool:
        return False