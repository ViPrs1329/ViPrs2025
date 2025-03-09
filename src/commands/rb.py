import commands2
import wpilib
# Can't import RobotContainer

import constants

class RB(commands2.Command):
    def __init__(self, pressed):
        super().__init__()
        self.buttons = pressed

    def initialize(self):
        self.buttons[3] = True

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.buttons[3] = False
        
    def isFinished(self) -> bool:
        return False