import commands2
import wpilib
# Can't import RobotContainer

import constants

def getElevatorLevel(pressedButtons):
    match pressedButtons:
        case [False, False, False, True]:
            pass
        case [False, False, True, False]:
            pass
        case [False, True, False, False]:
            pass
        case [True, False, False, False]:
            pass
    # TODO finish the match cases

class SetElevator(commands2.Command):
    def __init__(self, pressed):
        super().__init__()
        self.buttons = pressed

    def initialize(self):
        self.buttons[2] = True

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.buttons[2] = False
        
    def isFinished(self) -> bool:
        return True