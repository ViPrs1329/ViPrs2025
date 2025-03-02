import commands2
import wpilib
# Can't import RobotContainer

import constants

def getElevatorLevel(pressedButtons):
  match pressedButtons:
    case [False, False, False, True]:
      pass # L1
    case [False, False, True, False]:
      pass # L1
    case [False, True, False, False]:
      pass # L1
    case [True, False, False, False]:
      pass # L1
    case [True, True, False, False]:
      pass # L2
    case [True, False, True, False]:
      pass # base
    case [True, False, False, True]:
      pass # L2
    case [False, True, True, False]:
      pass # base
    case [False, True, False, True]:
      pass # L2
    case [False, False, True, True]:
      pass # L2
    case [True, True, True, False]:
      pass # L3
    case [True, True, False, True]:
      pass # L3
    case [True, False, True, True]:
      pass # L3
    case [False, True, True, True]:
      pass # L3
    case [True, True, True, True]:
      pass # L4
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