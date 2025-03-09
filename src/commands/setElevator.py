import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
# Can't import RobotContainer

import constants

class SetElevator(commands2.Command):
  def __init__(self, dir: str, Elev: Elevator, EE: EndEffector):
    super().__init__()
    self.dir = dir # "up" or "down"
    # self.buttons = pressed
    self.elevator = Elev
    self.EE = EE

  def initialize(self):
    print("in")
    elevatorLevel : int = self.getElevatorLevel()
    self.elevator.gotoPosition(constants.convert.in2rot(constants.reefConsts.reefLevels[elevatorLevel][1] + constants.elevatorConsts.verticalOffset))
    self.EE.setAlgaeArmAngle(constants.intakeConsts.algaeArmAngles[elevatorLevel])
  
  def decreaseLevel(self):
    print("down")
    if self.elevator.currentLevel <= 0:
      self.elevator.currentLevel = 0
    else:
      self.elevator.currentLevel -= 1
    return self.elevator.currentLevel

  def increaseLevel(self):
    print("up")
    if self.elevator.currentLevel >= 4:
      self.elevator.currentLevel = 4
    else:
      self.elevator.currentLevel += 1
    return self.elevator.currentLevel

  def getElevatorLevel(self):
    match self.dir:
      case "up":
        return self.increaseLevel()
      case "down":
        return self.decreaseLevel()
      case _:
        raise ValueError("Direction has to be 'up' or 'down'")
      
    # print(*self.buttons)
    # "return values are: {ground: 0, L1: 1, L2: 2, L3: 3, L4: 4}"
    # if pressedButtons[2] == True and pressedButtons[3] == False:
    #   return self.decreaseLevel()
    # elif pressedButtons[2] == False and pressedButtons[3] == True:
    #   return self.increaseLevel()  
    # return self.elevator.currentLevel

  def gotoLevel(self):
    #constants.reefConsts.reefLevels[level][0] is the maximum height of a branch
    level = self.elevator.currentLevel
    self.elevator.gotoPosition(constants.convert.in2rot(constants.reefConsts.reefLevels[level][1] + constants.elevatorConsts.verticalOffset) / 2)

  def execute(self):
    pass
  
  def isFinished(self) -> bool:
    return True