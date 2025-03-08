import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
# from subsystems.EndEffector import EndEffector
# Can't import RobotContainer

import constants

class SetElevator(commands2.Command):
  def __init__(self, pressed: list[bool], Elev: Elevator, EE):
    super().__init__()
    self.buttons = pressed
    self.elevator = Elev
    self.EE = EE

  def initialize(self):
    elevatorLevel : int = self.getself.elevatorLevel(self.buttons)
    self.elevator.elevatorPID.setSetpoint(constants.elevatorConsts.elevatorHeights[elevatorLevel])
    self.EE.setAlgaeArmAngle(constants.intakeConsts.algaeArmAngles[elevatorLevel])
  
  def decreaseLevel(self):
    if self.elevator.currentLevel <= 0:
      self.elevator.currentLevel = 0
    else:
      self.elevator.currentLevel -= 1
    return self.elevator.currentLevel

  def increaseLevel(self):
    if self.elevator.currentLevel >= 4:
      self.elevator.currentLevel = 4
    else:
      self.elevator.currentLevel += 1
    return self.elevator.currentLevel

  def getElevatorLevel(self, pressedButtons: list[bool]):
    "return values are: {ground: 0, L1: 1, L2: 2, L3: 3, L4: 4}"
    if pressedButtons[2] == True and pressedButtons[3] == False:
      return self.decreaseLevel()
    elif pressedButtons[2] == False and pressedButtons[3] == True:
      return self.increaseLevel()  

  def gotoLevel(self):
    #constants.reefConsts.reefLevels[level][0] is the maximum height of a branch
    level = self.elevator.currentLevel
    self.elevator.gotoPosition(constants.reefConsts.reefLevels[level][0] + constants.elevatorConsts.verticaloffset)

  def execute(self):
    pass
  
  def isFinished(self) -> bool:
    return True