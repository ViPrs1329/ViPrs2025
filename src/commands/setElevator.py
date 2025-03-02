import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
# Can't import RobotContainer

import constants

def decreaseLevel():
  if Elevator.currentLevel == 0:
    pass
  else:
    Elevator.currentLevel -= 1
  return Elevator.currentLevel

def increaseLevel():
  if Elevator.currentLevel == 4:
    pass
  else:
    Elevator.currentLevel += 1
  return Elevator.currentLevel

def getElevatorLevel(pressedButtons: list[bool]):
  "return values are: {ground: 0, L1: 1, L2: 2, L3: 3, L4: 4}"
  if pressedButtons[2] == True and pressedButtons[3] == False:
    return decreaseLevel()
  elif pressedButtons[2] == False and pressedButtons[4] == True:
    return increaseLevel()

class SetElevator(commands2.Command):
  def __init__(self, pressed: list[bool], elevatorSubsystem: Elevator):
    super().__init__()
    self.buttons = pressed
    self.elevator = elevatorSubsystem

  def initialize(self):
    elevatorLevel : int = getElevatorLevel(self.buttons)
    self.elevator.elevatorPID.setSetpoint(constants.elevatorConsts.elevatorHeights[elevatorLevel])
    
  def execute(self):
    pass
  
  def isFinished(self) -> bool:
    return True