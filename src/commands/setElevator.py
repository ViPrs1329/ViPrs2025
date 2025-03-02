import commands2
import wpilib
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
# Can't import RobotContainer

import constants

def getElevatorLevel(pressedButtons: list[bool]):
  "return values are: {ground: 0, L1: 1, L2: 2, L3: 3, L4: 4}"
  match pressedButtons:
    case [False, False, False, True]:
      return 1 # L1
    case [False, False, True, False]:
      return 1 # L1
    case [False, True, False, False]:
      return 1 # L1
    case [True, False, False, False]:
      return 1 # L1
    case [True, True, False, False]:
      return 2 # L2
    case [True, False, True, False]:
      return 0 # base
    case [True, False, False, True]:
      return 2 # L2
    case [False, True, True, False]:
      return 0 # base
    case [False, True, False, True]:
      return 2 # L2
    case [False, False, True, True]:
      return 2 # L2
    case [True, True, True, False]:
      return 3 # L3
    case [True, True, False, True]:
      return 3 # L3
    case [True, False, True, True]:
      return 3 # L3
    case [False, True, True, True]:
      return 3 # L3
    case [True, True, True, True]:
      return 4 # L4

class SetElevator(commands2.Command):
  def __init__(self, pressed: list[bool], elevatorSubsystem: Elevator, endEffectorSubsystem: EndEffector):
    super().__init__()
    self.buttons = pressed
    self.elevator = elevatorSubsystem
    self.EE = endEffectorSubsystem

  def initialize(self):
    elevatorLevel : int = getElevatorLevel(self.buttons)
    self.elevator.elevatorPID.setSetpoint(constants.elevatorConsts.elevatorHeights[elevatorLevel])
    self.EE.setAlgaeArmAngle(constants.intakeConsts.algaeArmAngles[elevatorLevel])
  
  def execute(self):
    pass
        
  def isFinished(self) -> bool:
    return True