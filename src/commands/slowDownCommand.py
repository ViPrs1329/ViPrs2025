import commands2
import wpilib
import constants

class ToggleSlow(commands2.Command):
  def __init__(self, scaleFactor: float):
    super.__init__()
    self.scaleFactor = constants.driveConsts.slowScale

  def initialize(self):
    self.scaleFactor = constants.driveConsts.slowScale

  def end(self):
    self.scaleFactor = 1