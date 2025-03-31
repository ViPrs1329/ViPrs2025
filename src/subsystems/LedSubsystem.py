import commands2
import wpilib

class LED(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self.blinkin = wpilib.Spark(0)
    self.blinkin.set(0.93)
    self.cooldown = 500
  
  def periodic(self):
    if self.cooldown > 0:
      self.cooldown -= 1
    else:
      self.setColor('white')

  def setColor(self, color):
    "sets the color to the color specified in led.md"
    if self.cooldown <= 0:
      match color:
        case "red":
          self.blinkin.set(0.61)
        case "orange":
          self.blinkin.set(0.65)
        case "yellow":
          self.blinkin.set(0.69)
        case "lime":
          self.blinkin.set(0.73)
        case "green":
          self.blinkin.set(0.77)
        case "aqua":
          self.blinkin.set(0.81)
        case "violet":
          self.blinkin.set(0.91)
        case "white":
          self.blinkin.set(0.93)
        case "black":
          self.blinkin.set(0.99)
        case "pink":
          self.blinkin.set(0.57)
        case _:
          self.blinkin.set(0.99)
      self.cooldown = 500