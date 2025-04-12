import commands2, constants
import wpilib
import ntcore

class LED(commands2.Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self.blinkin = wpilib.Spark(0)
    self.blinkin.set(0.93)
    self.cooldown = 500
    self.state = constants.RobotStates.noTag

    self.table = ntcore.NetworkTableInstance.getDefault()
    self.statePub = self.table.getStringTopic("State").publish()
  
  def periodic(self):
    match self.state:
      case constants.RobotStates.noTag:
        self.statePub.set("No Tag")
        self.setColor("red")
      case constants.RobotStates.Tag:
        self.statePub.set("Tag")
        self.setColor("aqua")
      case constants.RobotStates.aligning:
        self.statePub.set("Aligning")
        self.setColor("yellow")
      case constants.RobotStates.aligned:
        self.statePub.set("Aligned")
        self.setColor("green")

  def changeStates(self, tostate):
    if tostate == constants.RobotStates.noTag:
      self.state = constants.RobotStates.noTag

    if tostate == constants.RobotStates.Tag:
      if self.state == constants.RobotStates.noTag:
        self.state = constants.RobotStates.Tag

    if tostate == constants.RobotStates.aligning:
      if self.state == constants.RobotStates.Tag or self.state == constants.RobotStates.aligned:
        self.state = constants.RobotStates.aligning
    
    if tostate == constants.RobotStates.aligned:
      if self.state == constants.RobotStates.aligning or self.state == constants.RobotStates.Tag:
        self.state = constants.RobotStates.aligned

  def setColor(self, color):
    "sets the color to the color specified in led.md"
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