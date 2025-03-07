import commands2

class WaitUntilCoralIsDetected(commands2.WaitUntilCommand):
  def __init__(self, isInRange):
    super().__init__(isInRange)
    self.variable_supplier = isInRange # Store the supplier

  def isFinished(self):
      return self.variable_supplier() # Check the condition