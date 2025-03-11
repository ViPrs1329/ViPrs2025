import commands2
import wpilib
from subsystems.EndEffector import EndEffector
import constants

class AlgaeIntakeControl(commands2.Command):
    """
    Command to control the algae intake mechanism.
    This can be used for both intake and ejection by specifying the direction.
    """
    def __init__(self, endEffector: EndEffector, direction="intake"):
        super().__init__()
        self.EE = endEffector
        self.addRequirements(endEffector)
        self.direction = direction  # "intake" or "eject"
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        speed = constants.intakeConsts.algaeIntakeSpeed
        if self.direction == "eject":
            speed = -speed  # Reverse the motor for ejection
            
        self.EE.setAlgaeIntakeSpeed(speed)
        print(f"Algae {self.direction} started at speed: {speed}")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        # No action needed during execution - the motor keeps running
        pass
        
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        self.EE.setAlgaeIntakeSpeed(0)
        print(f"Algae {self.direction} stopped" + (" (interrupted)" if interrupted else ""))
        
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # This command runs until interrupted or canceled
        # For intake mode, could add a current-based detection
        if self.direction == "intake":
            current = self.EE.algae_intake_motor.getOutputCurrent()
            if current > constants.intakeConsts.algaeThresholdCurrent:
                print(f"Algae detected (current: {current}A)")
                return True
        return False