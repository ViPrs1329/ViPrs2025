import commands2
import wpilib
from subsystems.EndEffector import EndEffector
import constants

class AlgaeIntakeToggle(commands2.Command):
    """
    Command to toggle the algae intake mechanism on/off.
    This can be used for both intake and ejection by specifying the direction.
    
    Unlike the standard command, this stays running until explicitly canceled
    by pressing the same button again.
    """
    def __init__(self, endEffector: EndEffector, direction="intake"):
        super().__init__()
        self.EE = endEffector
        self.addRequirements(endEffector)
        self.direction = direction  # "intake" or "eject"
        self.timer = wpilib.Timer()
        self.is_running = False
        self.high_current_reading_count = 0  # Used to filter noise in current readings
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        speed = constants.intakeConsts.algaeIntakeSpeed
        if self.direction == "eject":
            speed = -speed  # Reverse the motor for ejection
            
        self.EE.setAlgaeIntakeSpeed(speed)
        self.timer.reset()
        self.timer.start()
        self.is_running = True
        self.high_current_reading_count = 0
        print(f"Algae {self.direction} TOGGLED ON at speed: {speed}")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        # For intake mode, check for current-based detection
        if self.direction == "intake":
            current = self.EE.algae_intake_motor.getOutputCurrent()
            if current > constants.intakeConsts.algaeThresholdCurrent:
                self.high_current_reading_count += 1
                if self.high_current_reading_count >= 5:  # Require several readings above threshold
                    print(f"Algae detected (current: {current:.2f}A)")
                    print("Stopping intake automatically")
                    self.cancel()
            else:
                self.high_current_reading_count = 0
        
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        self.EE.setAlgaeIntakeSpeed(0)
        self.timer.stop()
        self.is_running = False
        if interrupted:
            print(f"Algae {self.direction} TOGGLED OFF (interrupted)")
        else:
            print(f"Algae {self.direction} TOGGLED OFF")
        
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # This command runs until canceled
        return False