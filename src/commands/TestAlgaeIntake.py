import commands2
import wpilib
from subsystems.EndEffector import EndEffector
import time

class TestAlgaeIntake(commands2.Command):
    """
    Test command to verify algae intake motor direction and operation.
    This command will:
    1. Run the motor forward for 2 seconds
    2. Stop for 1 second
    3. Run the motor backward for 2 seconds
    4. Stop
    
    This helps verify the correct motor direction for intake and ejection.
    """
    def __init__(self, endEffector: EndEffector):
        super().__init__()
        self.EE = endEffector
        self.addRequirements(endEffector)
        self.timer = wpilib.Timer()
        self.stage = 0
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.timer.reset()
        self.timer.start()
        self.stage = 0
        print("Starting algae intake motor test sequence")
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        elapsed = self.timer.get()
        
        # Stage 0: Forward (intake) for 2 seconds
        if self.stage == 0:
            if elapsed < 2.0:
                self.EE.setAlgaeIntakeSpeed(0.3)  # Use a lower test speed
                print(f"Testing FORWARD direction (should be INTAKE) - Time: {elapsed:.1f}s")
            else:
                self.stage = 1
                self.timer.reset()
                self.timer.start()
                self.EE.setAlgaeIntakeSpeed(0)
                print("Forward test complete, pausing...")
        
        # Stage 1: Pause for 1 second
        elif self.stage == 1:
            if elapsed > 1.0:
                self.stage = 2
                self.timer.reset()
                self.timer.start()
                print("Starting reverse test...")
        
        # Stage 2: Backward (eject) for 2 seconds
        elif self.stage == 2:
            if elapsed < 2.0:
                self.EE.setAlgaeIntakeSpeed(-0.3)  # Use a lower test speed
                print(f"Testing REVERSE direction (should be EJECT) - Time: {elapsed:.1f}s")
            else:
                self.stage = 3
                self.EE.setAlgaeIntakeSpeed(0)
                print("Reverse test complete, test sequence finished")
        
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        self.EE.setAlgaeIntakeSpeed(0)
        self.timer.stop()
        if interrupted:
            print("Algae intake test was interrupted")
        else:
            print("Algae intake test completed successfully")
            
        print("VERIFICATION: ")
        print("- If motor spun inward during FORWARD, the direction is correct")
        print("- If motor spun outward during FORWARD, reverse the motor in the EndEffector.py file")
        
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # End after all stages are complete
        return self.stage >= 3