import commands2
from subsystems.LimelightSubsystem import LimelightSubsystem

class AprilTagMonitorCommand(commands2.RunCommand):
    def __init__(self, vision_subsystem: LimelightSubsystem, action_if_detected, action_if_not_detected):
        super().__init__(lambda: self.execute(), vision_subsystem)
        self.vision_subsystem = vision_subsystem
        self.action_if_detected = action_if_detected
        self.action_if_not_detected = action_if_not_detected
        self.temp = False

    def execute(self):
        if self.vision_subsystem.limelightLeftDetectsTag() or self.vision_subsystem.limelightRightDetectsTag():
            # Perform actions when an AprilTag is detected
            if self.temp == False:
              self.action_if_detected()
              self.temp = True
        else:
            # Perform actions when no AprilTag is detected
            if self.temp == True:
              self.action_if_not_detected()
              self.temp = False

    def end(self, interrupted: bool):
        # Optional: Perform any cleanup when the command ends
        pass

    def isFinished(self):
        # This command should run continuously, so it's never finished
        return False