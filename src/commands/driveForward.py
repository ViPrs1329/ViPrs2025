import commands2
import wpilib
# Can't import RobotContainer
from subsystems.SwerveDriveSubsystem import DriveTrain
import constants

class driveForward(commands2.Command):
    def __init__(self, driveTrain: DriveTrain):
        super().__init__()
        self.DT = driveTrain
        self.timer=wpilib.Timer()

    def initialize(self):
        self.DT.resetMotors()
        self.timer.restart()
        print('driveForward()')

    def execute(self):
        self.DT.driveFromChassisSpeeds(1, 0, 0)

    def end(self, interrupted: bool):
        self.DT.resetMotors()
        
    def isFinished(self) -> bool:
        return self.timer.get() >= constants.driveConsts.autonomousTimeautonomousTime