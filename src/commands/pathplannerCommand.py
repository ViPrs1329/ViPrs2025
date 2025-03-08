import wpilib
import pathplannerlib
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain

class FollowPathCommand(commands2.Command):
    """
    A command to follow a PathPlanner path.
    """

    def __init__(self, pathName):
        """
        Initializes the FollowPathCommand.

        Args:
            pathName (str): The name of the PathPlanner path file (without .path).
            driveFunction (function): A function that takes x, y, and heading outputs and drives the robot.
        """
        super().__init__()
        print('initFollowPath()')
        self.pathName = pathName
        self.driveFunction = DriveTrain.driveFromChassisSpeeds # THIS IS NOT A STATIC METHOD! THIS WON'T WORK!
        self.trajectory = None
        self.timer = wpilib.Timer()

    def initialize(self):
        print('initializeFollowPath()')
        """
        Called when the command is started.
        """
        self.trajectory = pathplannerlib.PathPlannerTrajectory(
            pathplannerlib.PathPlanner.loadPath(self.pathName, pathplannerlib.PathPlanner.FileType.ON_ROBOT)
        )
        self.timer.restart()

    def execute(self):
        print('executeFollowPath()')
        """
        Called repeatedly while the command is running.
        """
        currentState = self.trajectory.sample(self.timer.get())

        # Here, you would use PID controllers to calculate the outputs
        # based on the current state and your robot's sensors.
        # For simplicity, we'll just use the desired state directly.
        desiredX = currentState.x
        desiredY = currentState.y
        desiredHeading = currentState.heading

        # Call the drive function to move the robot
        self.driveFunction(desiredX, desiredY, desiredHeading)

    def isFinished(self):
        print('isFinishedFollowPath()')
        """
        Returns True when the command should stop.
        """
        return self.timer.get() >= self.trajectory.getTotalTime()

    def end(self):
        print('endFollowPath()')
        """
        Called when the command ends.
        """
        self.timer.stop()
        DriveTrain.resetMotors()
        