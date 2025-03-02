import wpilib
import pathplannerlib
from wpilib.command import Command
from constants import autonomousCommand
from subsystems.SwerveDriveSubsystem import DriveTrain

class FollowPathCommand(Command):
    """
    A command to follow a PathPlanner path.
    """

    def __init__(self, path_name: autonomousCommand, drive_function: DriveTrain.driveFromChassisSpeeds):
        """
        Initializes the FollowPathCommand.

        Args:
            path_name (str): The name of the PathPlanner path file (without .path).
            drive_function (function): A function that takes x, y, and heading outputs and drives the robot.
        """
        super().__init__("Follow Path")
        self.path_name = path_name
        self.drive_function = drive_function
        self.trajectory = None
        self.timer = wpilib.Timer()

    def initialize(self):
        """
        Called when the command is started.
        """
        self.trajectory = pathplannerlib.PathPlannerTrajectory(
            pathplannerlib.PathPlanner.loadPath(self.path_name, pathplannerlib.PathPlanner.FileType.ON_ROBOT)
        )
        self.timer.restart()

    def execute(self):
        """
        Called repeatedly while the command is running.
        """
        current_state = self.trajectory.sample(self.timer.get())

        # Here, you would use PID controllers to calculate the outputs
        # based on the current state and your robot's sensors.
        # For simplicity, we'll just use the desired state directly.
        desired_x = current_state.x
        desired_y = current_state.y
        desired_heading = current_state.heading

        # Call the drive function to move the robot
        self.drive_function(desired_x, desired_y, desired_heading)

    def isFinished(self):
        """
        Returns True when the command should stop.
        """
        return self.timer.get() >= self.trajectory.getTotalTime()

    def end(self):
        """
        Called when the command ends.
        """
        self.timer.stop()
        #optional: stop the robot here.