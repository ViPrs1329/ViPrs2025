#!/usr/bin/env python3
"""
Main robot class for Team VIPRS' 2025 FRC Reefscape competition robot.
"""
import wpilib
import commands2

from robotcontainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Main robot class. This is the entry point for the robot code.
    """

    def robotInit(self) -> None:
        """
        This function is called upon robot startup.
        The robot should be initialized here.
        """
        # Create the RobotContainer. This creates and binds all subsystems and commands.
        self.container = RobotContainer()

    def robotPeriodic(self) -> None:
        """
        This function is called every 20 ms, no matter the mode.
        """
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self) -> None:
        """
        This function is called once when autonomous mode starts.
        """
        self.autonomous_command = self.container.getAutonomousCommand()

        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        """
        This function is called periodically during autonomous mode.
        """
        pass

    def teleopInit(self) -> None:
        """
        This function is called once when teleop mode starts.
        """
        # This ensures that the autonomous stops running when
        # teleop starts running.
<<<<<<< HEAD
        # if self.autonomous_command is not None:
        #     self.autonomous_command.cancel()
=======
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
>>>>>>> 0f35bd26675a1644dc9f6f438f5c9e4297dc0f25

    def teleopPeriodic(self) -> None:
        """
        This function is called periodically during operator control.
        """
        pass

    def testInit(self) -> None:
        """
        This function is called once when test mode starts.
        """
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        """
        This function is called periodically during test mode.
        """
        pass

    def disabledInit(self) -> None:
        """
        This function is called once when the robot is disabled.
        """
        pass

    def disabledPeriodic(self) -> None:
        """
        This function is called periodically when the robot is disabled.
        """
        pass


if __name__ == "__main__":
    wpilib.run(Robot) 