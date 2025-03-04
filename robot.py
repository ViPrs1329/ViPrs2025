#!/usr/bin/env python3

import wpilib
import commands2
import ntcore
from pathplannerlib.path import PathPlannerPath
from robotcontainer import RobotContainer

class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # Create SmartDashboard instance
        self.sd = wpilib.SmartDashboard

        # Initialize subsystems
        self.container = RobotContainer()

    def robotPeriodic(self):
        """This function is called periodically regardless of mode."""
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self):
        """This function is called once each time the robot enters autonomous mode."""
        self.autonomous_command = self.container.getAutonomousCommand()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopInit(self):
        """This function is called once each time the robot enters teleop mode."""
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        """This function is called periodically during teleop."""
        pass

    def testInit(self):
        """This function is called once each time the robot enters test mode."""
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        pass

    def disabledInit(self):
        """This function is called once each time the robot is disabled."""
        pass

    def disabledPeriodic(self):
        """This function is called periodically while disabled."""
        pass

if __name__ == "__main__":
    wpilib.run(Robot)
