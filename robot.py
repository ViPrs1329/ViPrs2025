#!/usr/bin/env python3

import wpilib
import commands2
import ntcore
from pathplannerlib.path import PathPlannerPath
from robotcontainer import RobotContainer

class Robot(commands2.TimedCommandRobot):
    """
    Main robot class.
    """
    
    def robotInit(self) -> None:
        """
        Robot initialization.
        """
        # Create the robot container
        self.container = RobotContainer()
        
        # Set up autonomous command
        self.autonomous_command = None
    
    def robotPeriodic(self) -> None:
        """
        Periodic code for all robot modes.
        This runs after mode-specific periodic functions.
        """
        # Run command scheduler
        commands2.CommandScheduler.getInstance().run()
        
        # Update telemetry
        self.container.periodic()
    
    def autonomousInit(self) -> None:
        """
        Called when autonomous mode starts.
        """
        self.autonomous_command = self.container.getAutonomousCommand()
        
        if self.autonomous_command:
            self.autonomous_command.schedule()
    
    def autonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode.
        """
        pass
    
    def teleopInit(self) -> None:
        """
        Called when teleop mode starts.
        """
        # Cancel autonomous command if it's still running
        if self.autonomous_command:
            self.autonomous_command.cancel()
    
    def teleopPeriodic(self) -> None:
        """
        Periodic code for teleop mode.
        """
        pass
    
    def testInit(self) -> None:
        """
        Called when test mode starts.
        """
        # Cancel all running commands
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testPeriodic(self) -> None:
        """
        Periodic code for test mode.
        """
        pass
    
    def disabledInit(self) -> None:
        """
        Called when the robot is disabled.
        """
        pass
    
    def disabledPeriodic(self) -> None:
        """
        Periodic code for disabled mode.
        """
        pass

if __name__ == "__main__":
    wpilib.run(Robot)
