#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib
import rev
import commands2
from robotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self) -> None:
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        self.robotContainer: RobotContainer
        self.autonomousCommand: commands2.Command | None

        try:
            self.robotContainer = RobotContainer()
            self.autonomousCommand = None
        except Exception as e:
            raise RuntimeError(f"Failed to initialize RobotContainer:\n{e}")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

        self.robotContainer.drivetrain.switchToAutonomous()
        self.autonomousCommand = self.robotContainer.getAutonomousCommand()
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def autonomousExit(self) -> None:
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

        self.autonomousCommand = None

    def disabledInit(self):
        """This function is called initially when disabledd"""
        self.robotContainer.subsystemWrapper.resetSubsystems()

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()
            
        self.autonomousCommand = None

        self.robotContainer.subsystemWrapper.resetBeforeTeleop()
        self.robotContainer.drivetrain.switchToTeleop()
        
        
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        pass
        
    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        print("testInit()")
        
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""
        print("testPeriodic()")
        pass

    def simulationInit(self):
        print("Simulation init...")
        

    def simulationPeriodic(self):
        """"This function is called periodically during the simulation mode"""
        print("SimulationPeriodic()")
        


if __name__ == "__main__":
    wpilib.run(MyRobot)