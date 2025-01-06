#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib
import wpilib.drive
import rev
import commands2


class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand = None
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        print("robotInit()")

    def robotPeriodic(self):
        print("robotPeriodic()")
        
    
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        print("autonomousInit()")
        
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        print("autonomousPeriodic()")

        pass

    def disabledInit(self):
        """This function is called initially when disabledd"""
        print("disabledInit()")

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        print("teleopInit()")
        
        
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        print("teleopPeriodic()")
        

    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        print("testInit()")
        
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""
        print("testPeriodic()")
        pass

    def simulationInit(self):
        print("Simulation init...")
        

    def SimulationPeriodic(self):
        """"This function is called periodically during the simulation mode"""
        print("SimulationPeriodic()")
        


if __name__ == "__main__":
    wpilib.run(MyRobot)