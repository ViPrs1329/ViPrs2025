#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib
import wpilib.drive
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
import rev
import math
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain
import constants
import numpy as np

class MyRobot(commands2.TimedCommandRobot):
  def systemTempCheck(self):
    motorControllers = [
      self.drivetrain.frontLeftDrive,
      self.drivetrain.frontRightDrive,
      self.drivetrain.backLeftDrive,
      self.drivetrain.backRightDrive,
      self.drivetrain.backLeftRotation,
      self.drivetrain.backRightRotation,
      self.drivetrain.frontLeftRotation,
      self.drivetrain.frontRightRotation,
    ]

    burntFlag = False
    for motorController in motorControllers:
      temp = motorController.getMotorTemperature()
      if temp > 90:
        print(f"[x] Motor {motorController.getDeviceId()}, {temp}C")
        burntFlag = True
      else:
        print(f"[-] Motor {motorController.getDeviceId()}, {temp}C")
    
    if burntFlag:
      self.startRumble()
      for i in range(100):
        print("!!! ---------------- MOTORS TOO HOT ------------------- !!!")
        
  def startRumble(self):
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kRightRumble,1)
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kLeftRumble,1)

  def stopRumble(self):
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kRightRumble,0)
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kLeftRumble,0)

  autonomousCommand = None
  def robotInit(self):
    """
    This function is called upon program startup and
    should be used for any initialization code.
    """
    self.drivingXboxController = wpilib.XboxController(0)
    self.drivetrain = DriveTrain()

    print("robotInit()")

  def robotPeriodic(self):
    # print("robotPeriodic()")
    pass
        
  def autonomousInit(self):
    """This function is run once each time the robot enters autonomous mode."""
    print("autonomousInit()")

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
    self.stopRumble()
    self.drivetrain.resetHarder()
    self.systemTempCheck()

  def inputCurve(input: float):
    return (input ** 3)

  def tinputCurve(input: float):
    return (input ** 3) * constants.controller.tscale

  def distanceCorrectedInputCurve(x: float, y: float):
    d = math.sqrt(x * x + y * y)
    print(x)
    print(y)
    print(d)
    s = MyRobot.inputCurve(d)
    sx = x * s * constants.controller.scale
    sy = y * s * constants.controller.scale
    if sx * sx + sy * sy > 1:
      scale = 1 / math.sqrt(sx * sx + sy * sy)
      sx *= scale
      sy *= scale
    return sx, sy
  
  def teleopPeriodic(self):
    """This function is called periodically during teleoperated mode."""
    print("teleopPeriodic()")
    xSpeed, ySpeed = MyRobot.distanceCorrectedInputCurve(self.drivingXboxController.getLeftY(), self.drivingXboxController.getLeftX())
    # xSpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftY())
    # ySpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftX())
    tSpeed = MyRobot.tinputCurve(-self.drivingXboxController.getRightX())

    if abs(xSpeed) < constants.controller.XYdeadzone:
      xSpeed=0
    if abs(ySpeed) < constants.controller.XYdeadzone:
      ySpeed=0
    if abs(tSpeed) < constants.controller.Tdeadzone:
      tSpeed=0

    yaw = self.drivetrain.gyro.get_yaw().value_as_double

    h = yaw % 360
    if h < 0:
      h += 360

    h2 = h / 360

    heading = h2 * (math.pi * 2)

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, -tSpeed, Rotation2d(heading))
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        

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