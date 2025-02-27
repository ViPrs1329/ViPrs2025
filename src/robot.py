#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib

# If in simulation mode, load simulation hooks
if wpilib.RobotBase.isSimulation():
    import sim

import wpilib.drive
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d
import rev
import math
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.ElevatorSubsystem import Elevator
import constants
import numpy as np
import ntcore



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
      self.drivetrain.frontRightRotation#,
      #self.elevator.leftElevatorMotor,
      #self.elevator.rightElevatorMotor
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

    # initialize network tables
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("datatable")
    self.controllerXPub = table.getDoubleTopic("controller x").publish()
    self.controllerYPub = table.getDoubleTopic("controller y").publish()
    self.robotPosition = table.getStructTopic("robot pose", Pose2d).publish()
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
    s = MyRobot.inputCurve(d)
    sx = x * s
    sy = y * s
    if sx * sx + sy * sy > 1:
      scale = 1 / math.sqrt(sx * sx + sy * sy)
      sx *= scale
      sy *= scale
    return sx * constants.controller.scale, sy * constants.controller.scale
  
  def teleopPeriodic(self):
    """This function is called periodically during teleoperated mode."""
    # print("teleopPeriodic()")
    xSpeed, ySpeed = MyRobot.distanceCorrectedInputCurve(self.drivingXboxController.getLeftY(), self.drivingXboxController.getLeftX())
    # xSpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftY())
    # ySpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftX())
    self.controllerXPub.set(xSpeed)
    self.controllerYPub.set(ySpeed)

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
        
    self.robotPosition.set(self.drivetrain.combinedPosition)

  def testInit(self): 
    """This function is called once each time the robot enters test mode."""
    print("testInit()")
        
        
  def testPeriodic(self): 
    """This function is called periodically during test mode."""
    print("testPeriodic()")
    pass

  def simulationInit(self):
      print("Simulation init...")
      # Initialize any simulation-specific components
      self.simulation_table = ntcore.NetworkTableInstance.getDefault().getTable("simulation")
      
      # If you have LaserCAN sensors in EndEffector subsystem
      if hasattr(self, 'endEffector'):
          # Create network table entries for sensor values
          self.coral_entry_distance = self.simulation_table.getDoubleTopic("coral_entry_distance").publish()
          self.coral_stop_distance = self.simulation_table.getDoubleTopic("coral_stop_distance").publish()
          
          # Set initial values
          self.coral_entry_distance.set(1000)  # 1000mm (nothing detected)
          self.coral_stop_distance.set(1000)   # 1000mm (nothing detected)

  def SimulationPeriodic(self):
      # Update simulation values periodically
      if hasattr(self, 'endEffector'):
          # Get values from Network Tables that could be set by simulator GUI
          entry_distance = self.simulation_table.getDoubleTopic("coral_entry_distance").subscribe(1000).get()
          stop_distance = self.simulation_table.getDoubleTopic("coral_stop_distance").subscribe(1000).get()
          
          # Update the simulated sensors
          if hasattr(self.endEffector.coral_intake_LC, 'sim_device'):
              self.endEffector.coral_intake_LC.sim_device.set_simulated_distance(entry_distance)
          if hasattr(self.endEffector.coral_stop_LC, 'sim_device'):
              self.endEffector.coral_stop_LC.sim_device.set_simulated_distance(stop_distance)
        


if __name__ == "__main__":
  wpilib.run(MyRobot)