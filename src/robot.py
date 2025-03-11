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
from wpimath.geometry import Rotation2d, Pose2d
import rev
import math
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
import constants
import numpy as np
import ntcore

from commands.slow import Slow

from commands.lb import LB
from commands.lt import LT
from commands.rb import RB
from commands.rt import RT
from commands.setElevator import SetElevator
from commands.intake import Intake
from commands.driveForward import driveForward
from commands.algaeIntake import AlgaeIntake
from commands.algaeArmCyclePositions import AlgaeArmCyclePositions
from commands.ToggleDebugMode import ToggleDebugMode
# from commands.pathplannerCommand import FollowPathCommand
from commands.waitUntilCoralIsDetected import WaitUntilCoralIsDetected
from phoenix6.hardware import CANrange

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
      for i in range(100):
        print("!!! ---------------- MOTORS TOO HOT ------------------- !!!")
        
  def startRumble(self):
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kRightRumble,1)
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kLeftRumble,1)

  def stopRumble(self):
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kRightRumble,0)
    self.drivingXboxController.setRumble(self.drivingXboxController.RumbleType.kLeftRumble,0)

  def coralIsInRangeEE(self):
    "returns true when coral is detected"
    return self.canRangeEE.get_distance().value_as_double < constants.intakeConsts.coralDetectionThreshold

  def coralIsOutOfRangeFunnel(self):
    "returns true when coral is not detected"
    return self.canRangeFunnel.get_distance().value_as_double > constants.intakeConsts.coralDetectionThreshold

  def enableSlow(self):
    self.slowScaler = 0.1

  def disableSlow(self):
    self.slowScaler = 1

  def configureButtonBindings(self):
    # slow down the robot when right trigger is pressed
    self.drivingCommandXboxController.rightTrigger().onTrue(
      commands2.InstantCommand(self.enableSlow())
    )
    self.drivingCommandXboxController.rightTrigger().onFalse(
      commands2.InstantCommand(self.disableSlow())
    )

    # elevator positions

    # self.EEECommandXboxController.leftTrigger().whileTrue(LT(self.EEEPressedButtons))
    # self.EEECommandXboxController.rightTrigger().whileTrue(RT(self.EEEPressedButtons))
    self.EEECommandXboxController.leftBumper().onTrue(SetElevator("down", self.elevatorController, self.endEffector))
    self.EEECommandXboxController.rightBumper().onTrue(SetElevator("up", self.elevatorController, self.endEffector))
    # self.EEECommandXboxController.b().onTrue(SetElevator(self.EEEPressedButtons, self.elevatorController, self.endEffector))
    self.EEECommandXboxController.y().whileTrue(AlgaeIntake(self.endEffector))
    self.EEECommandXboxController.a().onTrue(AlgaeArmCyclePositions(self.endEffector))

    # self.coralIntakeCommand = commands2.ConditionalCommand(Intake(self.endEffector), commands2.InstantCommand(), self.canRangeFunnel.get_measurement)

    # self.coralIntakeCommand = commands2.ConditionalCommand(
    #   commands2.ConditionalCommand(
    #     commands2.ConditionalCommand(
    #       commands2.InstantCommand(), 
    #       Intake(self.endEffector), 
    #       self.coralIsInRange(self.canRangeFunnel)
    #     ), 
    #     commands2.InstantCommand(), 
    #     self.coralIsInRange(self.canRangeEE)
    #   ), 
    #   commands2.InstantCommand(), 
    #   self.coralIsInRange(self.canRangeFunnel)
    # )

    # wait until coral is detected by the EE canrange 
    # then wait until coral is undetected by the funnel canrange 
    # then stop the intake motors
    self.EEECommandXboxController.x().onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(lambda: print("in")),
        commands2.InstantCommand(lambda: self.endEffector.startCoralMotors()),
        WaitUntilCoralIsDetected(self.coralIsInRangeEE),
        WaitUntilCoralIsDetected(self.coralIsOutOfRangeFunnel),
        commands2.InstantCommand(lambda: self.endEffector.stopCoralMotors())
      )
    )

    # Add debug mode toggle on Back/Select button
    self.drivingCommandXboxController.back().onTrue(
        ToggleDebugMode(lambda: self.is_debug_mode)
    )

    # self.coralIntakeCommand = commands2.SequentialCommandGroup(
    #   WaitUntilCoralIsDetected(self.coralIsInRangeEE),
    #   WaitUntilCoralIsDetected(self.coralIsOutOfRangeFunnel),
    #   Intake(self.endEffector)
    # )
  autonomousCommand = driveForward

  def robotInit(self):
    """
    This function is called upon program startup and
    should be used for any initialization code.
    """
    self.drivingXboxController = wpilib.XboxController(0)
    self.drivingCommandXboxController = commands2.button.CommandXboxController(0)
    self.EEEXboxController = wpilib.XboxController(1)
    self.EEECommandXboxController = commands2.button.CommandXboxController(1)
    
    self.drivetrain = DriveTrain()
    self.elevatorController = Elevator()
    self.endEffector = EndEffector()
    #self.elevator = Elevator()

    # initialize network tables
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("datatable")
    self.controllerXPub = table.getDoubleTopic("controller x").publish()
    self.controllerYPub = table.getDoubleTopic("controller y").publish()
    self.robotPosition = table.getStructTopic("robot pose", Pose2d).publish()
    self.headingValue = table.getDoubleTopic("heading").publish()
    
    self.slowScaler = 1

    self.EEEPressedButtons = [False, False, False, False] # left trigger, right trigger, left bumper, right bumper

    self.scheduler = commands2.CommandScheduler.getInstance()

    self.canRangeFunnel = CANrange(constants.CANIDs.CanRangeFunnel)
    self.canRangeEE = CANrange(constants.CANIDs.CanRangeEE)

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
    self.configureButtonBindings()
    self.elevatorController.zeroElevator()
    # self.endEffector.startCoralMotors()

  def inputCurve(input: float):
    return (input ** 3)

  def tinputCurve(input: float):
    return (input ** 3) * constants.controller.tscale

  def distanceCorrectedInputCurve(x: float, y: float):
    d = math.sqrt(x * x + y * y)
    s = MyRobot.inputCurve(d)
    sx = x * s
    sy = y * s
    if d > 1:
      scale = 1 / math.sqrt(sx * sx + sy * sy)
      sx *= scale
      sy *= scale
    return sx * constants.controller.scale, sy * constants.controller.scale
  
  def teleopPeriodic(self):
    """This function is called periodically during teleoperated mode."""
    # self.drivetrain.stopMotors()
    # print("teleopPeriodic()")
    xSpeed, ySpeed = MyRobot.distanceCorrectedInputCurve(self.drivingXboxController.getLeftY(), self.drivingXboxController.getLeftX())
    # xSpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftY())
    # ySpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftX())
    # print('X Speed - ' + str(xSpeed))
    # print('Y Speed - ' + str(ySpeed))
    self.controllerXPub.set(xSpeed * self.slowScaler)
    self.controllerYPub.set(ySpeed * self.slowScaler)
    print('\nAlgae Arm Angle:')
    print(self.endEffector.getAlgaeArmAngle())
    #print('\nAlgae Motor Rotations')
    #print(self.endEffector.getAlgaeArmRotations())
    print('\nSetpoint:')
    print(self.endEffector.algaePID.getSetpoint())
    #print('\nDesired Velocity:')
    #print(desiredVelocity)
    

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
    self.headingValue.set(heading)
    #print(xSpeed, ySpeed, tSpeed)
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * self.slowScaler, ySpeed * self.slowScaler, -tSpeed, Rotation2d(heading))
    self.drivetrain.manualDriveFromChassisSpeeds(speeds)
    self.robotPosition.set(self.drivetrain.combinedPosition)

    self.scheduler.run()

    # important print statement
    # print(self.canRangeEE.get_distance().value_as_double)
#     print(f"""
# target in: {2 * constants.convert.rot2in(self.elevatorController.destination)}
# current in: {2 * constants.convert.rot2in(self.elevatorController.getElevatorPosition())}
# target rot: {self.elevatorController.destination}
# current rot: {self.elevatorController.getElevatorPosition()}
# current: {self.elevatorController.REM.getOutputCurrent()}""")

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