#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpinet
import wpilib
import wpilib.drive
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Translation3d, Rotation3d, Translation2d
import rev
import math
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.ElevatorSubsystem import Elevator
from subsystems.EndEffector import EndEffector
from subsystems.SimpleVisionSubsystem import SimpleVisionSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.LedSubsystem import LED
from team1329.SimpleButtonBoardDebug import SimpleButtonBoardDebug
import constants
import numpy as np
import ntcore
# from cscore import CameraServer

from commands.slow import Slow

from commands.lb import LB
from commands.lt import LT
from commands.rb import RB
from commands.rt import RT
from commands.setElevator import SetElevator
from commands.intake import Intake
from commands.driveForward import driveForward
from commands.DriveDistance import DriveDistance
from commands.algaeArmCyclePositions import AlgaeArmCyclePositions
from commands.ToggleDebugMode import ToggleDebugMode
from commands.JoystickElevatorControl import JoystickElevatorControl
from commands.SetElevatorWithDebugCheck import SetElevatorWithDebugCheck
from commands.MoveAlgaeArmToPosition import MoveAlgaeArmToPosition
from commands.AlgaeIntakeControl import AlgaeIntakeControl
from commands.TestAlgaeIntake import TestAlgaeIntake
# from commands.pathplannerCommand import FollowPathCommand
from commands.waitUntilCoralIsDetected import WaitUntilCoralIsDetected
from commands.autoAlign import AutoAlign
from commands.detectAprilTag import AprilTagMonitorCommand
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
    return self.canRangeFunnel.get_distance().value_as_double > 0.1 # constants don't work it was the same thing in coralisinrangeee

  def enableSlow(self):
    self.slowScaler = 0.1

  def disableSlow(self):
    self.slowScaler = 1

  def ejectCoral(self):
    if self.elevatorController.currentLevel != 1:
      self.endEffector.coral_intake_left_motor.set(constants.intakeConsts.intakeSpeed)
      self.endEffector.coral_intake_right_motor.set(constants.intakeConsts.intakeSpeed)
    else:
      self.endEffector.coral_intake_left_motor.set(constants.intakeConsts.intakeSpeed / 2)
      self.endEffector.coral_intake_right_motor.set(constants.intakeConsts.intakeSpeed / 4)

  def flopArm(self):
    # print("floppp")
    self.endEffector.flopArm = True

  def unFlopArm(self):
    # print("unnn floppp")
    self.endEffector.flopArm = False

  def shouldFlopArm(self):
    return self.endEffector.algae_intake_motor.getOutputCurrent() > 15 # can't put a constant here because python is dumb?????
  
  def goToBaseLevel(self):
    target_height = constants.reefConsts.reefLevels[0][1] + constants.elevatorConsts.verticalOffset
    target_position = constants.convert.in2rot(target_height) / 2
    self.elevatorController.currentLevel = 1
    # Move the elevator and arm to the appropriate positions
    self.elevatorController.gotoPosition(target_position)

  def goToL2(self):
    target_height = constants.reefConsts.reefLevels[1][1] + constants.elevatorConsts.verticalOffset
    target_position = constants.convert.in2rot(target_height) / 2
    self.elevatorController.currentLevel = 2
    self.elevatorController.gotoPosition(target_position)

  def goToL3(self):
    target_height = constants.reefConsts.reefLevels[2][1] + constants.elevatorConsts.verticalOffset
    target_position = constants.convert.in2rot(target_height) / 2
    self.elevatorController.currentLevel = 3
    self.elevatorController.gotoPosition(target_position)
  
  def switchToAutoDrive(self):
    self.manualDrive = False
  
  def switchToManualDrive(self):
    self.manualDrive = True

  def configureButtonBindings(self):
    # slow down the robot when right trigger is pressed
    self.drivingCommandXboxController.rightTrigger().onTrue(
      commands2.InstantCommand(lambda: self.enableSlow())
    )
    self.drivingCommandXboxController.rightTrigger().onFalse(
      commands2.InstantCommand(lambda: self.disableSlow())
    )
    self.drivingCommandXboxController.x().onTrue(
      commands2.InstantCommand(lambda: self.drivetrain.gyro.set_yaw(0))
    )

    # Auto Align to an april tag
    # TODO configure this command to a seperate game pad
    self.drivingCommandXboxController.povLeft().onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligning)
        ),
        commands2.InstantCommand(
          self.switchToAutoDrive
        ),
        AutoAlign(self.llController, self.drivetrain, "left"),
        commands2.InstantCommand(
          self.switchToManualDrive
        ),
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligned)
        )
      )
    )

    self.drivingCommandXboxController.povRight().onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligning)
        ),
        commands2.InstantCommand(
          self.switchToAutoDrive
        ),
        AutoAlign(self.llController, self.drivetrain, "right"),
        commands2.InstantCommand(
          self.switchToManualDrive
        ),
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligned)
        )
      )
    )
    
    # Button 10 for Auto Align Left
    self.buttonBoardCommandController.button(10).onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligning)
        ),
        commands2.InstantCommand(
          self.switchToAutoDrive
        ),
        AutoAlign(self.llController, self.drivetrain, "left"),
        commands2.InstantCommand(
          self.switchToManualDrive
        ),
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligned)
        )
      )
    )

    # Button 9 for Auto Align Right
    self.buttonBoardCommandController.button(9).onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligning)
        ),
        commands2.InstantCommand(
          self.switchToAutoDrive
        ),
        AutoAlign(self.llController, self.drivetrain, "right"),
        commands2.InstantCommand(
          self.switchToManualDrive
        ),
        commands2.InstantCommand(
          lambda: self.ledController.changeStates(constants.RobotStates.aligned)
        )
      )
    )
    """
    # Button ???: Elevator to L2 + Auto Align Left
    self.buttonBoardCommandController.button(???).onTrue(
        commands2.SequentialCommandGroup(
            # First, set elevator to L2
            commands2.InstantCommand(
                lambda: print("Moving to L2 and auto-aligning LEFT")
            ),
            commands2.InstantCommand(
                lambda: self.elevatorController.currentLevel = 1  # Start from a known position
            ),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            # Then auto-align left
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligning)
            ),
            commands2.InstantCommand(
                self.switchToAutoDrive
            ),
            AutoAlign(self.llController, self.drivetrain, "left"),
            commands2.InstantCommand(
                self.switchToManualDrive
            ),
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligned)
            )
        )
    )

    # Button ???: Elevator to L2 + Auto Align Right
    self.buttonBoardCommandController.button(???).onTrue(
        commands2.SequentialCommandGroup(
            # First, set elevator to L2
            commands2.InstantCommand(
                lambda: print("Moving to L2 and auto-aligning RIGHT")
            ),
            commands2.InstantCommand(
                lambda: self.elevatorController.currentLevel = 1  # Start from a known position
            ),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            # Then auto-align right
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligning)
            ),
            commands2.InstantCommand(
                self.switchToAutoDrive
            ),
            AutoAlign(self.llController, self.drivetrain, "right"),
            commands2.InstantCommand(
                self.switchToManualDrive
            ),
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligned)
            )
        )
    )

    # Button ???: Elevator to L3 + Auto Align Left
    self.buttonBoardCommandController.button(???).onTrue(
        commands2.SequentialCommandGroup(
            # First, set elevator to L3 (need two "up" commands from L1)
            commands2.InstantCommand(
                lambda: print("Moving to L3 and auto-aligning LEFT")
            ),
            commands2.InstantCommand(
                lambda: self.elevatorController.currentLevel = 1  # Start from a known position
            ),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            # Then auto-align left
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligning)
            ),
            commands2.InstantCommand(
                self.switchToAutoDrive
            ),
            AutoAlign(self.llController, self.drivetrain, "left"),
            commands2.InstantCommand(
                self.switchToManualDrive
            ),
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligned)
            )
        )
    )

    # Button ???: Elevator to L3 + Auto Align Right
    self.buttonBoardCommandController.button(???).onTrue(
        commands2.SequentialCommandGroup(
            # First, set elevator to L3 (need two "up" commands from L1)
            commands2.InstantCommand(
                lambda: print("Moving to L3 and auto-aligning RIGHT")
            ),
            commands2.InstantCommand(
                lambda: self.elevatorController.currentLevel = 1  # Start from a known position
            ),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
            # Then auto-align right
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligning)
            ),
            commands2.InstantCommand(
                self.switchToAutoDrive
            ),
            AutoAlign(self.llController, self.drivetrain, "right"),
            commands2.InstantCommand(
                self.switchToManualDrive
            ),
            commands2.InstantCommand(
                lambda: self.ledController.changeStates(constants.RobotStates.aligned)
            )
        )
    )
    """
  
    # Toggle front Limelight driver mode on/off using POV Up (D-pad Up)
    # self.drivingCommandXboxController.povUp().onTrue(
    #     commands2.InstantCommand(
    #         lambda: self.vision.toggleFrontLimelightMode()
    #     )
    # )
    
    # Toggle rear Limelight driver mode on/off using POV Down (D-pad Down)
    # self.drivingCommandXboxController.povDown().onTrue(
    #     commands2.InstantCommand(
    #         lambda: self.vision.toggleRearLimelightMode()
    #     )
    # )
    
    # Set both Limelights to driver mode using POV Left (D-pad Left)
    # self.drivingCommandXboxController.povLeft().onTrue(
    #     commands2.InstantCommand(
    #         lambda: [
    #             self.vision.setDriverMode(self.vision.limelight_front, True),
    #             self.vision.setDriverMode(self.vision.limelight_rear, True)
    #         ]
    #     )
    # )

    # elevator positions

    # self.EEECommandXboxController.leftTrigger().whileTrue(LT(self.EEEPressedButtons))
    # self.EEECommandXboxController.rightTrigger().whileTrue(RT(self.EEEPressedButtons))
    self.EEECommandXboxController.leftBumper().onTrue(
      commands2.ParallelCommandGroup(
        SetElevator("down", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()),
        commands2.InstantCommand(lambda: print("up"))
      )  
    )
    self.EEECommandXboxController.rightBumper().onTrue(SetElevator("up", self.elevatorController, self.endEffector, lambda: self.coralIsOutOfRangeFunnel()))
    
    # Elevator positions - button board
    # Button 3 - Base
    self.buttonBoardCommandController.button(3).onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand( lambda: self.goToBaseLevel() ),
        commands2.InstantCommand( lambda: print("Elevator to base BB"))
      )
    )

    # Button 2 - L2
    self.buttonBoardCommandController.button(2).onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand( lambda: self.goToL2() ),
        commands2.InstantCommand( lambda: print("Elevator to base BB"))
      )
    )

    # Button 1 - L3
    self.buttonBoardCommandController.button(1).onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand( lambda: self.goToL3() ),
        commands2.InstantCommand( lambda: print("Elevator to base BB"))
      )
    )
    
    # Eject coral - Xbox controller
    self.EEECommandXboxController.a().onTrue(
      commands2.InstantCommand(
        lambda: self.ejectCoral()
      )
    )

    # Eject coral - Button board
    # Axis 3 - 1.0 
    commands2.button.Trigger(lambda: abs(self.buttonBoardCommandController.getRawAxis(3) - 1.0) < 0.1).onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand(
          lambda: self.ejectCoral()
        ),
        commands2.InstantCommand(
          lambda: print(f"eject coral - {self.buttonBoardCommandController.getRawAxis(2)}")
        )
      )
    ).onFalse(  # Stop motor once trigger-button is not pressed
      commands2.InstantCommand(
        lambda: self.endEffector.stopCoralMotors()
      )
    )

    
    self.EEECommandXboxController.a().onFalse(
      commands2.InstantCommand(
        lambda: self.endEffector.stopCoralMotors()
      )
    )

    self.buttonBoardCommandController.button(7).onFalse(
      MoveAlgaeArmToPosition(self.endEffector, constants.intakeConsts.algaeZeroPosition)
    )
    
    
    # Y button - Intake algae
    # self.EEECommandXboxController.y().whileTrue(AlgaeIntakeControl(self.endEffector, "intake"))
    self.EEECommandXboxController.y().onTrue(
      commands2.InstantCommand(
        lambda: self.endEffector.algae_intake_motor.set(-constants.intakeConsts.algaeIntakeSpeed)
      )
    )

    self.buttonBoardCommandController.button(5).onTrue(
      commands2.InstantCommand(
        lambda: self.endEffector.algae_intake_motor.set(-constants.intakeConsts.algaeIntakeSpeed)
      )
    )

    # B button - Eject algae
    # self.EEECommandXboxController.b().whileTrue(AlgaeIntakeControl(self.endEffector, "eject"))
    self.EEECommandXboxController.b().onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand(
          lambda: self.endEffector.algae_intake_motor.set(constants.intakeConsts.algaeIntakeSpeed)
        ),
        commands2.InstantCommand(
          lambda: print("eject")
        )
      )
    )

    # Axis 4 -> 1.0 - Eject algae
    commands2.button.Trigger(lambda: abs(self.buttonBoardCommandController.getRawAxis(2) - 1.0) < 0.1).onTrue(
      commands2.ParallelCommandGroup(
        commands2.InstantCommand(
          lambda: self.endEffector.algae_intake_motor.set(constants.intakeConsts.algaeIntakeSpeed)
        ),
        commands2.InstantCommand(
          lambda: print(f"eject algae - {self.buttonBoardCommandController.getRawAxis(2)}")
        )
      )
    ).onFalse(  # Stop motor once trigger-button is not pressed
      commands2.InstantCommand(
        lambda: self.endEffector.algae_intake_motor.set(0)
      )
    )
    
    # Stop algae eject when button is not pressed
    self.EEECommandXboxController.b().onFalse(
      commands2.InstantCommand(
        lambda: self.endEffector.algae_intake_motor.set(0)
      )
    )

    # Start button to test algae intake
    self.EEECommandXboxController.start().onTrue(TestAlgaeIntake(self.endEffector))

    # For left trigger - 45 degrees (π/4 radians)
    self.EEECommandXboxController.leftTrigger().onTrue(
      MoveAlgaeArmToPosition(self.endEffector, constants.intakeConsts.algaeArmFloorIntakeAngle)  # 45 degrees in radians
    )

    self.buttonBoardCommandController.button(4).onTrue(
      MoveAlgaeArmToPosition(self.endEffector, constants.intakeConsts.algaeArmFloorIntakeAngle)  # 45 degrees in radians
    )

    # For right trigger - 135 degrees (3π/4 radians)
    self.EEECommandXboxController.rightTrigger().onTrue(
      MoveAlgaeArmToPosition(self.endEffector, constants.intakeConsts.algaeArmReefIntakeAngle)  # 135 degrees in radians
    )

    self.buttonBoardCommandController.button(8).onTrue(
      MoveAlgaeArmToPosition(self.endEffector, constants.intakeConsts.algaeArmReefIntakeAngle)  # 135 degrees in radians
    )

    # stow the arm when down dpad is pressed
    self.EEECommandXboxController.povUp().onTrue(
      MoveAlgaeArmToPosition(self.endEffector, 3.1)  # about 180 degrees in radians
    )

    #'''self.buttonBoardCommandController.button(8).onTrue(
    #  MoveAlgaeArmToPosition(self.endEffector, 3.1)  # about 180 degrees in radians
    #  print('algae arm stow')
    #)
    

    # wait until coral is detected by the EE canrange 
    # then wait until coral is undetected by the funnel canrange 
    # then stop the intake motors
    self.EEECommandXboxController.x().onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(lambda: print("in")),
        commands2.InstantCommand(lambda: self.goToBaseLevel()),
        commands2.InstantCommand(lambda: self.endEffector.startCoralMotors()),
        WaitUntilCoralIsDetected(self.coralIsInRangeEE),
        WaitUntilCoralIsDetected(self.coralIsOutOfRangeFunnel),
        commands2.InstantCommand(lambda: self.endEffector.stopCoralMotors())
      )
    )

    self.buttonBoardCommandController.button(6).onTrue(
      commands2.SequentialCommandGroup(
        commands2.InstantCommand(lambda: print("in")),
        commands2.InstantCommand(lambda: self.goToBaseLevel()),
        commands2.InstantCommand(lambda: self.endEffector.startCoralMotors()),
        WaitUntilCoralIsDetected(self.coralIsInRangeEE),
        WaitUntilCoralIsDetected(self.coralIsOutOfRangeFunnel),
        commands2.InstantCommand(lambda: self.endEffector.stopCoralMotors()),
        print('coral intake, button 6')
      )
    )

    # Add debug mode toggle on Back/Select button
    self.EEECommandXboxController.back().onTrue(
      ToggleDebugMode(self.is_debug_mode)
    )

    # self.coralIntakeCommand = commands2.SequentialCommandGroup(
    #   WaitUntilCoralIsDetected(self.coralIsInRangeEE),
    #   WaitUntilCoralIsDetected(self.coralIsOutOfRangeFunnel),
    #   Intake(self.endEffector)
    # )

  def negateOdometry(self, pose: Pose2d):
    x = pose.X()
    y = pose.Y()
    r = pose.rotation()
    return Pose2d(Translation2d(-x, -y), r)
  
  def robotInit(self):
    """
    This function is called upon program startup and
    should be used for any initialization code.
    """
    print("=== robotInit()")

    self.ledController = LED()

    self.manualDrive = True
    # camera = CameraServer.startAutomaticCapture()
    # camera.setFPS(15)

    # wpinet.PortForwarder.getInstance().add(5801, "172.29.2.21", 5801)
    # wpinet.PortForwarder.getInstance().add(5802, "172.29.1.31", 5801)

    wpinet.PortForwarder.getInstance().add(5801, "limelight-lside.local", 5802)
    wpinet.PortForwarder.getInstance().add(5802, "limelight-rside.local", 5802)

    self.autonomousCommand = None # Remove the default driveForward command

    self.is_debug_mode = [False]

    self.drivingXboxController = wpilib.XboxController(0)
    self.drivingCommandXboxController = commands2.button.CommandXboxController(0)
    self.EEEXboxController = wpilib.XboxController(1)
    self.EEECommandXboxController = commands2.button.CommandXboxController(1)
    self.buttonBoardController = wpilib.Joystick(2)
    self.buttonBoardCommandController = commands2.button.CommandJoystick(2)

    self.button_debugger = SimpleButtonBoardDebug()

    print("=============== JOYSTICK INFO =====================")
    print(f"Joystick 0 name: {wpilib.DriverStation.getJoystickName(0)}")
    print(f"Joystick 1 name: {wpilib.DriverStation.getJoystickName(1)}")
    print(f"Joystick 2 name: {wpilib.DriverStation.getJoystickName(2)}")
          
    
    self.drivetrain = DriveTrain()
    self.elevatorController = Elevator()
    self.endEffector = EndEffector()
    # self.vision = SimpleVisionSubsystem()
    self.llController = LimelightSubsystem()
    #self.elevator = Elevator()

    # initialize network tables
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("datatable")
    self.controllerXPub = table.getDoubleTopic("controller x").publish()
    self.controllerYPub = table.getDoubleTopic("controller y").publish()
    self.robotPosition = table.getStructTopic("robot pose", Pose2d).publish()
    self.headingValue = table.getDoubleTopic("heading").publish()
    self.llPredictionPosition = table.getStructTopic("April Tag Position", Pose3d).publish()
    self.origin = table.getStructTopic("origin", Pose3d).publish()
    self.origin.set(Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0)))
    self.negatedRobotPosition = table.getStructTopic("Negated Odometry", Pose2d).publish()
    self.slowScaler = 1

    self.EEEPressedButtons = [False, False, False, False] # left trigger, right trigger, left bumper, right bumper

    self.scheduler = commands2.CommandScheduler.getInstance()

    self.canRangeFunnel = CANrange(constants.CANIDs.CanRangeFunnel)
    self.canRangeEE = CANrange(constants.CANIDs.CanRangeEE)

    

    print("robotInit()")

  def robotPeriodic(self):
    # print("robotPeriodic()")
    targetPose = self.llController.getTargetPose()
    if targetPose: # check if there are april tags detected
      self.llPredictionPosition.set(self.llController.limelightPose2AdvantageScopePose(targetPose))
    else:
      self.llPredictionPosition.set(Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0)))
        
    if self.llController.limelightLeftDetectsTag() or self.llController.limelightRightDetectsTag():
      self.ledController.changeStates(constants.RobotStates.Tag)
    else:
      self.ledController.changeStates(constants.RobotStates.noTag)

    self.button_debugger.update()
    
  def autonomousInit(self):
    """This function is run once each time the robot enters autonomous mode."""
    print("autonomousInit()")

    # Create a command to drive forward 4 feet (converted to meters)
    feet_to_meters = 0.3048  # 1 foot = 0.3048 meters
    distance_feet = constants.autoConsts.autoDriveDistance
    distance_meters = distance_feet * feet_to_meters
    
    # Reset the drivetrain odometry before starting auto
    self.drivetrain.resetHarder()

    # Make sure motors are not in motion
    self.drivetrain.stopMotors()
    
    # Create and schedule the autonomous command
    self.autonomousCommand = DriveDistance(self.drivetrain, distance_meters, 0.3)
    self.scheduler.schedule(self.autonomousCommand)
    
    # print(f"Starting autonomous: Driving forward {distance_feet} feet ({distance_meters:.2f} meters)")

  def autonomousPeriodic(self):
    """This function is called periodically during autonomous."""
    print("autonomousPeriodic()")
    pass

  def disabledInit(self):
    """This function is called initially when disabledd"""
    print("disabledInit()")
    self.endEffector.stopAllMotors()

  def disabledPeriodic(self):
    pass

  def teleopInit(self): 
    """This function is called once each time the robot enters teleoperated mode."""
    print("teleopInit()")
    self.teleopCounter = 0
    self.stopRumble()
    self.drivetrain.resetHarder()
    self.systemTempCheck()
    self.configureButtonBindings()
    self.elevatorController.zeroElevator()
    self.manualDrive = True
    # self.endEffector.algaeDestination = 1.57
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
    self.teleopCounter += 1
    # self.drivetrain.stopMotors()
    # print("teleopPeriodic()")
    xSpeed, ySpeed = MyRobot.distanceCorrectedInputCurve(self.drivingXboxController.getLeftY(), self.drivingXboxController.getLeftX())
    # xSpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftY())
    # ySpeed = MyRobot.inputCurve(self.drivingXboxController.getLeftX())
    # print('X Speed - ' + str(xSpeed))
    # print('Y Speed - ' + str(ySpeed))
    self.controllerXPub.set(xSpeed * self.slowScaler)
    self.controllerYPub.set(ySpeed * self.slowScaler)

    if self.is_debug_mode[0]:
      if (self.teleopCounter % 50) == 0:  # Only print every ~1 second (assuming 50Hz loop)
          print("*** DEBUG MODE ACTIVE ***")

    self.scheduler.schedule(commands2.ConditionalCommand(
      commands2.InstantCommand(lambda: self.flopArm()),
      commands2.InstantCommand(lambda: self.unFlopArm()),
      lambda: self.shouldFlopArm()
    ))
    # print(self.shouldFlopArm(), self.endEffector.algae_intake_motor.getOutputCurrent() > 15)
    # print('\nAlgae Arm Angle:')
    # print(self.endEffector.getAlgaeArmAngle())
    #print('\nAlgae Motor Rotations')
    #print(self.endEffector.getAlgaeArmRotations())
    # print('\nSetpoint:')
    # print(self.endEffector.algaePID.getSetpoint())
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
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * self.slowScaler, ySpeed * self.slowScaler, -tSpeed * self.slowScaler, Rotation2d(heading))
    if self.manualDrive:
      self.drivetrain.manualDriveFromChassisSpeeds(speeds)
    self.robotPosition.set(self.drivetrain.currentPosition)
    self.negatedRobotPosition.set(self.negateOdometry(self.drivetrain.currentPosition))

    self.scheduler.run()

    # print(self.elevatorController.currentLevel, self.coralIsOutOfRangeFunnel())
    # print(self.canRangeFunnel.get_distance().value_as_double)
    # important print statement
    # print(self.canRangeEE.get_distance().value_as_double)
    '''
    print(f"""
target in: {2 * constants.convert.rot2in(self.elevatorController.destination)}
current in: {2 * constants.convert.rot2in(self.elevatorController.getElevatorPosition())}
target rot: {self.elevatorController.destination}
current rot: {self.elevatorController.getElevatorPosition()}
current: {self.elevatorController.REM.getOutputCurrent()}""")'
    '''

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