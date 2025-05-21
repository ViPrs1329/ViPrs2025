# DriveSubsystem.py
#
# 

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import SwerveModulePosition
from wpimath.kinematics import SwerveModuleState
from wpilib import DriverStation
from wpilib import Field2d
from wpilib import SmartDashboard

from phoenix6.hardware import Pigeon2

from commands2 import Subsystem
from constants import CANIDs
from constants import Drive

class SwerveModule:

    def __init__(self):
        self.currentPosition: SwerveModulePosition = SwerveModulePosition()
        self.currentState: SwerveModuleState = SwerveModuleState()

    def getPosition(self) -> SwerveModulePosition:
        return self.currentPosition
    
    def getState(self) -> SwerveModuleState:
        return self.currentState
    
    def setTargetState(self, targetState: SwerveModuleState) -> None:
        # optimize the state
        self.currentState.optimize(self.currentState.angle)

        # 0.02 is 50hz = rate at which main controll loop runs
        self.currentPosition = SwerveModulePosition(
            self.currentPosition.distance + (self.currentState.speed * 0.02),
            self.currentState.angle
        )

class DriveSubsystem(Subsystem):

    def __init__(self):
        super().__init__()

        # Initialize swerve modules and other things...
        self.gyro: Pigeon2 = Pigeon2(CANIDs.pigeon)
        self.field: Field2d = Field2d()
        self.modules: list[SwerveModule] = [
            SwerveModule(),
            SwerveModule(),
            SwerveModule(),
            SwerveModule()
        ]
        self.kinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(
            Drive.Consts.flModuleOffset,
            Drive.Consts.frModuleOffset,
            Drive.Consts.blModuleOffset,
            Drive.Consts.brModuleOffset
        )
        self.odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.getPositions()
        )

        try:
            self.config: RobotConfig = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                self.getPose,
                self.resetPose,
                self.getSpeeds,
                self.driveRobotRelative,
                PPHolonomicDriveController(
                    Drive.Consts.translationConstants,
                    Drive.Consts.rotationConstants
                ),
                self.config,
                self.shouldFlipPath,
                self
            )
        except Exception as inst:
            print("Failed to load PathPlanner config and configure AutoBuilder", inst)
        
        SmartDashboard.putData("Field", self.field)

    def periodic(self):
        self.odometry.update(self.gyro.getRotation2d(), self.getPositions())
        self.field.setRobotPose(self.getPose())

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()
    
    def resetPose(self, pose: Pose2d) -> None:
        print(pose)
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.getPositions(),
            pose
        )

    def getSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getModuleStates())
    
    def driveFieldRelative(self, fieldRelativeSpeeds) -> None:
        self.driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                self.getPose().rotation()
            )
        )

    def driveRobotRelative(self, robotRelativeSpeeds: ChassisSpeeds, ff: DriveFeedforwards | None = None) -> None:
        targetSpeeds: ChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02)
        targetStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState] = self.kinematics.toSwerveModuleStates(targetSpeeds)
        self.setStates(targetStates)

    def setStates(self, targetStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        targetStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, Drive.Consts.maxModuleSpeed)
        for i in range(len(self.modules)):
            self.modules[i].setTargetState(targetStates[i])

    def getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        states = []
        for i in range(len(self.modules)):
            states.append(self.modules[i].getState())

        return tuple(states)
    
    def getPositions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        positions = []
        for i in range(len(self.modules)):
            positions.append(self.modules[i])

        return tuple(positions)
    
    def shouldFlipPath(self):
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            return True
        else:
            return False
        
    def updateSpeeds(self):
        pass
            
    def updateHardware(self):
        # This method gets called periodically to update hardware state
        pass

    def cacheSensors(self):
        # This is called periodically to cache sensor data
        # so that we don't clog up the CAN network
        pass