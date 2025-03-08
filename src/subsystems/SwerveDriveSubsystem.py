import rev
import math
import commands2
import wpilib

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation, SmartDashboard
from wpimath import controller

from constants import CANIDs, driveConsts

from phoenix6.hardware import CANcoder, Pigeon2


def lratio(angle):
    """Converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    """Converts CANcoder absolute position to radians"""
    return (something/.5)*-math.pi

def deg2Rot2d(deg) -> Rotation2d:
    """Converts degrees to Rotation2d"""
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)

def ticks2radODOMETRY(something):
    """Converts CANcoder rotations to radians for odometry"""
    # units are in rotations
    return something * 2 * math.pi

def getSwerveModPos(rotEnc: CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    """Creates a SwerveModulePosition from encoder readings"""
    drive_position = (driveEnc.getPosition()/driveConsts.driveGearRatio) * driveConsts.wheelCircumference
    rotation_position = Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    return SwerveModulePosition(drive_position, rotation_position)

class DriveTrain(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        
        # Debug mode flag
        self.debug_mode = True
        
        self.robotOdometryPosition = Pose2d()
        self.combinedPosition = Pose2d()

        # Set up controllers
        self.drivingXboxController = wpilib.XboxController(0)

        # Drivetrain init 
        self.debug_print("Initializing swerve drive motors and encoders")
        
        # Rotation motors
        self.backLeftRotation = rev.SparkMax(CANIDs.SwerveModuleRotation2, rev.SparkMax.MotorType.kBrushless)
        self.backRightRotation = rev.SparkMax(CANIDs.SwerveModuleRotation3, rev.SparkMax.MotorType.kBrushless)
        self.frontLeftRotation = rev.SparkMax(CANIDs.SwerveModuleRotation1, rev.SparkMax.MotorType.kBrushless)
        self.frontRightRotation = rev.SparkMax(CANIDs.SwerveModuleRotation4, rev.SparkMax.MotorType.kBrushless)

        # Drive motors
        self.backLeftDrive = rev.SparkMax(CANIDs.SwerveModuleDrive2, rev.SparkMax.MotorType.kBrushless)
        self.backRightDrive = rev.SparkMax(CANIDs.SwerveModuleDrive3, rev.SparkMax.MotorType.kBrushless)
        self.frontLeftDrive = rev.SparkMax(CANIDs.SwerveModuleDrive1, rev.SparkMax.MotorType.kBrushless)
        self.frontRightDrive = rev.SparkMax(CANIDs.SwerveModuleDrive4, rev.SparkMax.MotorType.kBrushless)

        # Set motor inversion if needed
        if driveConsts.invertFrontLeftDrive:
            self.frontLeftDrive.setInverted(True)
        if driveConsts.invertFrontRightDrive:
            self.frontRightDrive.setInverted(True)
        if driveConsts.invertBackLeftDrive:
            self.backLeftDrive.setInverted(True)
        if driveConsts.invertBackRightDrive:
            self.backRightDrive.setInverted(True)
            
        if driveConsts.invertFrontLeftRotation:
            self.frontLeftRotation.setInverted(True)
        if driveConsts.invertFrontRightRotation:
            self.frontRightRotation.setInverted(True)
        if driveConsts.invertBackLeftRotation:
            self.backLeftRotation.setInverted(True)
        if driveConsts.invertBackRightRotation:
            self.backRightRotation.setInverted(True)

        # Set up motor configs
        self.debug_print("Configuring motor settings")
        
        # Create motor configs
        self.backLeftRotationConfig = rev.SparkBaseConfig()
        self.backLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.backLeftRotationConfig.smartCurrentLimit(driveConsts.rotationCurrentLimit)
        
        self.backRightRotationConfig = rev.SparkBaseConfig()
        self.backRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.backRightRotationConfig.smartCurrentLimit(driveConsts.rotationCurrentLimit)
        
        self.frontLeftRotationConfig = rev.SparkBaseConfig()
        self.frontLeftRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.frontLeftRotationConfig.smartCurrentLimit(driveConsts.rotationCurrentLimit)
        
        self.frontRightRotationConfig = rev.SparkBaseConfig()
        self.frontRightRotationConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.frontRightRotationConfig.smartCurrentLimit(driveConsts.rotationCurrentLimit)

        self.backLeftDriveConfig = rev.SparkBaseConfig()
        self.backLeftDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.backLeftDriveConfig.smartCurrentLimit(driveConsts.driveCurrentLimit)
        
        self.backRightDriveConfig = rev.SparkBaseConfig()
        self.backRightDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.backRightDriveConfig.smartCurrentLimit(driveConsts.driveCurrentLimit)
        
        self.frontLeftDriveConfig = rev.SparkBaseConfig()
        self.frontLeftDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.frontLeftDriveConfig.smartCurrentLimit(driveConsts.driveCurrentLimit)
        
        self.frontRightDriveConfig = rev.SparkBaseConfig()
        self.frontRightDriveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.frontRightDriveConfig.smartCurrentLimit(driveConsts.driveCurrentLimit)
        
        # Apply configs to motors
        self.backLeftRotation.configure(self.backLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.backRightRotation.configure(self.backRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.frontLeftRotation.configure(self.frontLeftRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.frontRightRotation.configure(self.frontRightRotationConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self.backLeftDrive.configure(self.backLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.backRightDrive.configure(self.backRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.frontLeftDrive.configure(self.frontLeftDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
        self.frontRightDrive.configure(self.frontRightDriveConfig, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # Get drive encoders
        self.debug_print("Getting motor encoders")
        self.frontRightDriveEnc = self.frontRightDrive.getEncoder()
        self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder()
        self.backRightDriveEnc = self.backRightDrive.getEncoder()
        self.backLeftDriveEnc = self.backLeftDrive.getEncoder()
        
        # Set encoder conversion factors
        self.frontRightDriveEnc.setPositionConversionFactor(driveConsts.driveEncoderPositionFactor)
        self.frontLeftDriveEnc.setPositionConversionFactor(driveConsts.driveEncoderPositionFactor)
        self.backRightDriveEnc.setPositionConversionFactor(driveConsts.driveEncoderPositionFactor)
        self.backLeftDriveEnc.setPositionConversionFactor(driveConsts.driveEncoderPositionFactor)
        
        self.frontRightDriveEnc.setVelocityConversionFactor(driveConsts.driveEncoderVelocityFactor)
        self.frontLeftDriveEnc.setVelocityConversionFactor(driveConsts.driveEncoderVelocityFactor)
        self.backRightDriveEnc.setVelocityConversionFactor(driveConsts.driveEncoderVelocityFactor)
        self.backLeftDriveEnc.setVelocityConversionFactor(driveConsts.driveEncoderVelocityFactor)

        # Set up angle encoders (CANcoders)
        self.debug_print("Setting up CANcoders")
        self.FrightEnc = CANcoder(CANIDs.EncoderModuleRotation4)
        self.FleftEnc = CANcoder(CANIDs.EncoderModuleRotation3)
        self.BrightEnc = CANcoder(CANIDs.EncoderModuleRotation2)
        self.BleftEnc = CANcoder(CANIDs.EncoderModuleRotation1)

        # PID Setup
        self.debug_print("Configuring rotation PID controllers")
        self.BleftPID = controller.PIDController(driveConsts.kP, driveConsts.kI, driveConsts.kD)
        self.BleftPID.enableContinuousInput(-.5, .5)
        self.BleftPID.setSetpoint(0.0)
        
        self.BrightPID = controller.PIDController(driveConsts.kP, driveConsts.kI, driveConsts.kD)
        self.BrightPID.enableContinuousInput(-.5, .5)
        self.BrightPID.setSetpoint(0.0)
        
        self.FleftPID = controller.PIDController(driveConsts.kP, driveConsts.kI, driveConsts.kD)
        self.FleftPID.enableContinuousInput(-.5, .5)
        self.FleftPID.setSetpoint(0.0)
        
        self.FrightPID = controller.PIDController(driveConsts.kP, driveConsts.kI, driveConsts.kD)
        self.FrightPID.enableContinuousInput(-.5, .5)
        self.FrightPID.setSetpoint(0.0)

        # Gyro initialization
        self.debug_print("Initializing gyro")
        self.gyro = Pigeon2(CANIDs.Gyro)
        self.resetGyro()

        # Kinematics setup
        self.debug_print("Setting up swerve drive kinematics")
        frontLeftLocation = Translation2d(driveConsts.wheelBase/2, driveConsts.trackWidth/2)
        frontRightLocation = Translation2d(driveConsts.wheelBase/2, -driveConsts.trackWidth/2)
        backLeftLocation = Translation2d(-driveConsts.wheelBase/2, driveConsts.trackWidth/2)
        backRightLocation = Translation2d(-driveConsts.wheelBase/2, -driveConsts.trackWidth/2)

        self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        # Initialize odometry
        self.debug_print("Setting up odometry")
        self.resetOdometry()

    def debug_print(self, message):
        """Print debug message if debug mode is enabled"""
        if self.debug_mode:
            print(f"[SwerveDrive Debug] {message}")

    def resetGyro(self):
        """Reset the gyro to zero"""
        self.debug_print("Resetting gyro")
        self.gyro.set_yaw(0)

    def resetOdometry(self, pose=Pose2d()):
        """Reset the odometry with an optional pose"""
        self.debug_print(f"Resetting odometry to {pose}")
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            deg2Rot2d(self.gyro.get_yaw().value_as_double),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            pose
        )

    def resetHarder(self, initialPose=Pose2d()):
        """Reset gyro and odometry with an optional pose"""
        self.resetGyro()
        self.resetOdometry(initialPose)
  
    def getPose(self):
        """Get the current robot pose from odometry"""
        return self.odometry.getPose()
  
    def shouldFlipPath(self): 
        """Check if path should be mirrored based on alliance color"""
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
  
    def getChassisSpeed(self) -> ChassisSpeeds:
        """Get the last commanded chassis speeds"""
        return self.lastChassisSpeed
  
    def updateOdometry(self) -> None:
        """Update odometry with current encoder values"""
        yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double)
        
        # Get the current module positions
        modulePositions = (
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
        )
        
        # Update odometry
        self.robotOdometryPosition = self.odometry.update(yaw, modulePositions)
        
        # Get rotation from pose
        robotRotationPose = self.odometry.getPose()
        
        # Combine position and rotation
        self.combinedPosition = Pose2d(
            x=self.robotOdometryPosition.x, 
            y=self.robotOdometryPosition.y, 
            rotation=robotRotationPose.rotation()
        )
        
        # Log odometry data to SmartDashboard
        if self.debug_mode:
            SmartDashboard.putNumber("Odometry X", self.combinedPosition.x)
            SmartDashboard.putNumber("Odometry Y", self.combinedPosition.y)
            SmartDashboard.putNumber("Odometry Rot", self.combinedPosition.rotation().degrees())

    def periodic(self) -> None:
        """Called periodically by the command scheduler"""
        self.updateOdometry()
        
        # Log encoder values to SmartDashboard if in debug mode
        if self.debug_mode:
            SmartDashboard.putNumber("FL Angle", self.FleftEnc.get_absolute_position().value_as_double * 360)
            SmartDashboard.putNumber("FR Angle", self.FrightEnc.get_absolute_position().value_as_double * 360)
            SmartDashboard.putNumber("BL Angle", self.BleftEnc.get_absolute_position().value_as_double * 360)
            SmartDashboard.putNumber("BR Angle", self.BrightEnc.get_absolute_position().value_as_double * 360)
            
            SmartDashboard.putNumber("Gyro", self.gyro.get_yaw().value_as_double)

    def resetMotors(self) -> None:
        """Stop all motors"""
        self.debug_print("Stopping all drive motors")
        self.stopMotors()

    def manualDriveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        """
        Drive the robot using chassis speeds.
        This is the main drive method used from robot.py
        """
        # Store the commanded speeds for telemetry
        self.lastChassisSpeed = speeds
        
        # Log the input speeds
        if self.debug_mode:
            SmartDashboard.putNumber("Input vx", speeds.vx)
            SmartDashboard.putNumber("Input vy", speeds.vy)
            SmartDashboard.putNumber("Input omega", speeds.omega)
            self.debug_print(f"Input speeds: vx={speeds.vx}, vy={speeds.vy}, omega={speeds.omega}")
        
        # Convert speeds to module states
        # Note: Negate values based on coordinate system conventions
        adjusted_speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega)
        moduleStates = self.kinematics.toSwerveModuleStates(adjusted_speeds)
        
        # Get individual module states
        frontLeft, frontRight, backLeft, backRight = moduleStates
        
        # Log raw module states (before optimization)
        if self.debug_mode:
            self.debug_print(f"Raw FL: speed={frontLeft.speed:.2f}, angle={frontLeft.angle.degrees():.2f} deg")
            self.debug_print(f"Raw FR: speed={frontRight.speed:.2f}, angle={frontRight.angle.degrees():.2f} deg")
            self.debug_print(f"Raw BL: speed={backLeft.speed:.2f}, angle={backLeft.angle.degrees():.2f} deg")
            self.debug_print(f"Raw BR: speed={backRight.speed:.2f}, angle={backRight.angle.degrees():.2f} deg")
        
        # Optimize module states to minimize wheel rotation
        frontLeftOptimized = SwerveModuleState.optimize(
            frontLeft, Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position().value_as_double))
        )
        frontRightOptimized = SwerveModuleState.optimize(
            frontRight, Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position().value_as_double))
        )
        backLeftOptimized = SwerveModuleState.optimize(
            backLeft, Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position().value_as_double))
        )
        backRightOptimized = SwerveModuleState.optimize(
            backRight, Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position().value_as_double))
        )
        
        # Log optimized module states
        if self.debug_mode:
            self.debug_print(f"Optimized FL: speed={frontLeftOptimized.speed:.2f}, angle={frontLeftOptimized.angle.degrees():.2f} deg")
            self.debug_print(f"Optimized FR: speed={frontRightOptimized.speed:.2f}, angle={frontRightOptimized.angle.degrees():.2f} deg")
            self.debug_print(f"Optimized BL: speed={backLeftOptimized.speed:.2f}, angle={backLeftOptimized.angle.degrees():.2f} deg")
            self.debug_print(f"Optimized BR: speed={backRightOptimized.speed:.2f}, angle={backRightOptimized.angle.degrees():.2f} deg")
        
        # Calculate PID outputs for rotation motors
        blrSpeed = -self.BleftPID.calculate(
            self.BleftEnc.get_absolute_position().value_as_double, 
            lratio(backLeftOptimized.angle.radians())
        )
        flrSpeed = -self.FleftPID.calculate(
            self.FleftEnc.get_absolute_position().value_as_double, 
            lratio(frontLeftOptimized.angle.radians())
        )
        brrSpeed = -self.BrightPID.calculate(
            self.BrightEnc.get_absolute_position().value_as_double, 
            lratio(backRightOptimized.angle.radians())
        )
        frrSpeed = -self.FrightPID.calculate(
            self.FrightEnc.get_absolute_position().value_as_double, 
            lratio(frontRightOptimized.angle.radians())
        )
        
        # Log module setpoints and current positions
        if self.debug_mode:
            self.debug_print(f"Current BL pos: {self.BleftEnc.get_absolute_position().value_as_double:.4f}, setpoint: {lratio(backLeftOptimized.angle.radians()):.4f}")
            self.debug_print(f"Current FL pos: {self.FleftEnc.get_absolute_position().value_as_double:.4f}, setpoint: {lratio(frontLeftOptimized.angle.radians()):.4f}")
            self.debug_print(f"Current BR pos: {self.BrightEnc.get_absolute_position().value_as_double:.4f}, setpoint: {lratio(backRightOptimized.angle.radians()):.4f}")
            self.debug_print(f"Current FR pos: {self.FrightEnc.get_absolute_position().value_as_double:.4f}, setpoint: {lratio(frontRightOptimized.angle.radians()):.4f}")
        
        # Get module drive speeds
        bldSpeed = backLeftOptimized.speed
        brdSpeed = backRightOptimized.speed
        fldSpeed = frontLeftOptimized.speed
        frdSpeed = frontRightOptimized.speed
        
        # Apply deadzone to drive speeds
        if abs(bldSpeed) < driveConsts.driveDeadzone:
            bldSpeed = 0
        if abs(brdSpeed) < driveConsts.driveDeadzone:
            brdSpeed = 0
        if abs(fldSpeed) < driveConsts.driveDeadzone:
            fldSpeed = 0
        if abs(frdSpeed) < driveConsts.driveDeadzone:
            frdSpeed = 0
            
        # Apply deadzone to rotation speeds
        if abs(blrSpeed) < driveConsts.rotationDeadzone:
            blrSpeed = 0
        if abs(brrSpeed) < driveConsts.rotationDeadzone:
            brrSpeed = 0
        if abs(flrSpeed) < driveConsts.rotationDeadzone:
            flrSpeed = 0
        if abs(frrSpeed) < driveConsts.rotationDeadzone:
            frrSpeed = 0
        
        # Log final motor commands
        if self.debug_mode:
            self.debug_print(f"Motor Commands:")
            self.debug_print(f"Drive: BL={bldSpeed:.2f}, BR={brdSpeed:.2f}, FL={fldSpeed:.2f}, FR={frdSpeed:.2f}")
            self.debug_print(f"Rotation: BL={blrSpeed:.2f}, BR={brrSpeed:.2f}, FL={flrSpeed:.2f}, FR={frrSpeed:.2f}")
            
            # Output to SmartDashboard for visualization
            SmartDashboard.putNumber("BL Drive", bldSpeed)
            SmartDashboard.putNumber("BR Drive", brdSpeed)
            SmartDashboard.putNumber("FL Drive", fldSpeed)
            SmartDashboard.putNumber("FR Drive", frdSpeed)
            
            SmartDashboard.putNumber("BL Rotation", blrSpeed)
            SmartDashboard.putNumber("BR Rotation", brrSpeed)
            SmartDashboard.putNumber("FL Rotation", flrSpeed)
            SmartDashboard.putNumber("FR Rotation", frrSpeed)
        
        # Set the motor speeds
        self.backLeftRotation.set(blrSpeed)
        self.backRightRotation.set(brrSpeed)
        self.frontLeftRotation.set(flrSpeed)
        self.frontRightRotation.set(frrSpeed)

        self.backLeftDrive.set(-bldSpeed)  # Note: Negative based on motor orientation
        self.backRightDrive.set(brdSpeed)
        self.frontLeftDrive.set(fldSpeed)
        self.frontRightDrive.set(frdSpeed)

    def driveFromChassisSpeeds(self, xSpeed, ySpeed, rotSpeed):
        """
        Alternative drive method that takes separate x, y, and rotation inputs.
        This is used for autonomous and other command-based control.
        """
        speeds = ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
        self.manualDriveFromChassisSpeeds(speeds)

    def stopMotors(self):
        """Stop all drive and rotation motors"""
        self.debug_print("Stopping all motors")
        self.frontLeftDrive.set(0)
        self.frontRightDrive.set(0)
        self.backLeftDrive.set(0)
        self.backRightDrive.set(0)

        self.frontLeftRotation.set(0)
        self.frontRightRotation.set(0)
        self.backLeftRotation.set(0)
        self.backRightRotation.set(0)