# SwerveDriveSubsystem.py
import rev
import math
import commands2
import wpilib

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpimath import controller

from constants import CANIDs, driveConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory

from phoenix6.hardware import CANcoder, Pigeon2


def lratio(angle):
    """converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    return (something/.5)*-math.pi

def deg2Rot2d(deg) -> Rotation2d:
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)

def ticks2radODOMETRY(something):
    # units are in rotations
    return something * 2* math.pi

def getSwerveModPos(rotEnc : CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    return SwerveModulePosition(
                                        # 2pi*r
        (driveEnc.getPosition()/6.75)*0.31918580816,
        Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    )

class DriveTrain(commands2.Subsystem):
    class Cache:
        def __init__(self):
            # Drive encoder positions
            self.frontLeftDrivePos = 0.0
            self.frontRightDrivePos = 0.0
            self.backLeftDrivePos = 0.0
            self.backRightDrivePos = 0.0
            
            # CANcoder absolute positions
            self.frontLeftAbsPos = 0.0
            self.frontRightAbsPos = 0.0
            self.backLeftAbsPos = 0.0
            self.backRightAbsPos = 0.0
            
            # Gyro data
            self.yaw = 0.0
            self.pitch = 0.0
            self.roll = 0.0
            
            # Motor currents
            self.frontLeftDriveCurrent = 0.0
            self.frontRightDriveCurrent = 0.0
            self.backLeftDriveCurrent = 0.0
            self.backRightDriveCurrent = 0.0
            self.frontLeftRotCurrent = 0.0
            self.frontRightRotCurrent = 0.0
            self.backLeftRotCurrent = 0.0
            self.backRightRotCurrent = 0.0
            
            # Call counters for less frequent sensor reads
            self.currentCounter = 0
    
    def __init__(self) -> None:
        super().__init__()
        
        # Initialize cache
        self.cache = self.Cache()
        
        self.robotOdometryPosition = Pose2d()
        self.combinedPosition = Pose2d()

        # Create configurations for motors
        drive_config = SparkMaxFactory.Configuration()
        drive_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        drive_config.current_limit = driveConsts.currentLimit
        drive_config.voltage_comp_enabled = True
        drive_config.voltage_comp_saturation = 12.0
        
        rotation_config = SparkMaxFactory.Configuration()
        rotation_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
        rotation_config.current_limit = driveConsts.currentLimit
        rotation_config.voltage_comp_enabled = True
        
        # Create motors using the factory to reduce CAN traffic
        self.frontLeftDrive = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleDrive1, drive_config)
        self.frontRightDrive = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleDrive4, drive_config)
        self.backLeftDrive = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleDrive2, drive_config)
        self.backRightDrive = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleDrive3, drive_config)
        
        self.frontLeftRotation = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleRotation1, rotation_config)
        self.frontRightRotation = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleRotation4, rotation_config)
        self.backLeftRotation = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleRotation2, rotation_config)
        self.backRightRotation = SparkMaxFactory.createSparkMax(CANIDs.SwerveModuleRotation3, rotation_config)

        # Drive encoders
        self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder()
        self.frontRightDriveEnc = self.frontRightDrive.getEncoder()
        self.backLeftDriveEnc = self.backLeftDrive.getEncoder()
        self.backRightDriveEnc = self.backRightDrive.getEncoder()

        # CANcoders for absolute position
        self.FleftEnc = CANcoder(CANIDs.EncoderModuleRotation3)
        self.FrightEnc = CANcoder(CANIDs.EncoderModuleRotation4)
        self.BleftEnc = CANcoder(CANIDs.EncoderModuleRotation1)
        self.BrightEnc = CANcoder(CANIDs.EncoderModuleRotation2)

        # PID Setup
        Kp = 4
        self.BleftPID = controller.PIDController(Kp, 0, 0)
        self.BleftPID.enableContinuousInput(-.5, .5)
        self.BleftPID.setSetpoint(0.0)
        
        self.BrightPID = controller.PIDController(Kp, 0, 0)
        self.BrightPID.enableContinuousInput(-.5, .5)
        self.BrightPID.setSetpoint(0.0)
        
        self.FleftPID = controller.PIDController(Kp, 0, 0)
        self.FleftPID.enableContinuousInput(-.5, .5)
        self.FleftPID.setSetpoint(0.0)
        
        self.FrightPID = controller.PIDController(Kp, 0, 0)
        self.FrightPID.enableContinuousInput(-.5, .5)
        self.FrightPID.setSetpoint(0.0)

        # Gyro initialization
        self.gyro = Pigeon2(CANIDs.PigeonID)
        self.gyro.set_yaw(0)

        # Kinematics - swerve module locations
        lv = 0.381  # location value (distance from center to module)
        frontrightlocation = Translation2d(lv, lv) 
        frontleftlocation = Translation2d(lv, -lv) 
        backleftlocation = Translation2d(-lv, -lv)         
        backrightlocation = Translation2d(-lv, lv)    

        self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )

        # Initialize odometry
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            Pose2d()
        )
        
        # Initialize sensor cache
        self.cacheSensors()

    def cacheSensors(self):
        """Cache all sensor values to reduce CAN traffic"""
        # Always cache encoder positions
        try:
            self.cache.frontLeftDrivePos = self.frontLeftDriveEnc.getPosition()
            self.cache.frontRightDrivePos = self.frontRightDriveEnc.getPosition()
            self.cache.backLeftDrivePos = self.backLeftDriveEnc.getPosition()
            self.cache.backRightDrivePos = self.backRightDriveEnc.getPosition()
            
            # Cache CANcoder absolute positions
            self.cache.frontLeftAbsPos = self.FleftEnc.get_absolute_position()._value
            self.cache.frontRightAbsPos = self.FrightEnc.get_absolute_position()._value
            self.cache.backLeftAbsPos = self.BleftEnc.get_absolute_position()._value
            self.cache.backRightAbsPos = self.BrightEnc.get_absolute_position()._value
            
            # Cache gyro data
            self.cache.yaw = self.gyro.get_yaw().value_as_double
            self.cache.pitch = self.gyro.get_pitch().value_as_double
            self.cache.roll = self.gyro.get_roll().value_as_double
            
            # Cache current readings less frequently
            if self.cache.currentCounter == 0:
                self.cache.frontLeftDriveCurrent = self.frontLeftDrive.getOutputCurrent()
                self.cache.frontRightDriveCurrent = self.frontRightDrive.getOutputCurrent()
                self.cache.backLeftDriveCurrent = self.backLeftDrive.getOutputCurrent()
                self.cache.backRightDriveCurrent = self.backRightDrive.getOutputCurrent()
                self.cache.frontLeftRotCurrent = self.frontLeftRotation.getOutputCurrent()
                self.cache.frontRightRotCurrent = self.frontRightRotation.getOutputCurrent()
                self.cache.backLeftRotCurrent = self.backLeftRotation.getOutputCurrent()
                self.cache.backRightRotCurrent = self.backRightRotation.getOutputCurrent()
                
            self.cache.currentCounter = (self.cache.currentCounter + 1) % 10
        except Exception as e:
            print(f"Error caching sensor values: {e}")

    def resetHarder(self, initialPose = Pose2d()):
        """Reset odometry and gyro to a specific pose."""
        self.gyro.set_yaw(0)
        
        # Re-create odometry with current module positions
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            deg2Rot2d(self.cache.yaw),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            initialPose
        )
        
        # Update cached positions
        self.cacheSensors()
    
    def getPose(self):
        """Get the current estimated pose of the robot."""
        return self.combinedPosition
    
    def shouldFlipPath(self):
        """Determine if the path should be flipped for the red alliance."""
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def getChassisSpeed(self) -> ChassisSpeeds:
        """Get the last commanded chassis speeds."""
        return self.lastChassisSpeed
    
    def updateOdometry(self) -> None:
        """Update the odometry with the latest sensor readings."""
        try:
            # Use cached values for better performance
            yaw = deg2Rot2d(self.cache.yaw)
            
            # Update odometry with current module positions
            self.robotOdometryPosition = self.odometry.update(
                yaw,
                (
                    getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                    getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                    getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                    getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
                )
            )
            
            # Combine position and rotation information
            robotRotationPose = self.odometry.getPose()
            self.combinedPosition = Pose2d(
                x=self.robotOdometryPosition.x, 
                y=self.robotOdometryPosition.y, 
                rotation=robotRotationPose.rotation()
            )
        except Exception as e:
            print(f"Error updating odometry: {e}")

    def periodic(self) -> None:
        """Called periodically during all robot modes."""
        # Update sensor cache
        self.cacheSensors()
        
        # Update odometry
        self.updateOdometry()
        
        # Check motor temperatures and stop if overheating
        if self.checkMotorTemperatures():
            self.stopMotors()
            wpilib.DriverStation.reportError("EMERGENCY STOP: Motor overheating detected!", False)
        
        # Debug telemetry
        wpilib.SmartDashboard.putNumber("Swerve/GyroYaw", self.cache.yaw)

    def resetMotors(self) -> None:
        """Reset motors if needed."""
        pass  # Placeholder for future implementation

    
    def manualDriveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        """Drive the robot using manual chassis speeds."""
        try:
            if not self.checkSensors():
                print("WARNING: Sensor issues detected, limiting drive capabilities")
                # For safety, you could reduce max speeds here or disable rotation
                # For example:
                speeds = ChassisSpeeds(speeds.vx * 0.5, speeds.vy * 0.5, 0.0)  # Half speed, no rotation
            
            # Get the last commanded speeds
            last_speeds = self.lastChassisSpeed

            # Apply acceleration limiting
            speeds = self.limitAcceleration(last_speeds, speeds)

            # Store the last commanded speed
            self.lastChassisSpeed = speeds

            # Debug print to verify speeds
            print(f"Setting chassis speeds - vx: {speeds.vx:.2f}, vy: {speeds.vy:.2f}, omega: {speeds.omega:.2f}")
            
            # Convert to module states
            speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega)
            moduleStates = self.kinematics.toSwerveModuleStates(speeds)
            
            # Desaturate wheel speeds (limit to max speed)
            maxModSpeed = 4.1
            frontLeft, frontRight, backLeft, backRight = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                moduleStates, 
                maxModSpeed
            )

            # Optimize module states
            frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
            Rotation2d(ticks2rad(self.cache.frontLeftAbsPos)))
            frontRightOptimized = SwerveModuleState.optimize(frontRight,
            Rotation2d(ticks2rad(self.cache.frontRightAbsPos)))
            backLeftOptimized = SwerveModuleState.optimize(backLeft,
            Rotation2d(ticks2rad(self.cache.backLeftAbsPos)))
            backRightOptimized = SwerveModuleState.optimize(backRight,
            Rotation2d(ticks2rad(self.cache.backRightAbsPos)))

            # Calculate PID outputs using cached absolute positions
            blPidOutput = -self.BleftPID.calculate(self.cache.backLeftAbsPos, lratio(backLeftOptimized.angle.radians()))
            flPidOutput = -self.FleftPID.calculate(self.cache.frontLeftAbsPos, lratio(frontLeftOptimized.angle.radians()))
            brPidOutput = -self.BrightPID.calculate(self.cache.backRightAbsPos, lratio(backRightOptimized.angle.radians()))
            frPidOutput = -self.FrightPID.calculate(self.cache.frontRightAbsPos, lratio(frontRightOptimized.angle.radians()))

            # Set rotation motors with limited outputs
            self.backLeftRotation.set(max(min(blPidOutput, 1.0), -1.0))
            self.frontLeftRotation.set(max(min(flPidOutput, 1.0), -1.0))
            self.backRightRotation.set(max(min(brPidOutput, 1.0), -1.0))
            self.frontRightRotation.set(max(min(frPidOutput, 1.0), -1.0))

            # Apply voltage to drive motors proportional to desired speed
            maxVoltage = 13
            self.backLeftDrive.setVoltage(-(backLeftOptimized.speed/maxModSpeed)*maxVoltage)
            self.backRightDrive.setVoltage((backRightOptimized.speed/maxModSpeed)*maxVoltage)
            self.frontLeftDrive.setVoltage((frontLeftOptimized.speed/maxModSpeed)*maxVoltage)
            self.frontRightDrive.setVoltage((frontRightOptimized.speed/maxModSpeed)*maxVoltage)
        except Exception as e:
            print(f"Error in manualDriveFromChassisSpeeds: {e}")
            self.stopMotors()

    def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        """Drive the robot with field-relative chassis speeds."""
        try:
            if not self.checkSensors():
                print("WARNING: Sensor issues detected, limiting drive capabilities")
                # For safety, you could reduce max speeds here or disable rotation
                # For example:
                speeds = ChassisSpeeds(speeds.vx * 0.5, speeds.vy * 0.5, 0.0)  # Half speed, no rotation

            # Get the last commanded speeds
            last_speeds = self.lastChassisSpeed
            
            # Apply acceleration limiting
            speeds = self.limitAcceleration(last_speeds, speeds)
            
            # Store the new commanded speed
            self.lastChassisSpeed = speeds

            # Counter-intuitive conversion (but don't change)
            Vx = speeds.vy
            Vy = speeds.vx

            # Convert to module states
            speeds = ChassisSpeeds(-Vx, -Vy, -speeds.omega)
            moduleStates = self.kinematics.toSwerveModuleStates(speeds)

            # Desaturate wheel speeds (limit to max speed)
            maxModSpeed = 4.1
            frontLeft, frontRight, backLeft, backRight = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                moduleStates, 
                maxModSpeed
            )
            
            # Optimize module states
            frontLeft.optimize(Rotation2d(ticks2rad(self.cache.frontLeftAbsPos)))
            frontRight.optimize(Rotation2d(ticks2rad(self.cache.frontRightAbsPos)))
            backLeft.optimize(Rotation2d(ticks2rad(self.cache.backLeftAbsPos)))
            backRight.optimize(Rotation2d(ticks2rad(self.cache.backRightAbsPos)))

            # Calculate PID outputs using cached absolute positions
            blPidOutput = -self.BleftPID.calculate(self.cache.backLeftAbsPos, lratio(backLeft.angle.radians()))
            flPidOutput = -self.FleftPID.calculate(self.cache.frontLeftAbsPos, lratio(frontLeft.angle.radians()))
            brPidOutput = -self.BrightPID.calculate(self.cache.backRightAbsPos, lratio(backRight.angle.radians()))
            frPidOutput = -self.FrightPID.calculate(self.cache.frontRightAbsPos, lratio(frontRight.angle.radians()))

            # Set rotation motors with limited outputs
            self.backLeftRotation.set(max(min(blPidOutput, 1.0), -1.0))
            self.frontLeftRotation.set(max(min(flPidOutput, 1.0), -1.0))
            self.backRightRotation.set(max(min(brPidOutput, 1.0), -1.0))
            self.frontRightRotation.set(max(min(frPidOutput, 1.0), -1.0))

            # Apply voltage to drive motors proportional to desired speed
            maxVoltage = 13
            self.backLeftDrive.setVoltage(-(backLeft.speed/maxModSpeed)*maxVoltage)
            self.backRightDrive.setVoltage((backRight.speed/maxModSpeed)*maxVoltage)
            self.frontLeftDrive.setVoltage((frontLeft.speed/maxModSpeed)*maxVoltage)
            self.frontRightDrive.setVoltage((frontRight.speed/maxModSpeed)*maxVoltage)
        except Exception as e:
            print(f"Error in driveFromChassisSpeeds: {e}")
            self.stopMotors()

    
    def limitAcceleration(self, current_speeds: ChassisSpeeds, target_speeds: ChassisSpeeds, dt: float = 0.02) -> ChassisSpeeds:
        """Limit acceleration rates to prevent jerky movements.
        
        Args:
            current_speeds: Current chassis speeds
            target_speeds: Target chassis speeds
            dt: Time difference since last update (default: 20ms)
            
        Returns:
            ChassisSpeeds with limited acceleration
        """
        # Maximum acceleration rates (adjust these values based on your robot's characteristics)
        max_linear_accel = 4.0  # m/s²
        max_angular_accel = 8.0  # rad/s²
        
        # Calculate maximum speed changes for this time step
        max_vx_change = max_linear_accel * dt
        max_vy_change = max_linear_accel * dt
        max_omega_change = max_angular_accel * dt
        
        # Limit vx acceleration
        vx_error = target_speeds.vx - current_speeds.vx
        vx_change = max(-max_vx_change, min(vx_error, max_vx_change))
        
        # Limit vy acceleration
        vy_error = target_speeds.vy - current_speeds.vy
        vy_change = max(-max_vy_change, min(vy_error, max_vy_change))
        
        # Limit omega acceleration
        omega_error = target_speeds.omega - current_speeds.omega
        omega_change = max(-max_omega_change, min(omega_error, max_omega_change))
    
        # Create new chassis speeds with limited acceleration
        return ChassisSpeeds(
            current_speeds.vx + vx_change,
            current_speeds.vy + vy_change,
            current_speeds.omega + omega_change
        )

    def checkSensors(self) -> bool:
        """Check sensors for valid readings.
        
        Returns:
            bool: True if sensors are OK, False if there are issues
        """
        try:
            # Check for invalid encoder readings
            encoder_issues = False
            
            # Check CANcoders for reasonable values (should be between 0-1 for absolute position)
            if not (0 <= self.cache.frontLeftAbsPos <= 1) and not (-1 <= self.cache.frontLeftAbsPos <= 0):
                print(f"WARNING: Front left encoder reading out of range: {self.cache.frontLeftAbsPos}")
                encoder_issues = True
                
            if not (0 <= self.cache.frontRightAbsPos <= 1) and not (-1 <= self.cache.frontRightAbsPos <= 0):
                print(f"WARNING: Front right encoder reading out of range: {self.cache.frontRightAbsPos}")
                encoder_issues = True
                
            if not (0 <= self.cache.backLeftAbsPos <= 1) and not (-1 <= self.cache.backLeftAbsPos <= 0):
                print(f"WARNING: Back left encoder reading out of range: {self.cache.backLeftAbsPos}")
                encoder_issues = True
                
            if not (0 <= self.cache.backRightAbsPos <= 1) and not (-1 <= self.cache.backRightAbsPos <= 0):
                print(f"WARNING: Back right encoder reading out of range: {self.cache.backRightAbsPos}")
                encoder_issues = True
            
            # Check for gyro issues - lookout for NaN or extremely large values
            if math.isnan(self.cache.yaw) or abs(self.cache.yaw) > 1000:
                print(f"WARNING: Gyro reading invalid: {self.cache.yaw}")
                return False
                
            return not encoder_issues
        except Exception as e:
            print(f"Error checking sensors: {e}")
            return False

    def checkMotorTemperatures(self) -> bool:
        """Check motor temperatures and return True if any are too hot.
        
        Returns:
            bool: True if any motor is overheating
        """
        try:
            temp_threshold = 80.0  # Celsius - adjust based on NEO specifications
            high_temp_threshold = 90.0  # Critical temperature
            
            # Get temperatures (use cached values for better performance)
            temps = []
            if hasattr(self.frontLeftDrive, 'getMotorTemperature'):
                temps.append(self.frontLeftDrive.getMotorTemperature())
            if hasattr(self.frontRightDrive, 'getMotorTemperature'):
                temps.append(self.frontRightDrive.getMotorTemperature())
            if hasattr(self.backLeftDrive, 'getMotorTemperature'):
                temps.append(self.backLeftDrive.getMotorTemperature())
            if hasattr(self.backRightDrive, 'getMotorTemperature'):
                temps.append(self.backRightDrive.getMotorTemperature())
            if hasattr(self.frontLeftRotation, 'getMotorTemperature'):
                temps.append(self.frontLeftRotation.getMotorTemperature())
            if hasattr(self.frontRightRotation, 'getMotorTemperature'):
                temps.append(self.frontRightRotation.getMotorTemperature())
            if hasattr(self.backLeftRotation, 'getMotorTemperature'):
                temps.append(self.backLeftRotation.getMotorTemperature())
            if hasattr(self.backRightRotation, 'getMotorTemperature'):
                temps.append(self.backRightRotation.getMotorTemperature())
            
            # Check for hot motors
            for temp in temps:
                if temp > high_temp_threshold:
                    print(f"CRITICAL: Motor temperature {temp}°C exceeds {high_temp_threshold}°C!")
                    return True
                elif temp > temp_threshold:
                    print(f"WARNING: Motor temperature {temp}°C exceeds {temp_threshold}°C!")
            
            return False
        except Exception as e:
            print(f"Error checking motor temperatures: {e}")
            return False  # Default to not reporting overheating on error

    def stopMotors(self):
        """Stop all motors."""
        try:
            # Stop drive motors
            self.frontLeftDrive.set(0)
            self.frontRightDrive.set(0)
            self.backLeftDrive.set(0)
            self.backRightDrive.set(0)

            # Stop rotation motors
            self.frontLeftRotation.set(0)
            self.frontRightRotation.set(0)
            self.backLeftRotation.set(0)
            self.backRightRotation.set(0)
        except Exception as e:
            print(f"Error stopping motors: {e}")