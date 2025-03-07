"""
Drive subsystem for swerve drive control.
"""
import math
import wpilib
import commands2
import rev
import phoenix6
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.units import inchesToMeters

from constants import *

class DriveSubsystem(commands2.Subsystem):
    """
    This subsystem controls the robot's swerve drive.
    """

    def __init__(self) -> None:
        super().__init__()

        # Initialize the gyro
        self.gyro = phoenix6.hardware.Pigeon2(PIGEON_ID)
        self.gyro.reset()  # Reset to 0 degrees

        # Create the kinematics object
        self.kinematics = SwerveDrive4Kinematics(
            # Front left
            Translation2d(
                inchesToMeters(ROBOT_LENGTH_INCHES / 2),
                inchesToMeters(ROBOT_WIDTH_INCHES / 2)
            ),
            # Front right
            Translation2d(
                inchesToMeters(ROBOT_LENGTH_INCHES / 2),
                inchesToMeters(-ROBOT_WIDTH_INCHES / 2)
            ),
            # Back left
            Translation2d(
                inchesToMeters(-ROBOT_LENGTH_INCHES / 2),
                inchesToMeters(ROBOT_WIDTH_INCHES / 2)
            ),
            # Back right
            Translation2d(
                inchesToMeters(-ROBOT_LENGTH_INCHES / 2),
                inchesToMeters(-ROBOT_WIDTH_INCHES / 2)
            )
        )

        # Create the swerve modules
        self.front_left = self._create_swerve_module(
            "Front Left",
            FL_DRIVE_MOTOR_ID,
            FL_TURN_MOTOR_ID,
            FL_CANCODER_ID
        )
        self.front_right = self._create_swerve_module(
            "Front Right",
            FR_DRIVE_MOTOR_ID,
            FR_TURN_MOTOR_ID,
            FR_CANCODER_ID
        )
        self.back_left = self._create_swerve_module(
            "Back Left",
            BL_DRIVE_MOTOR_ID,
            BL_TURN_MOTOR_ID,
            BL_CANCODER_ID
        )
        self.back_right = self._create_swerve_module(
            "Back Right",
            BR_DRIVE_MOTOR_ID,
            BR_TURN_MOTOR_ID,
            BR_CANCODER_ID
        )

        # Initialize field-relative drive
        self.field_relative = False

    def _create_swerve_module(self, name: str, drive_id: int, turn_id: int, encoder_id: int) -> tuple:
        """
        Creates a swerve module with the specified motor and encoder IDs.
        
        Returns a tuple of (drive_motor, turn_motor, turn_encoder, drive_pid, turn_pid)
        """

        # Constants for reset and persist modes
        reset_mode = rev.SparkBase.ResetMode.kNoResetSafeParameters
        persist_mode = rev.SparkBase.PersistMode.kNoPersistParameters

        # Create the drive motor
        drive_motor = rev.SparkMax(drive_id, rev.SparkLowLevel.MotorType.kBrushless)
        # drive_motor.setIdleMode(rev.SparkBase.IdleMode.kBrake)
        # drive_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        drive_conf = rev.SparkBaseConfig()
        drive_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        drive_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)
        
        # Create the turn motor
        turn_motor = rev.SparkMax(turn_id, rev.SparkLowLevel.MotorType.kBrushless)
        # turn_motor.setIdleMode(rev.SparkBase.IdleMode.kBrake)
        # turn_motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT)
        turn_conf = rev.SparkBaseConfig()
        turn_conf.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        turn_conf.smartCurrentLimit(NEO_CURRENT_LIMIT)
        
        # Create the turn encoder
        turn_encoder = phoenix6.hardware.CANcoder(encoder_id)
        
        # Create PID controllers
        '''
        drive_pid = drive_motor.getPIDController()
        drive_pid.setP(SWERVE_DRIVE_P)
        drive_pid.setI(SWERVE_DRIVE_I)
        drive_pid.setD(SWERVE_DRIVE_D)
        
        turn_pid = turn_motor.getPIDController()
        turn_pid.setP(SWERVE_TURN_P)
        turn_pid.setI(SWERVE_TURN_I)
        turn_pid.setD(SWERVE_TURN_D)
        '''

        drive_pid = drive_motor.getClosedLoopController()
        drive_conf.closedLoop.P(SWERVE_DRIVE_P)
        drive_conf.closedLoop.I(SWERVE_DRIVE_I)
        drive_conf.closedLoop.D(SWERVE_DRIVE_D)
        drive_motor.configure(drive_conf, reset_mode, persist_mode)

        turn_pid = turn_motor.getClosedLoopController()
        turn_conf.closedLoop.P(SWERVE_TURN_P)
        turn_conf.closedLoop.I(SWERVE_TURN_I)
        turn_conf.closedLoop.D(SWERVE_TURN_D)
        turn_motor.configure(turn_conf, reset_mode, persist_mode)

        
        return (drive_motor, turn_motor, turn_encoder, drive_pid, turn_pid)

    def getHeading(self) -> float:
        """
        Returns the robot's heading in degrees, from -180 to 180.
        """
        return math.remainder(self.gyro.get_yaw().value, 360)

    def getRotation2d(self) -> Rotation2d:
        """
        Returns the robot's rotation as a Rotation2d object.
        """
        return Rotation2d.fromDegrees(self.getHeading())

    def resetGyro(self) -> None:
        """
        Resets the gyro to 0 degrees.
        """
        self.gyro.reset()

    def drive(self, x_speed: float, y_speed: float, rot: float, rate_limit: bool = True) -> None:
        """
        Drives the robot using the specified speeds.
        
        Parameters
        ----------
        x_speed : float
            Speed of the robot in the x direction (forward) in feet per second
        y_speed : float
            Speed of the robot in the y direction (sideways) in feet per second
        rot : float
            Angular rate of the robot in radians per second
        rate_limit : bool
            Whether to enable rate limiting for smoother control
        """
        if rate_limit:
            # TODO: Implement rate limiting
            pass
            
        # Convert to chassis speeds
        if self.field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, self.getRotation2d()
            )
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)
            
        # Calculate module states
        swerve_module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        
        # Normalize wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, SWERVE_MAX_SPEED_FPS
        )
        
        # Set module states
        self._set_module_state(self.front_left, swerve_module_states[0])
        self._set_module_state(self.front_right, swerve_module_states[1])
        self._set_module_state(self.back_left, swerve_module_states[2])
        self._set_module_state(self.back_right, swerve_module_states[3])

    def _set_module_state(self, module: tuple, state: SwerveModuleState) -> None:
        """
        Sets the state of a swerve module.
        
        Parameters
        ----------
        module : tuple
            Tuple of (drive_motor, turn_motor, turn_encoder, drive_pid, turn_pid)
        state : SwerveModuleState
            Desired state of the module
        """
        drive_motor, turn_motor, turn_encoder, drive_pid, turn_pid = module
        
        # Optimize the state to avoid spinning more than 90 degrees
        current_angle = Rotation2d.fromDegrees(turn_encoder.get_absolute_position().value)
        optimized_state = SwerveModuleState.optimize(state, current_angle)
        
        # Check if optimized_state or its angle is None and handle accordingly
        if optimized_state is None or optimized_state.angle is None:
            # Use the original state if optimization fails
            optimized_state = state
        
        # Set the turn motor position
        turn_pid.setReference(optimized_state.angle.degrees(), rev.SparkMax.ControlType.kPosition)
        
        # Set the drive motor velocity
        drive_pid.setReference(
            optimized_state.speed,
            rev.SparkMax.ControlType.kVelocity
        )

    def toggleFieldRelative(self) -> None:
        """
        Toggles field-relative drive.
        """
        self.field_relative = not self.field_relative

    def periodic(self) -> None:
        """
        Periodic function that runs every 20ms.
        """
        # Update SmartDashboard with gyro data
        wpilib.SmartDashboard.putNumber("Robot Heading", self.getHeading())
        wpilib.SmartDashboard.putBoolean("Field Relative", self.field_relative)
        # TODO: Add any necessary periodic updates
        pass