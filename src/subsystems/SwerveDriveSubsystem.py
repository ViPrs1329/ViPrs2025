# src/subsystems/SwerveDriveSubsystem.py
import rev
import math
import commands2
import wpilib
import ntcore

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath import controller
from wpilib import DriverStation, RobotBase

from constants import CANIDs, driveConsts
from team254.LazySparkMax import LazySparkMax
from team254.SparkMaxFactory import SparkMaxFactory
from subsystems.BaseSubsystem import BaseSubsystem

# Import phoenix hardware conditionally to prevent import errors
try:
    from phoenix6.hardware import CANcoder, Pigeon2
    phoenix_imported = True
except ImportError:
    phoenix_imported = False
    print("Warning: Could not import phoenix6 hardware, using simulation")

# Import simulation module conditionally
try:
    from sim.swerve_drive_sim import SwerveDriveSim
    swerve_sim_imported = True
except ImportError:
    swerve_sim_imported = False
    print("Warning: Could not import SwerveDriveSim, using fallback")


def lratio(angle):
    """Converts -pi, pi to -.5, .5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    """Convert CANcoder ticks to radians"""
    return (something/.5)*-math.pi

def deg2Rot2d(deg) -> Rotation2d:
    """Convert degrees to Rotation2d"""
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)

def ticks2radODOMETRY(something):
    """Convert CANcoder ticks to radians for odometry"""
    # units are in rotations
    return something * 2 * math.pi

def getSwerveModPos(rotEnc, driveEnc) -> SwerveModulePosition:
    """Get the position of a swerve module for odometry"""
    try:
        # Ensure we have valid encoders
        if rotEnc is None or driveEnc is None:
            return SwerveModulePosition(0, Rotation2d())
            
        # Get rotation position safely
        try:
            rot_pos = rotEnc.get_position().value_as_double
        except (AttributeError, TypeError):
            # Fallback if get_position() doesn't work as expected
            try:
                rot_pos = rotEnc.getPosition()
            except:
                rot_pos = 0
                
        # Get drive position safely
        try:
            drive_pos = (driveEnc.getPosition()/driveConsts.DRIVE_GEAR_RATIO)*driveConsts.WHEEL_CIRCUMFERENCE
        except:
            drive_pos = 0
            
        return SwerveModulePosition(
            drive_pos,
            Rotation2d(ticks2radODOMETRY(rot_pos))
        )
    except Exception as e:
        print(f"Error in getSwerveModPos: {e}")
        return SwerveModulePosition(0, Rotation2d())


class SwerveModule:
    """
    Class representing a single swerve module with drive and rotation motors.
    """
    def __init__(self, name: str, drive_id: int, rotation_id: int, encoder_id: int, 
                 location: Translation2d, drive_inverted: bool = False, rot_inverted: bool = False):
        """
        Initialize a swerve module.
        
        Args:
            name (str): The name of this module (e.g., "FrontLeft")
            drive_id (int): CAN ID of the drive motor
            rotation_id (int): CAN ID of the rotation motor
            encoder_id (int): CAN ID of the CANcoder
            location (Translation2d): Position of this module relative to robot center
            drive_inverted (bool): Whether to invert the drive motor
            rot_inverted (bool): Whether to invert the rotation motor
        """
        self.name = name
        self.location = location
        self.is_simulation = RobotBase.isSimulation()
        
        # Initialize default values
        self.drive_motor = None
        self.rotation_motor = None
        self.rotation_encoder = None
        self.drive_encoder = None
        self.rotation_pid = None
        
        # Cache for sensor values
        self.drive_position = 0.0
        self.rotation_position = 0.0
        self.drive_current = 0.0
        self.rotation_current = 0.0
        
        try:
            # Create motor configurations
            drive_config = SparkMaxFactory.Configuration()
            if self.is_simulation:
                drive_config.idle_mode = rev.SparkMax.IdleMode.kBrake
            else:
                drive_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            drive_config.current_limit = driveConsts.currentLimit
            drive_config.voltage_comp_enabled = True
            drive_config.voltage_comp_saturation = 12.0
            drive_config.inverted = drive_inverted
            
            rotation_config = SparkMaxFactory.Configuration()
            if self.is_simulation:
                rotation_config.idle_mode = rev.SparkMax.IdleMode.kBrake
            else:
                rotation_config.idle_mode = rev.CANSparkMax.IdleMode.kBrake
            rotation_config.current_limit = driveConsts.currentLimit
            rotation_config.voltage_comp_enabled = True
            rotation_config.inverted = rot_inverted
            
            # Create motors
            self.drive_motor = SparkMaxFactory.createSparkMax(drive_id, drive_config)
            self.rotation_motor = SparkMaxFactory.createSparkMax(rotation_id, rotation_config)
            
            # Get drive encoder
            if self.drive_motor is not None:
                self.drive_encoder = self.drive_motor.getEncoder()
            
            # Create CANcoder
            if phoenix_imported and not self.is_simulation:
                self.rotation_encoder = CANcoder(encoder_id)
            else:
                # Create a dummy encoder for simulation
                class DummyEncoder:
                    def __init__(self):
                        self._value = 0.0
                    
                    def get_position(self):
                        class DummyValue:
                            def __init__(self, val):
                                self.value_as_double = val
                        return DummyValue(self._value)
                        
                    def get_absolute_position(self):
                        return self.get_position()
                
                self.rotation_encoder = DummyEncoder()
                print(f"Created dummy encoder for {name} module")
            
            # Create PID controller for rotation
            Kp = 4.0  # Proportional gain for rotation control
            self.rotation_pid = controller.PIDController(Kp, 0, 0)
            self.rotation_pid.enableContinuousInput(-.5, .5)
            self.rotation_pid.setSetpoint(0.0)
            
        except Exception as e:
            print(f"Error initializing {name} swerve module: {e}")
            
    def cacheSensors(self):
        """Cache sensor values to reduce CAN traffic"""
        try:
            if self.drive_encoder is not None:
                self.drive_position = self.drive_encoder.getPosition()
                
            if self.rotation_encoder is not None:
                try:
                    self.rotation_position = self.rotation_encoder.get_absolute_position()._value
                except AttributeError:
                    try:
                        # Try alternate API
                        self.rotation_position = self.rotation_encoder.get_position().value_as_double
                    except Exception as e:
                        print(f"Error getting rotation position for {self.name}: {e}")
        except Exception as e:
            print(f"Error caching {self.name} module sensors: {e}")
    
    def cacheCurrents(self):
        """Cache current values (called less frequently)"""
        try:
            if self.drive_motor is not None:
                self.drive_current = self.drive_motor.getOutputCurrent()
                
            if self.rotation_motor is not None:
                self.rotation_current = self.rotation_motor.getOutputCurrent()
        except Exception as e:
            print(f"Error caching {self.name} module currents: {e}")
    
    def getPosition(self) -> SwerveModulePosition:
        """Get the position of this module for odometry"""
        return getSwerveModPos(self.rotation_encoder, self.drive_encoder)
    
    def getState(self) -> SwerveModuleState:
        """Get the current state of this module"""
        try:
            # Get drive velocity
            drive_velocity = 0.0
            if self.drive_encoder is not None:
                drive_velocity = self.drive_encoder.getVelocity()
                
            # Get rotation angle
            rotation_angle = Rotation2d()
            if self.rotation_encoder is not None:
                try:
                    # Try to get rotation position from encoder
                    rot_pos = self.rotation_encoder.get_position().value_as_double
                    rotation_angle = Rotation2d(ticks2rad(rot_pos))
                except (AttributeError, TypeError):
                    # Fallback to cached rotation position
                    rotation_angle = Rotation2d(ticks2rad(self.rotation_position))
                
            return SwerveModuleState(drive_velocity, rotation_angle)
        except Exception as e:
            print(f"Error getting {self.name} module state: {e}")
            return SwerveModuleState(0, Rotation2d())
    
    def setDesiredState(self, desired_state: SwerveModuleState):
        """Set the desired state of this module"""
        try:
            if desired_state is None:
                print(f"Warning: Received None state for {self.name}, ignoring")
                return
                
            if not isinstance(desired_state, SwerveModuleState):
                print(f"Warning: Invalid state type for {self.name}, expected SwerveModuleState")
                return
                
            # Optimize the module state to avoid spinning more than 90 degrees
            if self.rotation_encoder is not None:
                current_rotation = Rotation2d(ticks2rad(self.rotation_position))
                optimized_state = SwerveModuleState.optimize(
                    desired_state,
                    current_rotation
                )
            else:
                optimized_state = desired_state
                
            # Set drive motor speed
            if self.drive_motor is not None:
                self.drive_motor.set(optimized_state.speed / driveConsts.MAX_SPEED)
                
            # Set rotation motor position
            if self.rotation_motor is not None and self.rotation_pid is not None:
                target_angle = optimized_state.angle.radians()
                current_angle = ticks2rad(self.rotation_position)
                self.rotation_pid.setSetpoint(target_angle)
                rotation_output = self.rotation_pid.calculate(current_angle)
                self.rotation_motor.set(rotation_output)
                
        except Exception as e:
            print(f"Error setting {self.name} module state: {e}")
    
    def stop(self):
        """Stop this module's motors"""
        try:
            if self.drive_motor is not None:
                self.drive_motor.set(0)
                
            if self.rotation_motor is not None:
                self.rotation_motor.set(0)
        except Exception as e:
            print(f"Error stopping {self.name} module: {e}")
            
    def resetEncoders(self):
        """Reset all encoders to zero"""
        try:
            if self.drive_encoder is not None:
                self.drive_encoder.setPosition(0)
                
            if self.rotation_encoder is not None:
                self.rotation_encoder.setPosition(0)
                
            # Reset cached values
            self.drive_position = 0.0
            self.rotation_position = 0.0
        except Exception as e:
            print(f"Error resetting {self.name} module encoders: {e}")


class SwerveDrive(commands2.Subsystem):
    """
    Subsystem for controlling the swerve drive.
    """
    def __init__(self):
        super().__init__()
        
        # Initialize simulation flag
        self.is_simulation = RobotBase.isSimulation()
        
        # Calculate module positions
        half_width = driveConsts.TRACKWIDTH / 2.0
        half_length = driveConsts.WHEELBASE / 2.0
        
        # Create swerve modules
        self.front_left = SwerveModule(
            "Front Left",
            CANIDs.SwerveModuleDrive1,
            CANIDs.SwerveModuleRotation1,
            CANIDs.EncoderModuleRotation1,
            Translation2d(half_length, half_width),
            drive_inverted=True,
            rot_inverted=True
        )
        
        self.front_right = SwerveModule(
            "Front Right",
            CANIDs.SwerveModuleDrive2,
            CANIDs.SwerveModuleRotation2,
            CANIDs.EncoderModuleRotation2,
            Translation2d(half_length, -half_width),
            drive_inverted=True,
            rot_inverted=True
        )
        
        self.back_left = SwerveModule(
            "Back Left",
            CANIDs.SwerveModuleDrive3,
            CANIDs.SwerveModuleRotation3,
            CANIDs.EncoderModuleRotation3,
            Translation2d(-half_length, half_width),
            drive_inverted=True,
            rot_inverted=True
        )
        
        self.back_right = SwerveModule(
            "Back Right",
            CANIDs.SwerveModuleDrive4,
            CANIDs.SwerveModuleRotation4,
            CANIDs.EncoderModuleRotation4,
            Translation2d(-half_length, -half_width),
            drive_inverted=True,
            rot_inverted=True
        )
        
        # Store modules in a list for easy iteration
        self.modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right
        ]
        
        # Create kinematics object
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left.location,
            self.front_right.location,
            self.back_left.location,
            self.back_right.location
        )
        
        # Initialize gyro
        if phoenix_imported and not self.is_simulation:
            self.gyro = Pigeon2(CANIDs.PIGEON)
        else:
            # Create a dummy gyro for simulation
            class DummyEncoder:
                def __init__(self):
                    self._value = 0.0
                    self._velocity = 0.0
                
                def get_position(self):
                    class DummyValue:
                        def __init__(self, val):
                            self.value_as_double = val
                    return DummyValue(self._value)
                    
                def get_absolute_position(self):
                    return self.get_position()
                    
                def getPosition(self):
                    """Get the position of the encoder."""
                    return self._value
                    
                def getVelocity(self):
                    """Get the velocity of the encoder."""
                    return self._velocity
                    
                def setPosition(self, position):
                    """Set the position of the encoder."""
                    self._value = position
                    
                def setVelocity(self, velocity):
                    """Set the velocity of the encoder."""
                    self._velocity = velocity
            
            self.gyro = DummyGyro()
            print("Created dummy gyro for simulation")
            
        # Initialize simulation components if needed
        if self.is_simulation and swerve_sim_imported:
            try:
                self.sim = SwerveDriveSim(self)
                print("Successfully initialized SwerveDriveSim")
            except Exception as e:
                print(f"Warning: Failed to initialize SwerveDriveSim: {e}")
                self.sim = None
        else:
            self.sim = None
            print("Running without simulation")
        
        # Initialize odometry with safe default values
        try:
            # Get initial module positions
            module_positions = [
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition()
            ]
            
            # Create odometry object
            self.odometry = SwerveDrive4Odometry(
                self.kinematics,
                self.getGyroYaw(),
                module_positions,
                Pose2d()
            )
            print("Successfully initialized odometry")
        except Exception as e:
            print(f"Warning: Failed to initialize odometry: {e}")
            # Create a dummy odometry object
            class DummyOdometry:
                def __init__(self):
                    self._pose = Pose2d()
                
                def update(self, *args):
                    return self._pose
                    
                def getPoseMeters(self):
                    return self._pose
                    
                def resetPosition(self, *args):
                    pass
                    
            self.odometry = DummyOdometry()
        
        # Initialize field-oriented control
        self.field_oriented = True
        
        # Initialize robot pose
        self.robot_pose = Pose2d()
        
        # Initialize last chassis speed
        self.last_chassis_speed = ChassisSpeeds()
        
        # Initialize network tables
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.drive_table = self.nt.getTable("SwerveDrive")
        self.speeds_pub = self.drive_table.getDoubleTopic("ChassisSpeedsX").publish()
        
        # Initialize simulation publishers if needed
        if self.is_simulation:
            self.pose_x_pub = self.drive_table.getDoubleTopic("PoseX").publish()
            self.pose_y_pub = self.drive_table.getDoubleTopic("PoseY").publish()
            self.pose_rot_pub = self.drive_table.getDoubleTopic("PoseRot").publish()
            self.gyro_yaw_pub = self.drive_table.getDoubleTopic("GyroYaw").publish()
    
    def cacheSensors(self):
        """Cache sensor values to reduce CAN traffic"""
        try:
            # Cache module sensors
            for module in self.modules:
                module.cacheSensors()
                
            # Cache gyro
            if self.gyro is not None:
                self.gyro_yaw = self.gyro.getYaw()
                self.gyro_pitch = self.gyro.getPitch()
                
        except Exception as e:
            print(f"Error caching swerve drive sensors: {e}")
    
    def updateOdometry(self):
        """Update the robot's odometry"""
        try:
            # Update odometry
            self.robot_pose = self.odometry.update(
                self.getGyroYaw(),
                (
                    self.front_left.getPosition(),
                    self.front_right.getPosition(),
                    self.back_left.getPosition(),
                    self.back_right.getPosition()
                )
            )
            
            # Update simulation if needed
            if self.is_simulation and self.sim is not None:
                self.sim.update(0.02)  # Use fixed timestep for simulation
                
        except Exception as e:
            print(f"Error updating swerve drive odometry: {e}")
    
    def updateNetworkTables(self):
        """Update network tables with current state"""
        try:
            # Update pose
            self.drive_table.putNumber("PoseX", self.robot_pose.X())
            self.drive_table.putNumber("PoseY", self.robot_pose.Y())
            self.drive_table.putNumber("PoseRot", self.robot_pose.rotation().degrees())
            
            # Update gyro
            self.drive_table.putNumber("GyroYaw", self.getGyroYaw().degrees())
            self.drive_table.putNumber("GyroPitch", self.getGyroPitch())
            
            # Update simulation if needed
            if self.is_simulation:
                self.pose_x_pub.set(self.robot_pose.X())
                self.pose_y_pub.set(self.robot_pose.Y())
                self.pose_rot_pub.set(self.robot_pose.rotation().degrees())
                self.gyro_yaw_pub.set(self.getGyroYaw().degrees())
                
        except Exception as e:
            print(f"Error updating swerve drive network tables: {e}")
    
    def updateDashboard(self):
        """Update the dashboard with current state"""
        try:
            # Update module states
            for module in self.modules:
                state = module.getState()
                self.drive_table.putNumber(f"{module.name}Speed", state.speed)
                self.drive_table.putNumber(f"{module.name}Angle", state.angle.degrees())
                
            # Update chassis speeds
            chassis_speeds = self.getChassisSpeeds()
            self.drive_table.putNumber("ChassisSpeedX", chassis_speeds.vx)
            self.drive_table.putNumber("ChassisSpeedY", chassis_speeds.vy)
            self.drive_table.putNumber("ChassisSpeedOmega", chassis_speeds.omega)
            
        except Exception as e:
            print(f"Error updating swerve drive dashboard: {e}")
    
    def periodic(self):
        """Update the swerve drive subsystem."""
        # Update odometry
        if self.odometry is not None:
            try:
                # Get module positions safely
                positions = []
                for module in self.modules:
                    try:
                        pos = getSwerveModPos(module.rotation_encoder, module.drive_motor)
                        positions.append(pos)
                    except Exception as e:
                        print(f"Error getting module position: {e}")
                        positions.append(SwerveModulePosition(0, Rotation2d()))
                
                # Update odometry with valid positions
                self.odometry.update(
                    self.gyro.getRotation2d(),
                    positions
                )
                
                # Get current pose
                self.pose = self.odometry.getPose()
                
                # Update field
                if self.field is not None:
                    self.field.setRobotPose(self.pose)
                
                # Publish pose to NetworkTables
                if self.pose_pub is not None:
                    self.pose_pub.set([self.pose.X(), self.pose.Y(), self.pose.rotation().degrees()])
            except Exception as e:
                print(f"Error updating odometry: {e}")
        
        # Update module states
        try:
            # Get current chassis speeds
            speeds = self.kinematics.toChassisSpeeds(self.getModuleStates())
            
            # Publish speeds to NetworkTables
            if self.speeds_pub is not None:
                self.speeds_pub.set([speeds.vx, speeds.vy, speeds.omega])
                
            # Update simulation if available
            if self.sim is not None:
                self.sim.update()
                
            # Update module states
            if self.kinematics is not None:
                module_states = self.kinematics.toSwerveModuleStates(ChassisSpeeds())
                if module_states is not None:
                    self.setModuleStates(module_states)
        except Exception as e:
            print(f"Error updating module states: {e}")
    
    def resetGyro(self):
        """Reset the gyro to zero"""
        try:
            if self.gyro is not None:
                self.gyro.reset()
        except Exception as e:
            print(f"Error resetting gyro: {e}")
    
    def getGyroYaw(self) -> Rotation2d:
        """Get the current yaw angle from the gyro"""
        try:
            if self.gyro is not None:
                return Rotation2d(math.radians(self.gyro.getYaw()))
            return Rotation2d()
        except Exception as e:
            print(f"Error getting gyro yaw: {e}")
            return Rotation2d()
    
    def getGyroPitch(self) -> float:
        """Get the current pitch angle from the gyro"""
        try:
            if self.gyro is not None:
                return self.gyro.getPitch()
            return 0.0
        except Exception as e:
            print(f"Error getting gyro pitch: {e}")
            return 0.0
    
    def getChassisSpeeds(self) -> ChassisSpeeds:
        """Get the current chassis speeds"""
        try:
            if self.is_simulation and self.sim is not None:
                return self.sim.getChassisSpeeds()
            # Get module states as a tuple
            module_states = (
                self.front_left.getState(),
                self.front_right.getState(),
                self.back_left.getState(),
                self.back_right.getState()
            )
            return self.kinematics.toChassisSpeeds(module_states)
        except Exception as e:
            print(f"Error getting chassis speeds: {e}")
            return ChassisSpeeds()
    
    def drive(self, x_speed: float, y_speed: float, rot_speed: float, field_oriented: bool = True):
        """
        Drive the robot with the given speeds.
        
        Args:
            x_speed: Forward speed in m/s
            y_speed: Leftward speed in m/s
            rot_speed: Counterclockwise rotation speed in rad/s
            field_oriented: Whether to use field-oriented control
        """
        try:
            # Create chassis speeds
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot_speed)
            
            # Convert to field-oriented speeds if needed
            if field_oriented:
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed, y_speed, rot_speed, self.getGyroYaw()
                )
            
            # Store for next get_chassis_speeds call
            self.last_chassis_speed = chassis_speeds
            
            # Convert to module states
            module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            
            # Set module states
            if self.is_simulation and self.sim is not None:
                self.sim.setModuleStates(module_states)
            else:
                for module, state in zip(self.modules, module_states):
                    if state is not None:
                        module.setDesiredState(state)
                    else:
                        print(f"Warning: Received None state for {module.name}, using default state")
                        # Use a default state with zero speed and current angle
                        current_state = module.getState()
                        if current_state is not None:
                            module.setDesiredState(SwerveModuleState(0, current_state.angle))
                        else:
                            module.setDesiredState(SwerveModuleState(0, Rotation2d()))
                    
        except Exception as e:
            print(f"Error driving swerve drive: {e}")
    
    def stop(self):
        """Stop all motors"""
        try:
            if self.is_simulation and self.sim is not None:
                self.sim.stop()
            else:
                for module in self.modules:
                    current_state = module.getState()
                    if current_state is not None:
                        module.setDesiredState(SwerveModuleState(0, current_state.angle))
                    else:
                        print(f"Warning: Received None state for {module.name}, using default stop state")
                        module.setDesiredState(SwerveModuleState(0, Rotation2d()))
        except Exception as e:
            print(f"Error stopping swerve drive: {e}")
    
    def resetPose(self, pose: Pose2d):
        """Reset the robot's pose"""
        try:
            self.robot_pose = pose
            self.odometry.resetPosition(
                self.getGyroYaw(),
                (
                    self.front_left.getPosition(),
                    self.front_right.getPosition(),
                    self.back_left.getPosition(),
                    self.back_right.getPosition()
                ),
                pose
            )
        except Exception as e:
            print(f"Error resetting swerve drive pose: {e}")
    
    def resetEncoders(self):
        """Reset all module encoders"""
        try:
            if self.is_simulation and self.sim is not None:
                self.sim.reset_encoders()
            else:
                for module in self.modules:
                    module.resetEncoders()
        except Exception as e:
            print(f"Error resetting swerve drive encoders: {e}")
    
    def getPose(self) -> Pose2d:
        """Get the current robot pose"""
        return self.robot_pose
    
    def getField(self) -> None:
        """Get the field object for visualization."""
        # In newer WPILib versions, field visualization is handled through NetworkTables
        # The robot's pose is already being published in updateNetworkTables()
        return None

    
    def setModuleStates(self, desired_states):
        """Set the desired states for all swerve modules."""
        try:
            # Handle None states
            if desired_states is None:
                print("Warning: Received None states, using default states")
                desired_states = [
                    wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d())
                ] * 4
            
            # Ensure we have enough states
            if len(desired_states) < 4:
                print(f"Warning: Not enough states ({len(desired_states)}), padding with defaults")
                default_state = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d())
                desired_states.extend([default_state] * (4 - len(desired_states)))
            
            # Validate and optimize each module's state
            for i, (module, state) in enumerate(zip(self.modules, desired_states)):
                try:
                    # Validate the state
                    if state is None:
                        print(f"Warning: Module {i} state is None, using default")
                        state = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d())
                    
                    # Validate the state's attributes
                    if not hasattr(state, 'speed') or state.speed is None:
                        print(f"Warning: Module {i} has invalid speed, using 0")
                        speed = 0
                    else:
                        speed = state.speed
                        
                    if not hasattr(state, 'angle') or state.angle is None:
                        print(f"Warning: Module {i} has invalid angle, using current")
                        # Try to get current angle, default to 0 if that fails
                        try:
                            angle = module.getState().angle
                        except:
                            angle = wpimath.geometry.Rotation2d()
                    else:
                        angle = state.angle
                    
                    # Recreate the state with validated components
                    validated_state = wpimath.kinematics.SwerveModuleState(speed, angle)
                    
                    # Set the module state
                    module.setDesiredState(validated_state)
                    
                except Exception as e:
                    print(f"Error setting module {i} state: {e}")
                    # Set a safe default state
                    try:
                        module.setDesiredState(
                            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d())
                        )
                    except Exception as inner_e:
                        print(f"Critical error setting default state for module {i}: {inner_e}")
                        
        except Exception as e:
            print(f"Error in setModuleStates: {e}")
            # Attempt to stop all modules in case of severe error
            for module in self.modules:
                try:
                    module.stop()
                except:
                    pass

    def getModuleState(self, index):
        """Get the current state of a swerve module."""
        try:
            if index < 0 or index >= len(self.modules):
                print(f"Warning: Invalid module index {index}")
                return SwerveModuleState(0, Rotation2d())
            
            module = self.modules[index]
            if module is None:
                print(f"Warning: Module {index} is None")
                return SwerveModuleState(0, Rotation2d())
            
            # Get drive velocity
            try:
                drive_velocity = module.drive_motor.getVelocity()
            except Exception as e:
                print(f"Error getting drive velocity for module {index}: {e}")
                drive_velocity = 0
            
            # Get rotation position
            try:
                rotation_position = module.rotation_encoder.getPosition()
            except Exception as e:
                print(f"Error getting rotation position for module {index}: {e}")
                rotation_position = 0
            
            # Create and return the module state
            return SwerveModuleState(
                drive_velocity,
                Rotation2d(ticks2rad(rotation_position))
            )
        except Exception as e:
            print(f"Error in getModuleState: {e}")
            return SwerveModuleState(0, Rotation2d())

    def getModuleStates(self):
        """Get the current states of all swerve modules."""
        try:
            states = []
            for i in range(len(self.modules)):
                state = self.getModuleState(i)
                if state is None:
                    print(f"Warning: Module {i} state is None, using default")
                    state = SwerveModuleState(0, Rotation2d())
                states.append(state)
            return states
        except Exception as e:
            print(f"Error in getModuleStates: {e}")
            return [SwerveModuleState(0, Rotation2d())] * 4