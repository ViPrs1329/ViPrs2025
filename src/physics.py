# src/physics.py
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# motors and sensors, and then you run your robot code as normal. This file is
# intended to be modified by you to accurately simulate your robot.

import hal.simulation
import wpilib.simulation
import wpimath.geometry
from wpimath.system.plant import DCMotor
import wpimath.units as units
from pyfrc.physics.core import PhysicsInterface
import typing
import ntcore

if typing.TYPE_CHECKING:
    from robot import MyRobot

# Simulation constants
kMotorPort = 10  # Motor PWM port(s)
kEncoderChannelA = 0  # Encoder DIO channel A
kEncoderChannelB = 1  # Encoder DIO channel B
kGearRatio = 10.71  # Gearbox reduction
kWheelRadius = 0.0508  # Meters - radius of the wheels

# This tracks the physics simulation
class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
        :param robot: your robot object
        """
        self.physics_controller = physics_controller
        self.robot = robot

        # Create simulation for swerve drive
        # We're using a simpler approach here, not the complete DifferentialDriveSim
        # since the arguments to createKitbotSim don't match our needs

        # Create simulation pigeon object
        try:
            self.pigeon = hal.simulation.SimDeviceSimAccessors.SimDouble(
                "Pigeon2[21]", "Yaw"
            )
        except Exception as e:
            print(f"Warning: Could not create Pigeon simulator: {e}")
            self.pigeon = None

        # Create simulated encoders for the swerve modules
        try:
            self.cancoders = {}
            for name, id in [
                ("FLEncoder", 19),
                ("FREncoder", 18), 
                ("BLEncoder", 20),
                ("BREncoder", 17)
            ]:
                self.cancoders[name] = hal.simulation.SimDeviceSimAccessors.SimDouble(
                    f"CANcoder[{id}]", "position"
                )
        except Exception as e:
            print(f"Warning: Could not create CANcoder simulator: {e}")
            self.cancoders = {}

        # Set initial robot position
        try:
            self.physics_controller.field.setRobotPose(
                wpimath.geometry.Pose2d(2, 2, wpimath.geometry.Rotation2d(0))
            )
        except Exception as e:
            print(f"Warning: Could not set robot pose: {e}")

        # Initialize robot position and heading
        self.position = wpimath.geometry.Pose2d(2, 2, wpimath.geometry.Rotation2d(0))
        self.gyro_angle = 0.0

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # Simulate the drivetrain
        try:
            # Check if we have a valid drivetrain object
            if hasattr(self.robot, "container") and hasattr(self.robot.container, "drivetrain"):
                dt = self.robot.container.drivetrain

                # Get the chassis speeds from the drivetrain
                speeds = dt.getChassisSpeed()
                
                # Very simple simulation model
                # In a real simulation, you'd use proper kinematics and dynamics
                vx = speeds.vx
                vy = speeds.vy
                omega = speeds.omega
                
                # Update the robot's position and orientation
                # This is a simplified model - a real swerve model would be more complex
                dx = vx * tm_diff
                dy = vy * tm_diff
                dtheta = omega * tm_diff
                
                # Update robot position
                cos_angle = self.position.rotation().cos()
                sin_angle = self.position.rotation().sin()
                
                # Apply field-oriented transformation
                x = self.position.x + (dx * cos_angle - dy * sin_angle)
                y = self.position.y + (dx * sin_angle + dy * cos_angle)
                theta = self.position.rotation().radians() + dtheta
                
                # Create new pose
                self.position = wpimath.geometry.Pose2d(
                    x, y, wpimath.geometry.Rotation2d(theta)
                )
                
                # Update gyro
                self.gyro_angle += dtheta * 180 / 3.14159
                if self.pigeon:
                    self.pigeon.set(self.gyro_angle)
                
                # Update the robot's position on the field
                self.physics_controller.field.setRobotPose(self.position)
                
        except Exception as e:
            # Just print the exception and continue - don't want to crash simulation
            print(f"Physics simulation error: {e}")

        # Publish robot pose to NetworkTables for dashboard
        try:
            field_table = ntcore.NetworkTableInstance.getDefault().getTable("field")
            pose_pub = field_table.getStructTopic("robot_pose", wpimath.geometry.Pose2d).publish()
            pose_pub.set(self.position)
        except Exception as e:
            print(f"Error publishing robot pose: {e}")