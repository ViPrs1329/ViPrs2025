# src/physics.py
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# motors and sensors, and then you run your robot code as normal. This file is
# intended to be modified by you to accurately simulate your robot.
import math
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
        """
        # Simulate the drivetrain
        try:
            # Check if we have a valid drivetrain object
            if hasattr(self.robot, "container") and hasattr(self.robot.container, "drivetrain"):
                dt = self.robot.container.drivetrain

                # Get the chassis speeds from the drivetrain
                speeds = dt.getChassisSpeed()
                
                # Print current speeds for debugging
                print(f"Sim speeds - vx: {speeds.vx:.2f}, vy: {speeds.vy:.2f}, omega: {speeds.omega:.2f}")
                
                # Convert from robot-oriented to field-oriented speeds
                # This is a critical step for swerve simulation
                current_angle = self.position.rotation().radians()
                cos_angle = math.cos(current_angle)
                sin_angle = math.sin(current_angle)
                
                # Calculate field-oriented velocity
                vx_field = speeds.vx * cos_angle - speeds.vy * sin_angle
                vy_field = speeds.vx * sin_angle + speeds.vy * cos_angle
                
                # Update robot position based on velocities
                # new_x = self.position.x + vx_field * tm_diff
                # new_y = self.position.y + vy_field * tm_diff
                # new_angle = self.position.rotation().radians() + speeds.omega * tm_diff

                # Direct integration - don't worry about field vs robot orientation for now
                new_x = self.position.x + speeds.vx * tm_diff
                new_y = self.position.y + speeds.vy * tm_diff
                new_angle = self.position.rotation().radians() + speeds.omega * tm_diff
                
                # Create new pose
                self.position = wpimath.geometry.Pose2d(
                    new_x, new_y, wpimath.geometry.Rotation2d(new_angle)
                )
                
                # Update gyro
                self.gyro_angle = math.degrees(new_angle)
                if self.pigeon:
                    self.pigeon.set(self.gyro_angle)
                
                # Update the robot's position on the field
                self.physics_controller.field.setRobotPose(self.position)
                
                # Print current position for debugging
                print(f"Sim position - x: {new_x:.2f}, y: {new_y:.2f}, angle: {math.degrees(new_angle):.2f}")
                
                # Print detailed debug info
                print(f"PHYSICS: speeds={speeds.vx:.2f},{speeds.vy:.2f},{speeds.omega:.2f} → " +
                    f"pos=({new_x:.2f},{new_y:.2f},{math.degrees(new_angle):.1f}°)")
                
        except Exception as e:
            # Just print the exception and continue - don't want to crash simulation
            print(f"Physics simulation error: {e}")

        # Publish robot pose to NetworkTables for dashboard
        try:
            field_table = ntcore.NetworkTableInstance.getDefault().getTable("field")
            
            # Publish individual components
            x_pub = field_table.getDoubleTopic("robot_x").publish()
            y_pub = field_table.getDoubleTopic("robot_y").publish()
            rot_pub = field_table.getDoubleTopic("robot_rotation").publish()
            
            x_pub.set(self.position.x)
            y_pub.set(self.position.y)
            rot_pub.set(math.degrees(self.position.rotation().radians()))
            
            # Add verbose logging for debugging
            print(f"Published to NT - x: {self.position.x:.2f}, y: {self.position.y:.2f}, rot: {math.degrees(self.position.rotation().radians()):.2f}")
            
        except Exception as e:
            print(f"Error publishing robot pose: {e}")