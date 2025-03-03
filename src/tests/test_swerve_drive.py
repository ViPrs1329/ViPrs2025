# test_swerve_drive.py
import wpilib
import commands2
import ntcore
import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from subsystems.SwerveDriveSubsystem import SwerveDrive, SwerveModule

class SwerveDriveTestRobot(wpilib.TimedRobot):
    """
    Test program for the SwerveDrive subsystem.
    This can be run in simulation mode to test the SwerveDrive functionality
    without deploying to the robot.
    """
    
    def robotInit(self):
        """Robot initialization code."""
        print("Initializing SwerveDrive Test Robot")
        
        # Create the subsystem
        self.swerveDrive = SwerveDrive()
        
        # Create a joystick for control
        self.controller = wpilib.XboxController(0)
        
        # Create NetworkTables for simulation control and monitoring
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("SwerveDriveTest")
        
        # Create publishers for test controls
        self.test_mode_pub = self.table.getStringTopic("test_mode").publish()
        self.selected_module_pub = self.table.getStringTopic("selected_module").publish()
        self.drive_speed_pub = self.table.getDoubleTopic("drive_speed").publish()
        self.rotation_angle_pub = self.table.getDoubleTopic("rotation_angle").publish()
        
        # Create subscribers for simulation control
        self.test_mode_sub = self.table.getStringTopic("test_mode").subscribe("full_drive")
        self.selected_module_sub = self.table.getStringTopic("selected_module").subscribe("all")
        self.drive_speed_sub = self.table.getDoubleTopic("drive_speed").subscribe(0.0)
        self.rotation_angle_sub = self.table.getDoubleTopic("rotation_angle").subscribe(0.0)
        
        # Initialize values
        self.test_mode_pub.set("full_drive")
        self.selected_module_pub.set("all")
        self.drive_speed_pub.set(0.0)
        self.rotation_angle_pub.set(0.0)
        
        # Command scheduler
        self.scheduler = commands2.CommandScheduler.getInstance()
        
        # Test modes:
        # - "full_drive": Normal drive control with joystick
        # - "single_module_drive": Test drive motor of a single module
        # - "single_module_rotation": Test rotation motor of a single module
        # - "module_state": Test setting a specific state for a module
        self.test_mode = "full_drive"
        self.selected_module = "all"  # "fl", "fr", "bl", "br", or "all"
        
        print("SwerveDrive Test Robot initialized")

    def robotPeriodic(self):
        """Periodic code for all robot modes."""
        # Run the command scheduler
        self.scheduler.run()
        
        # Update test parameters from NetworkTables
        self.test_mode = self.test_mode_sub.get()
        self.selected_module = self.selected_module_sub.get()
        
        # Update dashboard with current state
        self.updateDashboard()

    def updateDashboard(self):
        """Update dashboard with current state."""
        # Get the current pose
        pose = self.swerveDrive.getPose()
        
        # Publish pose data
        pose_table = self.table.getSubTable("pose")
        pose_table.getDoubleTopic("x").publish().set(pose.X())
        pose_table.getDoubleTopic("y").publish().set(pose.Y())
        pose_table.getDoubleTopic("rotation").publish().set(pose.rotation().degrees())
        
        # Publish module states
        modules = {
            "fl": self.swerveDrive.frontLeftModule,
            "fr": self.swerveDrive.frontRightModule,
            "bl": self.swerveDrive.backLeftModule,
            "br": self.swerveDrive.backRightModule
        }
        
        for name, module in modules.items():
            module_table = self.table.getSubTable(f"module_{name}")
            
            # Get current state
            position = module.getPosition()
            
            # Publish data
            module_table.getDoubleTopic("drive_position").publish().set(position.distance)
            module_table.getDoubleTopic("rotation_position").publish().set(position.angle.degrees())
            module_table.getDoubleTopic("drive_current").publish().set(module.drive_current)
            module_table.getDoubleTopic("rotation_current").publish().set(module.rotation_current)

    def teleopInit(self):
        """Initialization code for teleop mode."""
        print("SwerveDrive Test: Teleop Initialized")
        
        # Stop any running commands
        self.scheduler.cancelAll()
        
        # Reset the drive if needed
        if self.controller.getBackButtonPressed():
            self.swerveDrive.resetGyro()

    def teleopPeriodic(self):
        """Periodic code for teleop mode."""
        # Handle different test modes
        if self.test_mode == "full_drive":
            # Normal drive control with joystick
            x_speed = -self.controller.getLeftY() * 2  # m/s
            y_speed = -self.controller.getLeftX() * 2  # m/s
            rot_speed = -self.controller.getRightX() * math.pi  # rad/s
            
            # Apply deadband
            if abs(x_speed) < 0.1:
                x_speed = 0
            if abs(y_speed) < 0.1:
                y_speed = 0
            if abs(rot_speed) < 0.1:
                rot_speed = 0
                
            # Drive the robot
            self.swerveDrive.drive(x_speed, y_speed, rot_speed, True)
            
        elif self.test_mode == "single_module_drive":
            # Test drive motor of a single module
            drive_speed = self.drive_speed_sub.get()
            
            # Get the selected module
            module = self.getSelectedModule()
            if module:
                # Create a state with the current angle and the desired speed
                current_position = module.getPosition()
                desired_state = SwerveModuleState(drive_speed, current_position.angle)
                module.setDesiredState(desired_state)
            else:
                # Stop all modules
                self.swerveDrive.stopMotors()
                
        elif self.test_mode == "single_module_rotation":
            # Test rotation motor of a single module
            rotation_angle = self.rotation_angle_sub.get()
            
            # Get the selected module
            module = self.getSelectedModule()
            if module:
                # Create a state with zero speed and the desired angle
                desired_state = SwerveModuleState(0, Rotation2d.fromDegrees(rotation_angle))
                module.setDesiredState(desired_state)
            else:
                # Stop all modules
                self.swerveDrive.stopMotors()
                
        elif self.test_mode == "module_state":
            # Test setting a specific state for a module
            drive_speed = self.drive_speed_sub.get()
            rotation_angle = self.rotation_angle_sub.get()
            
            # Get the selected module
            module = self.getSelectedModule()
            if module:
                # Create the desired state
                desired_state = SwerveModuleState(drive_speed, Rotation2d.fromDegrees(rotation_angle))
                module.setDesiredState(desired_state)
            else:
                # Stop all modules
                self.swerveDrive.stopMotors()
        
        # Reset gyro if back button is pressed
        if self.controller.getBackButtonPressed():
            self.swerveDrive.resetGyro()

    def getSelectedModule(self):
        """Get the selected module based on the current selection."""
        if self.selected_module == "fl":
            return self.swerveDrive.frontLeftModule
        elif self.selected_module == "fr":
            return self.swerveDrive.frontRightModule
        elif self.selected_module == "bl":
            return self.swerveDrive.backLeftModule
        elif self.selected_module == "br":
            return self.swerveDrive.backRightModule
        else:
            return None  # "all" or invalid selection

    def simulationInit(self):
        """Initialization code for simulation mode."""
        print("SwerveDrive Test: Simulation Initialized")

    def simulationPeriodic(self):
        """Periodic code for simulation mode."""
        # This is where you would update simulation parameters if needed
        pass

if __name__ == "__main__":
    wpilib.run(SwerveDriveTestRobot) 