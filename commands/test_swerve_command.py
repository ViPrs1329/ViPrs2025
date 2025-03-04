import commands2
import wpilib
from commands2.command import Command
from subsystems.drive_subsystem import DriveSubsystem

class TestSwerveCommand(Command):
    """
    A command to test basic swerve drive functionality.
    Tests:
    1. Module rotation (each module should rotate in place)
    2. Module drive (each module should drive forward)
    3. Robot movement (forward, strafe, rotate)
    """
    
    def __init__(self, drive_subsystem: DriveSubsystem):
        super().__init__()
        self.drive = drive_subsystem
        self.addRequirements(drive_subsystem)
        
        # Test state variables
        self.current_test = 0
        self.timer = wpilib.Timer()
        self.test_states = [
            self.test_module_rotation,
            self.test_module_drive,
            self.test_robot_movement
        ]
    
    def initialize(self):
        """Called when the command is initially scheduled."""
        self.current_test = 0
        self.timer.restart()
        wpilib.SmartDashboard.putString("Test State", "Starting Module Rotation Test")
    
    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        # Run the current test
        self.test_states[self.current_test]()
    
    def test_module_rotation(self):
        """Test each module's rotation."""
        time = self.timer.get()
        
        if time < 2.0:  # First 2 seconds: rotate all modules to 90 degrees
            wpilib.SmartDashboard.putString("Test State", "Rotating modules to 90 degrees")
            self.drive.set_module_states([
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(90))
            ])
        elif time < 4.0:  # Next 2 seconds: rotate all modules to -90 degrees
            wpilib.SmartDashboard.putString("Test State", "Rotating modules to -90 degrees")
            self.drive.set_module_states([
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(-90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(-90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(-90)),
                commands2.SwerveModuleState(0, commands2.Rotation2d.fromDegrees(-90))
            ])
        else:  # Move to next test
            self.current_test += 1
            self.timer.restart()
            wpilib.SmartDashboard.putString("Test State", "Starting Module Drive Test")
    
    def test_module_drive(self):
        """Test each module's drive motor."""
        time = self.timer.get()
        
        if time < 2.0:  # Drive all modules forward
            wpilib.SmartDashboard.putString("Test State", "Driving modules forward")
            self.drive.set_module_states([
                commands2.SwerveModuleState(1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(1.0, commands2.Rotation2d.fromDegrees(0))
            ])
        elif time < 4.0:  # Drive all modules backward
            wpilib.SmartDashboard.putString("Test State", "Driving modules backward")
            self.drive.set_module_states([
                commands2.SwerveModuleState(-1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(-1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(-1.0, commands2.Rotation2d.fromDegrees(0)),
                commands2.SwerveModuleState(-1.0, commands2.Rotation2d.fromDegrees(0))
            ])
        else:  # Move to next test
            self.current_test += 1
            self.timer.restart()
            wpilib.SmartDashboard.putString("Test State", "Starting Robot Movement Test")
    
    def test_robot_movement(self):
        """Test overall robot movement."""
        time = self.timer.get()
        
        if time < 2.0:  # Drive forward
            wpilib.SmartDashboard.putString("Test State", "Driving robot forward")
            self.drive.drive(1.0, 0.0, 0.0)
        elif time < 4.0:  # Strafe right
            wpilib.SmartDashboard.putString("Test State", "Strafing robot right")
            self.drive.drive(0.0, 1.0, 0.0)
        elif time < 6.0:  # Rotate clockwise
            wpilib.SmartDashboard.putString("Test State", "Rotating robot clockwise")
            self.drive.drive(0.0, 0.0, 1.0)
        else:  # End test
            self.current_test += 1
            wpilib.SmartDashboard.putString("Test State", "Tests Complete")
    
    def end(self, interrupted: bool):
        """Called when the command ends."""
        # Stop all movement
        self.drive.drive(0.0, 0.0, 0.0)
        wpilib.SmartDashboard.putString("Test State", "Test Ended")
    
    def isFinished(self) -> bool:
        """Returns true when all tests are complete."""
        return self.current_test >= len(self.test_states) 