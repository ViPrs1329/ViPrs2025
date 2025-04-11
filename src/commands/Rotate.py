# Rotate.py
import commands2
import wpilib
from wpimath.kinematics import ChassisSpeeds
from subsystems.SwerveDriveSubsystem import DriveTrain

class Rotate(commands2.Command):
    """
    Command to rotate the robot by a specified angle in place.
    Uses the gyro to track progress and stops when the rotation is complete.
    
    Positive angles rotate clockwise, negative angles rotate counter-clockwise.
    """
    
    def __init__(self, drivetrain: DriveTrain, angle_degrees: float, clockwise=True, max_speed: float = 0.3, 
                 timeout_seconds: float = 3.0, tolerance_degrees: float = 12.0):
        """
        Create a command to rotate the robot by a specific angle.
        
        Args:
            drivetrain: The drive subsystem to use
            angle_degrees: The angle to rotate in degrees (positive = clockwise)
            max_speed: Maximum rotation speed (0 to 1)
            timeout_seconds: Maximum time to spend trying to reach the target angle
            tolerance_degrees: Tolerance in degrees for considering the rotation complete
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.timer = wpilib.Timer()
        
        # Parameters
        self.angle_to_rotate = angle_degrees  # Target rotation amount in degrees
        self.max_speed = max_speed
        self.timeout = timeout_seconds
        self.tolerance = tolerance_degrees
        self.clockwise = clockwise
        
        # PID constants for rotation control
        self.kP = 0.01  # Proportional gain
        self.kI = 0.00   # Integral gain
        self.kD = 0.0 # Derivative gain
        
        # Runtime variables
        self.starting_angle = 0.0
        self.target_angle = 0.0
        self.last_error = 0.0
        self.error_sum = 0.0
        
    def initialize(self):
        """Called when the command is initially scheduled."""
        # Record the starting angle
        self.starting_angle = self.drivetrain.gyro.get_yaw().value_as_double
        # self.target_angle = self.starting_angle + self.angle_to_rotate
        self.target_angle = self.starting_angle + 180
        
        # Normalize the target angle to be between -180 and 180
        if self.target_angle > 180.0:
            self.target_angle -= 360.0
        elif self.target_angle < -180.0:
            self.target_angle += 360.0
            
        print(f"Starting rotation: current={self.starting_angle:.1f}°, " +
              f"target={self.target_angle:.1f}°, delta={self.angle_to_rotate:.1f}°")
        
        # Reset PID variables
        self.last_error = 0.0
        self.error_sum = 0.0
        
        # Reset the timer
        self.timer.reset()
        self.timer.start()
        
    def execute(self):
        """Called repeatedly when this Command is scheduled to run."""
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        
        # Calculate the error (how far we are from the target angle)
        error = self.target_angle - current_angle
        
        # Normalize the error to be between -180 and 180
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
            
        # Calculate derivative term (rate of change of error)
        derivative = error - self.last_error
        self.last_error = error
        
        # Update integral term with anti-windup
        if abs(error) < 30.0:  # Only accumulate when close to target
            self.error_sum += error
            # Clamp integral term to prevent excessive buildup
            self.error_sum = max(-50, min(50, self.error_sum))
        
        # Calculate PID terms
        p_term = self.kP * error
        i_term = self.kI * self.error_sum
        d_term = self.kD * derivative
        
        # Calculate the rotation speed
        rotation_speed = p_term + i_term + d_term
        
        # Clamp the rotation speed between -max_speed and max_speed
        # rotation_speed = max(-self.max_speed, min(self.max_speed, rotation_speed))
        rotation_speed = -self.max_speed if self.clockwise else self.max_speed
        
        # Set a minimum rotation speed to overcome friction
        # if abs(rotation_speed) < 0.15 and abs(error) > self.tolerance:
        #     rotation_speed = 0.15 if error > 0 else -0.15
            
        # Create a ChassisSpeeds object with only rotation
        speeds = ChassisSpeeds(0, 0, rotation_speed)
        
        # Drive the robot
        self.drivetrain.manualDriveFromChassisSpeeds(speeds)
        
        # Debug output (only periodically to reduce console spam)
        if self.timer.advanceIfElapsed(0.2):  # Print every 0.2 seconds
            elapsed = self.timer.get()
            print(f"Rotation: current={current_angle:.1f}°, target={self.target_angle:.1f}°, " +
                  f"error={error:.1f}°, speed={rotation_speed:.2f}, time={elapsed:.1f}s")
    
    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        # Stop the robot
        self.drivetrain.stopMotors()
        
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        error = self.target_angle - current_angle
        
        # Normalize the error
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
            
        if interrupted:
            print(f"Rotation interrupted! Final angle: {current_angle:.1f}°, " +
                  f"target: {self.target_angle:.1f}°, error: {error:.1f}°")
        else:
            print(f"Rotation completed! Final angle: {current_angle:.1f}°, " +
                  f"target: {self.target_angle:.1f}°, error: {error:.1f}°")
        
        self.timer.stop()
    
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        current_angle = self.drivetrain.gyro.get_yaw().value_as_double
        
        # Calculate how far we are from the target
        error = self.target_angle - current_angle
        
        # Normalize the error
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
            
        # Check if we're within tolerance
        is_on_target = abs(error) <= self.tolerance
        
        # Also set a maximum time for the rotation (safety)
        timed_out = self.timer.get() > self.timeout
        
        if timed_out and not is_on_target:
            print(f"Rotation timed out after {self.timeout} seconds!")
            
        return is_on_target or timed_out