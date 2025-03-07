"""
Default command for teleop drive control.
"""
import math
import commands2
import wpilib
from typing import Callable

<<<<<<< HEAD
from subsystems.drivesubsystem import DriveSubsystem
from constants import *
=======
from robot.subsystems.drivesubsystem import DriveSubsystem
from robot.constants import *
>>>>>>> 0f35bd26675a1644dc9f6f438f5c9e4297dc0f25

class DefaultDriveCommand(commands2.Command):
    """
    A command to drive the robot with joystick input.
    """
    
    def __init__(
        self,
        drive_subsystem: DriveSubsystem,
        x_speed_supplier: Callable[[], float],
        y_speed_supplier: Callable[[], float],
        rot_supplier: Callable[[], float],
        precision_mode_supplier: Callable[[], bool]
    ) -> None:
        """
        Creates a new DefaultDriveCommand.
        
        Parameters
        ----------
        drive_subsystem : DriveSubsystem
            The drive subsystem this command will run on
        x_speed_supplier : Callable[[], float]
            Function that supplies the x speed
        y_speed_supplier : Callable[[], float]
            Function that supplies the y speed
        rot_supplier : Callable[[], float]
            Function that supplies the rotation speed
        precision_mode_supplier : Callable[[], bool]
            Function that supplies whether precision mode is active
        """
        super().__init__()
        
        self.drive_subsystem = drive_subsystem
        self.x_speed_supplier = x_speed_supplier
        self.y_speed_supplier = y_speed_supplier
        self.rot_supplier = rot_supplier
        self.precision_mode_supplier = precision_mode_supplier
        
        self.addRequirements(drive_subsystem)
        
    def execute(self) -> None:
        """
        Called every time the scheduler runs while the command is scheduled.
        """
        # Get the joystick inputs
        x_speed = self.x_speed_supplier()
        y_speed = self.y_speed_supplier()
        rot = self.rot_supplier()
        
        # Apply deadband
        x_speed = self._apply_deadband(x_speed)
        y_speed = self._apply_deadband(y_speed)
        rot = self._apply_deadband(rot)
        
        # Square the inputs for better control
        x_speed = math.copysign(x_speed * x_speed, x_speed)
        y_speed = math.copysign(y_speed * y_speed, y_speed)
        rot = math.copysign(rot * rot, rot)
        
        # Apply precision mode if active
        if self.precision_mode_supplier():
            x_speed *= 0.5
            y_speed *= 0.5
            rot *= 0.5
            
        # Convert to meters per second
        x_speed *= SWERVE_MAX_SPEED_FPS
        y_speed *= SWERVE_MAX_SPEED_FPS
        rot *= SWERVE_MAX_ANGULAR_SPEED
        
        # Drive
        self.drive_subsystem.drive(x_speed, y_speed, rot)
<<<<<<< HEAD
        print("defaultDriveCommand.execute(): ", x_speed, y_speed, rot)
=======
>>>>>>> 0f35bd26675a1644dc9f6f438f5c9e4297dc0f25
        
    def _apply_deadband(self, value: float) -> float:
        """
        Applies a deadband to the given value.
        
        Parameters
        ----------
        value : float
            The value to apply the deadband to
            
        Returns
        -------
        float
            The value with the deadband applied
        """
        if abs(value) < DEADBAND:
            return 0.0
        return (value - math.copysign(DEADBAND, value)) / (1.0 - DEADBAND)
        
    def end(self, interrupted: bool) -> None:
        """
        Called when the command ends.
        """
        self.drive_subsystem.drive(0, 0, 0)