# src/phoenix6/hardware.py
import wpilib
from wpimath.geometry import Rotation2d

# Only use simulation classes if in simulation mode
if wpilib.RobotBase.isSimulation():
    print("Using Phoenix 6 simulation classes")
    
    class Pigeon2:
        """Simulation for a Pigeon2 IMU."""
        
        def __init__(self, device_id):
            self.device_id = device_id
            self._yaw = 0
            self._pitch = 0
            self._roll = 0
            print(f"Created simulation Pigeon2 ID={device_id}")
        
        def set_yaw(self, yaw):
            """Set the yaw angle."""
            self._yaw = yaw
        
        def get_yaw(self):
            """Get the yaw angle."""
            return SimStatusSignal(self._yaw)
        
        def get_pitch(self):
            """Get the pitch angle."""
            return SimStatusSignal(self._pitch)
        
        def get_roll(self):
            """Get the roll angle."""
            return SimStatusSignal(self._roll)

    class SimStatusSignal:
        """Simulation for a status signal returned by Phoenix 6 devices."""
        
        def __init__(self, value):
            self.value_as_double = value
            self._value = value

    class CANcoder:
        """Simulation for a CANcoder."""
        
        def __init__(self, device_id):
            self.device_id = device_id
            self._position = 0.0  # Absolute position in rotations (0-1)
            print(f"Created simulation CANcoder ID={device_id}")
        
        def get_position(self):
            """Get the absolute position."""
            return SimStatusSignal(self._position)
        
        def get_absolute_position(self):
            """Get the absolute position."""
            return SimStatusSignal(self._position)
else:
    # Import the real Phoenix 6 library when not in simulation
    try:
        from phoenix6.hardware import (
            Pigeon2,
            CANcoder
        )
    except ImportError:
        print("Warning: Could not import real Phoenix 6 hardware library")
        
        # Create dummy classes to avoid errors
        class Pigeon2:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("Phoenix 6 hardware library not available")
                
        class CANcoder:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("Phoenix 6 hardware library not available")