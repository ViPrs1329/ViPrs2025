import wpilib
import ntcore
from wpimath.geometry import Rotation2d

class Pigeon2:
    """Simulation for a Pigeon2 IMU."""
    
    def __init__(self, device_id):
        self.device_id = device_id
        self._yaw = 0
        self._pitch = 0
        self._roll = 0
        
        # Create a NetworkTables entry for monitoring/control
        nt = ntcore.NetworkTableInstance.getDefault()
        self.table = nt.getTable(f"Sim/Pigeon2/{device_id}")
        self.yaw_pub = self.table.getDoubleTopic("yaw").publish()
        self.pitch_pub = self.table.getDoubleTopic("pitch").publish()
        self.roll_pub = self.table.getDoubleTopic("roll").publish()
        
        # Initialize values
        self.yaw_pub.set(0)
        self.pitch_pub.set(0)
        self.roll_pub.set(0)
    
    def set_yaw(self, yaw):
        """Set the yaw angle."""
        self._yaw = yaw
        self.yaw_pub.set(yaw)
    
    def get_yaw(self):
        """Get the yaw angle."""
        # Check if value updated from NetworkTables
        self._yaw = self.table.getDoubleTopic("yaw").subscribe(self._yaw).get()
        return SimStatusSignal(self._yaw)
    
    def get_pitch(self):
        """Get the pitch angle."""
        self._pitch = self.table.getDoubleTopic("pitch").subscribe(self._pitch).get()
        return SimStatusSignal(self._pitch)
    
    def get_roll(self):
        """Get the roll angle."""
        self._roll = self.table.getDoubleTopic("roll").subscribe(self._roll).get()
        return SimStatusSignal(self._roll)

class SimStatusSignal:
    """Simulation for a status signal returned by Phoenix 6 devices."""
    
    def __init__(self, value):
        self.value_as_double = value

class CANcoder:
    """Simulation for a CANcoder."""
    
    def __init__(self, device_id):
        self.device_id = device_id
        self._position = 0.0  # Absolute position in rotations (0-1)
        
        # Create a NetworkTables entry for monitoring/control
        nt = ntcore.NetworkTableInstance.getDefault()
        self.table = nt.getTable(f"Sim/CANcoder/{device_id}")
        self.position_pub = self.table.getDoubleTopic("position").publish()
        self.position_pub.set(0)
    
    def get_position(self):
        """Get the absolute position."""
        # Check if value updated from NetworkTables
        self._position = self.table.getDoubleTopic("position").subscribe(self._position).get()
        return SimStatusSignal(self._position)
    
    def get_absolute_position(self):
        """Get the absolute position."""
        # Check if value updated from NetworkTables
        self._position = self.table.getDoubleTopic("position").subscribe(self._position).get()
        return SimStatusSignal(self._position)