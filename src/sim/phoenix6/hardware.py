# sim/phoenix6/hardware.py
"""
Simulation implementations for Phoenix 6 hardware devices.

This module provides simulated versions of:
- Pigeon2 (gyro/IMU)
- CANcoder (absolute encoder)
- CANrange (distance sensor)
"""

import ntcore
import math
import wpilib

class StatusSignal:
    """Base class for simulated status signals."""
    
    def __init__(self, value):
        """
        Initialize the simulated status signal.
        
        Args:
            value: The initial value
        """
        self.value_as_double = value
        self._value = value


class Pigeon2:
    """Simulation for a Pigeon2 IMU."""
    
    def __init__(self, device_id):
        """
        Initialize the simulated Pigeon2.
        
        Args:
            device_id (int): The CAN ID of the device
        """
        self.device_id = device_id
        
        # Initialize IMU values
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        
        # Create NetworkTables entries
        self._setup_network_tables()
        
        print(f"Created simulated Pigeon2 with ID {device_id}")
    
    def _setup_network_tables(self):
        """Set up NetworkTables entries for simulation."""
        try:
            self._nt = ntcore.NetworkTableInstance.getDefault()
            self._table = self._nt.getTable(f"Sim/Pigeon2/{self.device_id}")
            
            # Create publishers
            self._yaw_pub = self._table.getDoubleTopic("yaw").publish()
            self._pitch_pub = self._table.getDoubleTopic("pitch").publish()
            self._roll_pub = self._table.getDoubleTopic("roll").publish()
            
            # Initialize values
            self._yaw_pub.set(0.0)
            self._pitch_pub.set(0.0)
            self._roll_pub.set(0.0)
            
            # Create subscribers
            self._yaw_sub = self._table.getDoubleTopic("yaw").subscribe(0.0)
            self._pitch_sub = self._table.getDoubleTopic("pitch").subscribe(0.0)
            self._roll_sub = self._table.getDoubleTopic("roll").subscribe(0.0)
            
        except Exception as e:
            print(f"Warning: Could not set up NetworkTables for Pigeon2 {self.device_id}: {e}")
    
    def set_yaw(self, yaw):
        """
        Set the yaw angle of the gyro.
        
        Args:
            yaw (float): The yaw angle in degrees
        """
        self._yaw = yaw
        self._yaw_pub.set(yaw)
    
    def get_yaw(self):
        """
        Get the yaw angle of the gyro.
        
        Returns:
            StatusSignal: The yaw angle status signal
        """
        # Check for updates from NetworkTables
        self._yaw = self._yaw_sub.get()
        return StatusSignal(self._yaw)
    
    def get_pitch(self):
        """
        Get the pitch angle of the gyro.
        
        Returns:
            StatusSignal: The pitch angle status signal
        """
        # Check for updates from NetworkTables
        self._pitch = self._pitch_sub.get()
        return StatusSignal(self._pitch)
    
    def get_roll(self):
        """
        Get the roll angle of the gyro.
        
        Returns:
            StatusSignal: The roll angle status signal
        """
        # Check for updates from NetworkTables
        self._roll = self._roll_sub.get()
        return StatusSignal(self._roll)


class CANcoder:
    """Simulation for a CANcoder absolute encoder."""
    
    def __init__(self, device_id):
        """
        Initialize the simulated CANcoder.
        
        Args:
            device_id (int): The CAN ID of the device
        """
        self.device_id = device_id
        
        # Initialize encoder values
        self._position = 0.0  # In rotations (0-1)
        self._velocity = 0.0  # In rotations per second
        self._absolute_position = 0.0  # In rotations (0-1)
        
        # Create NetworkTables entries
        self._setup_network_tables()
        
        print(f"Created simulated CANcoder with ID {device_id}")
    
    def _setup_network_tables(self):
        """Set up NetworkTables entries for simulation."""
        try:
            self._nt = ntcore.NetworkTableInstance.getDefault()
            self._table = self._nt.getTable(f"Sim/CANcoder/{self.device_id}")
            
            # Create publishers
            self._position_pub = self._table.getDoubleTopic("position").publish()
            self._velocity_pub = self._table.getDoubleTopic("velocity").publish()
            self._absolute_position_pub = self._table.getDoubleTopic("absolutePosition").publish()
            
            # Initialize values
            self._position_pub.set(0.0)
            self._velocity_pub.set(0.0)
            self._absolute_position_pub.set(0.0)
            
            # Create subscribers
            self._position_sub = self._table.getDoubleTopic("position").subscribe(0.0)
            self._velocity_sub = self._table.getDoubleTopic("velocity").subscribe(0.0)
            self._absolute_position_sub = self._table.getDoubleTopic("absolutePosition").subscribe(0.0)
            
        except Exception as e:
            print(f"Warning: Could not set up NetworkTables for CANcoder {self.device_id}: {e}")
    
    def get_position(self):
        """
        Get the position of the encoder.
        
        Returns:
            StatusSignal: The position status signal (in rotations)
        """
        # Check for updates from NetworkTables
        self._position = self._position_sub.get()
        return StatusSignal(self._position)
    
    def get_velocity(self):
        """
        Get the velocity of the encoder.
        
        Returns:
            StatusSignal: The velocity status signal (in rotations per second)
        """
        # Check for updates from NetworkTables
        self._velocity = self._velocity_sub.get()
        return StatusSignal(self._velocity)
    
    def get_absolute_position(self):
        """
        Get the absolute position of the encoder.
        
        Returns:
            StatusSignal: The absolute position status signal (in rotations)
        """
        # Check for updates from NetworkTables
        self._absolute_position = self._absolute_position_sub.get()
        return StatusSignal(self._absolute_position)


class CANrange:
    """Simulation for a LaserCAN distance sensor."""
    
    def __init__(self, device_id):
        """
        Initialize the simulated distance sensor.
        
        Args:
            device_id (int): The CAN ID of the device
        """
        self.device_id = device_id
        
        # Initialize sensor values
        self._distance = 8.0  # 8 meters (8000mm)
        self._status = 0  # 0 = Good
        
        # Create NetworkTables entries
        self._setup_network_tables()
        
        print(f"Created simulated CANrange with ID {device_id}")
    
    def _setup_network_tables(self):
        """Set up NetworkTables entries for simulation."""
        try:
            self._nt = ntcore.NetworkTableInstance.getDefault()
            self._table = self._nt.getTable(f"Sim/CANrange/{self.device_id}")
            
            # Create publishers
            self._distance_pub = self._table.getDoubleTopic("distance").publish()
            self._status_pub = self._table.getIntegerTopic("status").publish()
            
            # Initialize values
            self._distance_pub.set(self._distance)
            self._status_pub.set(self._status)
            
            # Create subscribers
            self._distance_sub = self._table.getDoubleTopic("distance").subscribe(self._distance)
            self._status_sub = self._table.getIntegerTopic("status").subscribe(self._status)
            
        except Exception as e:
            print(f"Warning: Could not set up NetworkTables for CANrange {self.device_id}: {e}")
    
    def get_distance(self):
        """
        Get the distance measured by the sensor.
        
        Returns:
            StatusSignal: The distance status signal (in meters)
        """
        # Check for updates from NetworkTables
        self._distance = self._distance_sub.get()
        return StatusSignal(self._distance)
    
    def is_good(self):
        """
        Check if the sensor status is good.
        
        Returns:
            StatusSignal: The status signal (0 = Good)
        """
        # Check for updates from NetworkTables
        self._status = self._status_sub.get()
        return StatusSignal(self._status == 0)
    
    def get_measurement(self):
        """
        Get the measurement from the sensor.
        
        Returns:
            tuple: (distance_mm, status)
        """
        # Check for updates from NetworkTables
        self._distance = self._distance_sub.get()
        self._status = self._status_sub.get()
        
        # Convert meters to millimeters
        distance_mm = self._distance * 1000.0
        
        return distance_mm, self._status
    
    def set_simulated_distance(self, distance_mm, status=0):
        """
        Set the simulated distance and status.
        
        Args:
            distance_mm (float): The distance in millimeters
            status (int, optional): The status code. Defaults to 0 (Good).
        """
        # Convert millimeters to meters
        self._distance = distance_mm / 1000.0
        self._status = status
        
        # Update NetworkTables
        self._distance_pub.set(self._distance)
        self._status_pub.set(self._status)