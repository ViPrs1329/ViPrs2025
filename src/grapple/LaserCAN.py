import wpilib
import struct

class LaserCAN:
    # API IDs extracted from lasercan.rs
    LASERCAN_API_GET_MEASUREMENT = 0x01
    LASERCAN_API_SET_RANGING_MODE = 0x02
    LASERCAN_API_SET_TIMING_BUDGET = 0x03
    LASERCAN_API_SET_ROI = 0x04
    
    # Status code constants
    STATUS_GOOD = 0
    STATUS_SIGNAL_FAIL = 1
    STATUS_OUT_OF_RANGE = 2
    STATUS_TIMEOUT = 3
    STATUS_ERROR = 255  # Default error status

    def __init__(self, can_id: int):
        """Initialize LaserCAN sensor with a given CAN ID.
        
        Args:
            can_id (int): The CAN ID of the LaserCAN sensor
        """
        self.can_id = can_id
        self.sim_device = None
        
        # Check if we're in simulation mode
        if wpilib.RobotBase.isSimulation():
            # Initialize simulation device
            self.sim_device = SimLaserCAN(can_id)
        else:
            # Initialize real hardware
            self.device = wpilib.CAN(
                deviceId=can_id,
                deviceManufacturer=wpilib.CAN.kTeamManufacturer,
                deviceType=wpilib.CAN.kMiscellaneous
            )
            self.can_data = wpilib.CANData()

    def send_command(self, api_id: int, data: bytes = b''):
        """Send a command to the LaserCAN device over CAN.
        
        Args:
            api_id (int): The API command ID
            data (bytes, optional): Command data. Defaults to empty bytes.
            
        Raises:
            ValueError: If data length exceeds 8 bytes
        """
        if wpilib.RobotBase.isSimulation():
            # In simulation, just ignore commands
            return
            
        if len(data) > 8:
            raise ValueError("Data length exceeds 8 bytes.")
        self.device.writePacket(data.ljust(8, b'\x00'), api_id)

    def get_measurement(self):
        """Request a measurement from the LaserCAN sensor.
        
        Returns:
            tuple: (distance_mm, status) where:
                - distance_mm (int): Distance in millimeters
                - status (int): Status code (0=Good, 1=SignalFail, 2=OutOfRange, etc.)
            or None if no measurement received
        """
        if wpilib.RobotBase.isSimulation():
            if self.sim_device:
                return self.sim_device.get_measurement()
            return 8000, self.STATUS_ERROR  # Default simulation values
            
        # Real hardware implementation
        self.send_command(self.LASERCAN_API_GET_MEASUREMENT)
        if self.device.readPacketTimeout(self.LASERCAN_API_GET_MEASUREMENT, 100, self.can_data):
            data = self.can_data.data
            distance_mm = struct.unpack_from('<H', data, 0)[0]  # 2-byte little-endian
            status = data[2]  # 1-byte status
            return distance_mm, status
        return None

    def get_measurement_safe(self, default_distance=8000, default_status=STATUS_ERROR):
        """Get measurement with fallback values if read fails.
        
        Args:
            default_distance (int, optional): Default distance to return if read fails. 
                Defaults to 8000mm.
            default_status (int, optional): Default status to return if read fails.
                Defaults to STATUS_ERROR (255).
                
        Returns:
            tuple: (distance_mm, status) - either actual values or defaults
        """
        result = self.get_measurement()
        if result is None:
            print(f"Warning: LaserCAN ID {self.can_id} read failed, using defaults")
            return default_distance, default_status
        return result

    def is_in_range(self, min_mm=0, max_mm=8000):
        """Check if the measured distance is within the specified range.
        
        Args:
            min_mm (int, optional): Minimum valid distance in mm. Defaults to 0.
            max_mm (int, optional): Maximum valid distance in mm. Defaults to 8000.
            
        Returns:
            bool: True if measurement is valid and within range, False otherwise
        """
        measurement = self.get_measurement()
        if measurement:
            distance, status = measurement
            return status == self.STATUS_GOOD and min_mm <= distance <= max_mm
        return False

    def is_object_detected(self, threshold_mm=100):
        """Check if an object is detected within the given threshold distance.
        
        Args:
            threshold_mm (int, optional): Distance threshold in mm. Defaults to 100.
            
        Returns:
            bool: True if object detected within threshold, False otherwise
        """
        measurement = self.get_measurement()
        if measurement:
            distance, status = measurement
            return status == self.STATUS_GOOD and distance < threshold_mm
        return False

    def set_ranging_mode(self, mode: int):
        """Set the ranging mode of the sensor.
        
        Args:
            mode (int): The ranging mode to set
                0 = Short range
                1 = Medium range
                2 = Long range
        """
        self.send_command(self.LASERCAN_API_SET_RANGING_MODE, struct.pack('<B', mode))

    def set_timing_budget(self, budget: int):
        """Set the timing budget for measurements.
        
        Args:
            budget (int): Timing budget in milliseconds (typically 20-1000)
                Longer timing budget = more accurate but slower measurements
        """
        self.send_command(self.LASERCAN_API_SET_TIMING_BUDGET, struct.pack('<H', budget))

    def set_roi(self, x: int, y: int, width: int, height: int):
        """Set the region of interest for the sensor.
        
        Args:
            x (int): X coordinate of ROI (0-15)
            y (int): Y coordinate of ROI (0-15)
            width (int): Width of ROI (1-16)
            height (int): Height of ROI (1-16)
        """
        roi_data = struct.pack('<BBBB', x, y, width, height)
        self.send_command(self.LASERCAN_API_SET_ROI, roi_data)


class SimLaserCAN:
    """Simulation implementation of LaserCAN sensor"""
    
    def __init__(self, can_id: int):
        """Initialize simulated LaserCAN sensor.
        
        Args:
            can_id (int): The CAN ID to simulate
        """
        self.can_id = can_id
        self.simulated_distance = 8000  # Default: 8000mm (nothing detected)
        self.simulated_status = 0       # Default: Good status
        
        # Create NetworkTable entries for this sensor
        import ntcore
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.nt_instance.getTable(f"Sim/LaserCAN/{can_id}")
        
        # Create publisher for distance that UI can subscribe to
        self.distance_pub = self.nt_table.getDoubleTopic("distance").publish()
        self.status_pub = self.nt_table.getIntegerTopic("status").publish()
        
        # Initialize published values
        self.distance_pub.set(self.simulated_distance)
        self.status_pub.set(self.simulated_status)
    
    def get_measurement(self):
        """Get simulated measurement.
        
        Returns:
            tuple: (distance_mm, status)
        """
        # Check if values have been updated through NetworkTables
        self.simulated_distance = self.nt_table.getDoubleTopic("distance").subscribe(self.simulated_distance).get()
        self.simulated_status = self.nt_table.getIntegerTopic("status").subscribe(self.simulated_status).get()
        
        return self.simulated_distance, self.simulated_status
    
    def set_simulated_distance(self, distance, status=0):
        """Set the simulated distance and status.
        
        Args:
            distance (float): Distance in millimeters
            status (int, optional): Status code. Defaults to 0 (good).
        """
        self.simulated_distance = distance
        self.simulated_status = status
        
        # Update NetworkTables values
        self.distance_pub.set(distance)
        self.status_pub.set(status)

'''
# Example usage
if __name__ == "__main__":
    laser = LaserCAN(5)  # Initialize with CAN ID 5
    measurement = laser.get_measurement()
    if measurement:
        distance, status = measurement
        print(f"Distance: {distance} mm, Status: {status}")
    else:
        print("No measurement received.")
'''