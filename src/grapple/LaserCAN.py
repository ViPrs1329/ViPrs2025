import wpilib
import struct

class LaserCAN:
    # API IDs extracted from lasercan.rs
    LASERCAN_API_GET_MEASUREMENT = 0x01
    LASERCAN_API_SET_RANGING_MODE = 0x02
    LASERCAN_API_SET_TIMING_BUDGET = 0x03
    LASERCAN_API_SET_ROI = 0x04

    def __init__(self, can_id: int):
        """Initialize LaserCAN sensor with a given CAN ID."""
        self.device = wpilib.CAN(
            deviceId=can_id,
            deviceManufacturer=wpilib.CAN.kTeamManufacturer,
            deviceType=wpilib.CAN.kMiscellaneous
        )
        self.can_data = wpilib.CANData()

    def send_command(self, api_id: int, data: bytes = b''):
        """Send a command to the LaserCAN device over CAN."""
        if len(data) > 8:
            raise ValueError("Data length exceeds 8 bytes.")
        self.device.writePacket(data.ljust(8, b'\x00'), api_id)

    def get_measurement(self):
        """Request a measurement from the LaserCAN sensor."""
        self.send_command(self.LASERCAN_API_GET_MEASUREMENT)
        if self.device.readPacketTimeout(self.LASERCAN_API_GET_MEASUREMENT, 100, self.can_data):
            data = self.can_data.data
            distance_mm = struct.unpack_from('<H', data, 0)[0]  # 2-byte little-endian
            status = data[2]  # 1-byte status
            return distance_mm, status
        return None

    def set_ranging_mode(self, mode: int):
        """Set the ranging mode of the sensor."""
        self.send_command(self.LASERCAN_API_SET_RANGING_MODE, struct.pack('<B', mode))

    def set_timing_budget(self, budget: int):
        """Set the timing budget for measurements."""
        self.send_command(self.LASERCAN_API_SET_TIMING_BUDGET, struct.pack('<H', budget))

    def set_roi(self, x: int, y: int, width: int, height: int):
        """Set the region of interest for the sensor."""
        roi_data = struct.pack('<BBBB', x, y, width, height)
        self.send_command(self.LASERCAN_API_SET_ROI, roi_data)

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