# Create a file: src/sim/sim_lasercan.py
class SimLaserCAN:
    def __init__(self, can_id):
        self.can_id = can_id
        self.simulated_distance = 1000  # Default 1000mm
        self.simulated_status = 0  # Good status
        
    def get_measurement(self):
        return self.simulated_distance, self.simulated_status
        
    def set_simulated_distance(self, distance, status=0):
        self.simulated_distance = distance
        self.simulated_status = status