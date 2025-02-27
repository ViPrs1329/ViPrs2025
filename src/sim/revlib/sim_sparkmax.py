import wpilib

class SparkRelativeEncoder:
    """Simulation for SparkRelativeEncoder"""
    
    class Type:
        kHallSensor = 0
        kQuadrature = 1
    
    def __init__(self, spark_max):
        self.spark_max = spark_max
        self._position = 0
        self._velocity = 0
    
    def getPosition(self):
        """Get the encoder position."""
        return self.spark_max._position * 42  # Convert to encoder counts
    
    def getVelocity(self):
        """Get the encoder velocity."""
        return self.spark_max._speed * 42 * 60  # RPM
    
    def setPosition(self, position):
        """Set the encoder position."""
        self.spark_max._position = position / 42

class SimSparkMax:
    """Simulation replacement for rev.SparkMax"""
    
    class MotorType:
        kBrushless = 0
        kBrushed = 1
    
    class IdleMode:
        kCoast = 0
        kBrake = 1
    
    def __init__(self, device_id, motor_type):
        self.device_id = device_id
        self.motor_type = motor_type
        self._speed = 0
        self._inverted = False
        self._position = 0
        self._voltage = 0
        self._temperature = 30  # Default temp 30°C
        
        # Create sim objects through WPILib
        self.sim_motor = wpilib.simulation.PWMSim(device_id % 20)  # Use modulo to keep in PWM range
        self.sim_collection = self.sim_motor.getSimCollection()
        
        # Create a NetworkTables entry for monitoring
        import ntcore
        nt = ntcore.NetworkTableInstance.getDefault()
        self.table = nt.getTable(f"Sim/SparkMax/{device_id}")
        self.speed_pub = self.table.getDoubleTopic("speed").publish()
        self.position_pub = self.table.getDoubleTopic("position").publish()
        self.temperature_pub = self.table.getDoubleTopic("temperature").publish()
    
    def set(self, speed):
        """Set the motor speed."""
        self._speed = speed if not self._inverted else -speed
        self.sim_motor.setSpeed(self._speed)
        self.speed_pub.set(self._speed)
    
    def setVoltage(self, voltage):
        """Set the motor voltage."""
        self._voltage = voltage
        max_voltage = 12.0
        self.set(voltage / max_voltage)
    
    def getEncoder(self, encoderType=None, counts_per_rev=42):
        """Get an encoder object."""
        return SimEncoder(self)
    
    def setInverted(self, inverted):
        """Set whether the motor is inverted."""
        self._inverted = inverted
    
    def getInverted(self):
        """Check if the motor is inverted."""
        return self._inverted
    
    def getOutputCurrent(self):
        """Get the output current."""
        return abs(self._speed * 30)  # Simulate current based on speed
    
    def getMotorTemperature(self):
        """Get the motor temperature."""
        # Simulate temperature rising with speed
        self._temperature = 30 + abs(self._speed) * 10
        return self._temperature
    
    def restoreFactoryDefaults(self):
        """Reset to factory defaults."""
        pass
    
    def getDeviceId(self):
        """Get the device ID."""
        return self.device_id
    
    def follow(self, leader, invert=False):
        """Follow another motor controller."""
        pass
    
    def configure(self, config, reset_mode=None, persist_mode=None):
        """Configure the motor controller."""
        pass
    
    def periodic(self):
        """Update simulation state."""
        # Update position based on speed
        self._position += self._speed * 0.02  # 20ms update
        self.position_pub.set(self._position)
        self.temperature_pub.set(self._temperature)

# Alias SparkFlex to the same simulation class for now
SimSparkFlex = SimSparkMax

class SimEncoder:
    """Simulation for a SparkMax encoder."""
    
    def __init__(self, spark_max):
        self.spark_max = spark_max
        self._position = 0
        self._velocity = 0
    
    def getPosition(self):
        """Get the encoder position."""
        return self.spark_max._position * 42  # Convert to encoder counts
    
    def getVelocity(self):
        """Get the encoder velocity."""
        return self.spark_max._speed * 42 * 60  # RPM
    
    def setPosition(self, position):
        """Set the encoder position."""
        self.spark_max._position = position / 42

class SparkBaseConfig:
    """Simulation for SparkBaseConfig."""
    
    class IdleMode:
        kCoast = 0
        kBrake = 1
    
    def __init__(self):
        self.idle_mode = self.IdleMode.kCoast
        self.current_limit = 40
    
    def setIdleMode(self, mode):
        """Set the idle mode."""
        self.idle_mode = mode
        return self
    
    def smartCurrentLimit(self, limit):
        """Set the current limit."""
        self.current_limit = limit
        return self