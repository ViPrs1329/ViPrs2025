import wpilib
import ntcore

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
        
        # Use a simulated motor
        pwm_channel = (device_id % 20)  # Make sure it stays in valid PWM range
        # Only create PWM if we don't exceed available channels
        if pwm_channel < wpilib.PWM.kPwmChannels:
            self.pwm = wpilib.PWM(pwm_channel)
            self.sim_motor = wpilib.simulation.PWMSim(pwm_channel)
        else:
            self.pwm = None
            self.sim_motor = None
        
        # Create NetworkTables entries for monitoring
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(f"Sim/SparkMax/{device_id}")
        self.speed_pub = self.table.getDoubleTopic("speed").publish()
        self.position_pub = self.table.getDoubleTopic("position").publish()
        self.temperature_pub = self.table.getDoubleTopic("temperature").publish()
        self.current_pub = self.table.getDoubleTopic("current").publish()
        
        # Initialize values
        self.speed_pub.set(0)
        self.position_pub.set(0)
        self.temperature_pub.set(30)
        self.current_pub.set(0)
        
        print(f"Created simulation SparkMax ID={device_id}")
    
    def set(self, speed):
        """Set the motor speed."""
        self._speed = speed if not self._inverted else -speed
        
        # Update PWM if available
        if self.pwm:
            try:
                self.pwm.setSpeed(self._speed)
            except:
                pass  # Ignore PWM errors in simulation
        
        # Update NetworkTables
        self.speed_pub.set(self._speed)
        self.current_pub.set(abs(self._speed * 30))  # Simulate current based on speed
        
        # Simulate temperature rising with speed
        self._temperature = 30 + abs(self._speed * 10)
        self.temperature_pub.set(self._temperature)
        
        # Update position based on speed (simple simulation)
        self._position += self._speed * 0.02  # Assuming 20ms update rate
        self.position_pub.set(self._position)
    
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

# Alias SparkFlex to the same simulation class for now
SimSparkFlex = SimSparkMax

class SimEncoder:
    """Simulation for a SparkMax encoder."""
    
    class Type:
        kHallSensor = 0
        kQuadrature = 1
    
    def __init__(self, spark_max):
        self.spark_max = spark_max
    
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