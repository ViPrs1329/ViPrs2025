# src/sim/revlib/sim_sparkmax.py
import wpilib
import ntcore

class SimEncoder:
    """Simulation for a SparkMax encoder."""
    
    class Type:
        kHallSensor = 0
        kQuadrature = 1
    
    def __init__(self, spark_max, encoder_type=None, counts_per_rev=42):
        self.spark_max = spark_max
        self._position = 0
        self._velocity = 0
        self._position_conversion_factor = 1.0
        self._velocity_conversion_factor = 1.0
    
    def getPosition(self):
        """Get the encoder position."""
        return self.spark_max._position * 42 * self._position_conversion_factor
    
    def getVelocity(self):
        """Get the encoder velocity."""
        return self.spark_max._speed * 42 * 60 * self._velocity_conversion_factor
    
    def setPosition(self, position):
        """Set the encoder position."""
        self.spark_max._position = position / (42 * self._position_conversion_factor)
        
    def setPositionConversionFactor(self, factor):
        """Set position conversion factor."""
        self._position_conversion_factor = factor
        
    def setVelocityConversionFactor(self, factor):
        """Set velocity conversion factor."""
        self._velocity_conversion_factor = factor

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

class SimSparkMax:
    """Simulation replacement for rev.SparkMax"""
    
    class MotorType:
        kBrushless = 0
        kBrushed = 1
    
    class IdleMode:
        kCoast = 0
        kBrake = 1
        
    class ControlType:
        kDutyCycle = 0
        kVelocity = 1
        kVoltage = 2
        kPosition = 3
        kSmartMotion = 4
        kCurrent = 5
        kSmartVelocity = 6
    
    def __init__(self, device_id, motor_type):
        self.device_id = device_id
        self.motor_type = motor_type
        self._speed = 0
        self._inverted = False
        self._position = 0
        self._voltage = 0
        self._temperature = 30  # Default temp 30°C
        self._idle_mode = self.IdleMode.kCoast
        self._current_limit = 40
        self._encoder = None
        self._pid_controller = None
        
        # Use a simulated motor - with a safety check on PWM channels
        # In WPILib 2025, we need to use a different approach than kPwmChannels
        # The default number of PWM channels is typically 10 in WPILib
        MAX_PWM_CHANNELS = 10
        pwm_channel = (device_id % MAX_PWM_CHANNELS)  # Make sure it stays in valid PWM range
        
        # Only try to create a PWM channel if it's in a valid range
        try:
            if pwm_channel < MAX_PWM_CHANNELS:
                self.pwm = wpilib.PWM(pwm_channel)
                self.sim_motor = wpilib.simulation.PWMSim(pwm_channel)
            else:
                self.pwm = None
                self.sim_motor = None
        except Exception as e:
            print(f"Warning: Could not create PWM for device {device_id}: {e}")
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
        if self._encoder is None:
            self._encoder = SimEncoder(self, encoderType, counts_per_rev)
        return self._encoder
    
    def getAbsoluteEncoder(self, encoder_type=None):
        """Get an absolute encoder object."""
        # Create a simulated absolute encoder based on the relative encoder
        from team254.LazySparkMax import SimSparkMaxAbsoluteEncoder
        return SimSparkMaxAbsoluteEncoder(self)
    
    def setIdleMode(self, mode):
        """Set the idle mode."""
        self._idle_mode = mode
    
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
        self._speed = 0
        self._inverted = False
        self._position = 0
        self._idle_mode = self.IdleMode.kCoast
        self._current_limit = 40
    
    def getDeviceId(self):
        """Get the device ID."""
        return self.device_id
    
    def follow(self, leader, invert=False):
        """Follow another motor controller."""
        pass
    
    def enableVoltageCompensation(self, voltage):
        """Enable voltage compensation."""
        pass
    
    def disableVoltageCompensation(self):
        """Disable voltage compensation."""
        pass
    
    def setSmartCurrentLimit(self, limit):
        """Set the current limit."""
        self._current_limit = limit
    
    def burnFlash(self):
        """Burn the configuration to flash memory."""
        pass
    
    def getPIDController(self):
        """Get a PID controller."""
        if self._pid_controller is None:
            self._pid_controller = SimPIDController(self)
        return self._pid_controller
    
    def configure(self, config, reset_mode=None, persist_mode=None):
        """Configure the motor controller."""
        pass

# Alias SparkFlex to the same simulation class
SimSparkFlex = SimSparkMax

class SimPIDController:
    """Simulation for a SparkMax PID controller."""
    
    def __init__(self, spark_max):
        self.spark_max = spark_max
        self._p = 0.0
        self._i = 0.0
        self._d = 0.0
        self._ff = 0.0
        self._feedback_device = None
    
    def setP(self, p):
        """Set the proportional gain."""
        self._p = p
    
    def setI(self, i):
        """Set the integral gain."""
        self._i = i
    
    def setD(self, d):
        """Set the derivative gain."""
        self._d = d
    
    def setFF(self, ff):
        """Set the feedforward gain."""
        self._ff = ff
    
    def setFeedbackDevice(self, device):
        """Set the feedback device."""
        self._feedback_device = device
    
    def setReference(self, value, ctrl_type, pidSlot=0, arbFeedforward=0.0):
        """Set the reference point."""
        if ctrl_type == SimSparkMax.ControlType.kPosition:
            # For position control, move towards the target
            current_pos = 0.0
            if self._feedback_device is not None:
                try:
                    current_pos = self._feedback_device.getPosition()
                except:
                    current_pos = self.spark_max.getEncoder().getPosition()
            else:
                current_pos = self.spark_max.getEncoder().getPosition()
            
            # Simple proportional control for simulation
            error = value - current_pos
            output = self._p * error
            
            # Limit output
            output = max(min(output, 1.0), -1.0)
            
            # Set motor output
            self.spark_max.set(output)
        elif ctrl_type == SimSparkMax.ControlType.kVelocity:
            # For velocity control, set a speed
            self.spark_max.set(value * 0.01)  # Arbitrary scaling
        else:
            # For other control types, just set the value directly
            self.spark_max.set(value)