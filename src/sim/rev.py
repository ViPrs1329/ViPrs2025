# sim/rev.py
"""
Simulation implementation of the REV Robotics motor controllers and related classes.

This module provides simulated replacements for:
- SparkMax and CANSparkMax
- SparkFlex
- SparkMaxAbsoluteEncoder
- SparkRelativeEncoder
- SparkPIDController
"""

import wpilib
import ntcore
import math
from wpimath.geometry import Rotation2d

class SparkMaxAbsoluteEncoder:
    """Simulation for a SparkMax absolute encoder."""
    
    # Enum for encoder types (matches REV API)
    class Type:
        kDutyCycle = 0
    
    def __init__(self, spark_max, encoder_type=None):
        """
        Initialize the simulated absolute encoder.
        
        Args:
            spark_max: The parent SparkMax controller
            encoder_type: The type of encoder (ignored in simulation)
        """
        self.spark_max = spark_max
        self._position = 0.0
        self._velocity = 0.0
        self._position_conversion_factor = 1.0
        self._velocity_conversion_factor = 1.0
        self._zero_offset = 0.0
        self._inverted = False
    
    def getPosition(self):
        """Get the position of the encoder."""
        position = self._position
        if self._inverted:
            position = -position
        return position + self._zero_offset
    
    def getVelocity(self):
        """Get the velocity of the encoder."""
        velocity = self._velocity
        if self._inverted:
            velocity = -velocity
        return velocity
    
    def setPositionConversionFactor(self, factor):
        """Set the position conversion factor."""
        self._position_conversion_factor = factor
    
    def setVelocityConversionFactor(self, factor):
        """Set the velocity conversion factor."""
        self._velocity_conversion_factor = factor
    
    def setZeroOffset(self, offset):
        """Set the zero offset of the encoder."""
        self._zero_offset = offset
    
    def setInverted(self, inverted):
        """Set whether the encoder is inverted."""
        self._inverted = inverted


class SparkRelativeEncoder:
    """Simulation for a SparkMax relative encoder."""
    
    # Enum for encoder types (matches REV API)
    class Type:
        kHallSensor = 0
        kQuadrature = 1
    
    def __init__(self, spark_max, encoder_type=None, counts_per_rev=42):
        """
        Initialize the simulated relative encoder.
        
        Args:
            spark_max: The parent SparkMax controller
            encoder_type: The type of encoder
            counts_per_rev: The encoder counts per revolution
        """
        self.spark_max = spark_max
        self.encoder_type = encoder_type
        self.counts_per_rev = counts_per_rev
        
        self._position = 0.0
        self._velocity = 0.0
        self._position_conversion_factor = 1.0
        self._velocity_conversion_factor = 1.0
    
    def getPosition(self):
        """Get the position of the encoder."""
        return self._position * self._position_conversion_factor
    
    def getVelocity(self):
        """Get the velocity of the encoder."""
        return self._velocity * self._velocity_conversion_factor
    
    def setPosition(self, position):
        """Set the position of the encoder."""
        self._position = position / self._position_conversion_factor
    
    def setPositionConversionFactor(self, factor):
        """Set the position conversion factor."""
        self._position_conversion_factor = factor
    
    def setVelocityConversionFactor(self, factor):
        """Set the velocity conversion factor."""
        self._velocity_conversion_factor = factor


class SparkPIDController:
    """Simulation for a SparkMax PID controller."""
    
    def __init__(self, spark_max):
        """
        Initialize the simulated PID controller.
        
        Args:
            spark_max: The parent SparkMax controller
        """
        self.spark_max = spark_max
        self._p = 0.0
        self._i = 0.0
        self._d = 0.0
        self._ff = 0.0
        self._izone = 0.0
        self._output_min = -1.0
        self._output_max = 1.0
        self._setpoint = 0.0
        self._control_type = 0  # kDutyCycle
        self._feedback_device = None
    
    def setP(self, gain):
        """Set the proportional gain."""
        self._p = gain
    
    def setI(self, gain):
        """Set the integral gain."""
        self._i = gain
    
    def setD(self, gain):
        """Set the derivative gain."""
        self._d = gain
    
    def setFF(self, gain):
        """Set the feed forward gain."""
        self._ff = gain
    
    def setIZone(self, izone):
        """Set the IZone (integral zone)."""
        self._izone = izone
    
    def setOutputRange(self, min_output, max_output):
        """Set the output range."""
        self._output_min = min_output
        self._output_max = max_output
    
    def setFeedbackDevice(self, device):
        """Set the feedback device for the PID controller."""
        self._feedback_device = device
    
    def setReference(self, value, ctrl_type, pid_slot=0, arbFeedforward=0.0):
        """
        Set the reference value for the PID controller.
        
        Args:
            value: The reference value
            ctrl_type: The control type (kDutyCycle, kVelocity, etc.)
            pid_slot: The PID slot to use (ignored in simulation)
            arbFeedforward: Arbitrary feed forward (ignored in simulation)
        """
        self._setpoint = value
        self._control_type = ctrl_type
        
        # In simulation, we'll just set the motor directly
        # In a real implementation, we'd do real PID control
        self.spark_max.set(value)


class BaseSparkMax:
    """Base class for simulated SparkMax controllers."""
    
    # Enum for idle modes (matches REV API)
    class IdleMode:
        kCoast = 0
        kBrake = 1
    
    # Enum for motor types (matches REV API)
    class MotorType:
        kBrushless = 0
        kBrushed = 1
    
    # Enum for control types (matches REV API)
    class ControlType:
        kDutyCycle = 0
        kVelocity = 1
        kPosition = 2
        kVoltage = 3
        kCurrent = 4
        kSmartMotion = 5
        kSmartVelocity = 6
        kSmartVoltage = 7
    
    def __init__(self, device_id, motor_type):
        """
        Initialize the base simulated SparkMax.
        
        Args:
            device_id (int): The CAN ID of the controller
            motor_type (int): The type of motor (brushed or brushless)
        """
        self.device_id = device_id
        self.motor_type = motor_type
        
        # State variables
        self._applied_output = 0.0
        self._inverted = False
        self._idle_mode = self.IdleMode.kCoast
        self._voltage_comp_enabled = False
        self._voltage_comp_saturation = 12.0
        self._current_limit = 80
        self._position = 0.0
        self._velocity = 0.0
        self._current = 0.0
        self._temperature = 25.0  # 25°C default temperature
        
        # Create simulated hardware
        self._create_simulated_hardware()
        
        # Create NetworkTables entries for simulation
        self._setup_network_tables()
        
        # Create cached objects
        self._encoder = None
        self._absolute_encoder = None
        self._pid_controller = None
    
    def _create_simulated_hardware(self):
        """Create simulated hardware for the controller."""
        # Create a PWM channel for simulation if this is a valid PWM channel
        # We'll use PWM for simulation visualization
        try:
            if 0 <= self.device_id < 10:  # PWM channels are typically 0-9
                self._pwm = wpilib.PWM(self.device_id)
                self._sim_pwm = wpilib.simulation.PWMSim(self.device_id)
            else:
                self._pwm = None
                self._sim_pwm = None
        except Exception as e:
            print(f"Warning: Could not create PWM simulation for ID {self.device_id}: {e}")
            self._pwm = None
            self._sim_pwm = None
    
    def _setup_network_tables(self):
        """Set up NetworkTables entries for simulation."""
        try:
            self._nt = ntcore.NetworkTableInstance.getDefault()
            self._table = self._nt.getTable(f"Sim/SparkMax/{self.device_id}")
            
            # Create publishers
            self._output_pub = self._table.getDoubleTopic("output").publish()
            self._position_pub = self._table.getDoubleTopic("position").publish()
            self._velocity_pub = self._table.getDoubleTopic("velocity").publish()
            self._current_pub = self._table.getDoubleTopic("current").publish()
            self._temperature_pub = self._table.getDoubleTopic("temperature").publish()
            
            # Initialize values
            self._output_pub.set(0.0)
            self._position_pub.set(0.0)
            self._velocity_pub.set(0.0)
            self._current_pub.set(0.0)
            self._temperature_pub.set(25.0)
            
            # Create subscribers
            self._position_sub = self._table.getDoubleTopic("position").subscribe(0.0)
            
        except Exception as e:
            print(f"Warning: Could not set up NetworkTables for SparkMax {self.device_id}: {e}")
    
    def set(self, speed):
        """
        Set the motor output value.
        
        Args:
            speed (float): The speed value (-1.0 to 1.0)
        """
        # Apply inversion if needed
        if self._inverted:
            speed = -speed
        
        # Store the applied output
        self._applied_output = speed
        
        # Update PWM output for simulation visualization
        if self._pwm is not None:
            try:
                # PWM expects values from -1 to 1
                self._pwm.setSpeed(speed)
            except Exception as e:
                pass  # Ignore PWM errors in simulation
        
        # Update NetworkTables
        self._output_pub.set(speed)
        
        # Simulate motor physics
        self._simulate_motor_physics()
    
    def _simulate_motor_physics(self):
        """Simulate basic motor physics."""
        # Update position based on velocity
        self._position += self._velocity * 0.02  # Assuming 20ms update rate
        
        # Update velocity based on applied output (simplified)
        # In reality, this would be affected by load, battery voltage, etc.
        target_velocity = self._applied_output * 5600  # Assuming max RPM of 5600
        self._velocity += (target_velocity - self._velocity) * 0.1  # Simple low-pass filter
        
        # Simulate current draw based on applied output and velocity
        # High current when starting or stalling, lower at speed
        velocity_factor = abs(self._velocity / 5600)  # Normalized velocity (0-1)
        stall_current = 100  # Amps at stall
        free_current = 1.5  # Amps at free speed
        self._current = abs(self._applied_output) * (stall_current * (1 - velocity_factor) + free_current * velocity_factor)
        
        # Simulate temperature rise based on current
        ambient_temp = 25.0  # °C
        max_temp_rise = 80.0  # Max temperature rise above ambient
        temp_rise_factor = (self._current / 100) ** 2  # Current^2 affects heating
        target_temp = ambient_temp + (max_temp_rise * temp_rise_factor)
        self._temperature += (target_temp - self._temperature) * 0.001  # Very slow temperature change
        
        # Update NetworkTables
        self._position_pub.set(self._position)
        self._velocity_pub.set(self._velocity)
        self._current_pub.set(self._current)
        self._temperature_pub.set(self._temperature)
    
    def setInverted(self, inverted):
        """Set whether the motor output is inverted."""
        self._inverted = inverted
    
    def getInverted(self):
        """Get whether the motor output is inverted."""
        return self._inverted
    
    def setIdleMode(self, mode):
        """Set the idle mode (coast or brake)."""
        self._idle_mode = mode
    
    def getIdleMode(self):
        """Get the idle mode."""
        return self._idle_mode
    
    def enableVoltageCompensation(self, nominal_voltage):
        """Enable voltage compensation."""
        self._voltage_comp_enabled = True
        self._voltage_comp_saturation = nominal_voltage
    
    def disableVoltageCompensation(self):
        """Disable voltage compensation."""
        self._voltage_comp_enabled = False
    
    def setSmartCurrentLimit(self, limit):
        """Set the smart current limit."""
        self._current_limit = limit
    
    def getOutputCurrent(self):
        """Get the output current."""
        return self._current
    
    def getMotorTemperature(self):
        """Get the motor temperature."""
        return self._temperature
    
    def getEncoder(self, encoder_type=None, counts_per_rev=42):
        """
        Get the encoder attached to the motor.
        
        Args:
            encoder_type: The type of encoder
            counts_per_rev: The counts per revolution
            
        Returns:
            SparkRelativeEncoder: The encoder object
        """
        if self._encoder is None:
            self._encoder = SparkRelativeEncoder(self, encoder_type, counts_per_rev)
        return self._encoder
    
    def getAbsoluteEncoder(self, encoder_type=None):
        """
        Get the absolute encoder attached to the motor.
        
        Args:
            encoder_type: The type of encoder
            
        Returns:
            SparkMaxAbsoluteEncoder: The absolute encoder object
        """
        if self._absolute_encoder is None:
            self._absolute_encoder = SparkMaxAbsoluteEncoder(self, encoder_type)
        return self._absolute_encoder
    
    def getPIDController(self):
        """
        Get the PID controller for the motor.
        
        Returns:
            SparkPIDController: The PID controller
        """
        if self._pid_controller is None:
            self._pid_controller = SparkPIDController(self)
        return self._pid_controller
    
    def setVoltage(self, voltage):
        """
        Set the output voltage.
        
        Args:
            voltage (float): The output voltage
        """
        # Convert voltage to speed (-1 to 1)
        speed = voltage / 12.0
        self.set(speed)
    
    def stopMotor(self):
        """Stop the motor."""
        self.set(0)
    
    def getDeviceId(self):
        """Get the device ID."""
        return self.device_id
    
    def burnFlash(self):
        """Burn the configuration to flash (no-op in simulation)."""
        pass
    
    def restoreFactoryDefaults(self):
        """Restore factory defaults."""
        self._inverted = False
        self._idle_mode = self.IdleMode.kCoast
        self._voltage_comp_enabled = False
        self._voltage_comp_saturation = 12.0
        self._current_limit = 80


class SparkMax(BaseSparkMax):
    """Simulation for a SparkMax motor controller."""
    
    def __init__(self, device_id, motor_type=BaseSparkMax.MotorType.kBrushless):
        """
        Initialize the simulated SparkMax.
        
        Args:
            device_id (int): The CAN ID of the controller
            motor_type (int): The type of motor (brushed or brushless)
        """
        super().__init__(device_id, motor_type)
        print(f"Created simulated SparkMax with ID {device_id}")
    
    def follow(self, leader, invert=False):
        """
        Configure this controller to follow another.
        
        Args:
            leader: The controller to follow
            invert (bool): Whether to invert the follower output
        """
        # In simulation, we'll just remember who we're following
        self._leader = leader
        self._follow_inverted = invert


class CANSparkMax(SparkMax):
    """Alias for SparkMax to match the REV API."""
    pass


class SparkFlex(BaseSparkMax):
    """Simulation for a SparkFlex motor controller."""
    
    def __init__(self, device_id, motor_type=BaseSparkMax.MotorType.kBrushless):
        """
        Initialize the simulated SparkFlex.
        
        Args:
            device_id (int): The CAN ID of the controller
            motor_type (int): The type of motor (brushed or brushless)
        """
        super().__init__(device_id, motor_type)
        print(f"Created simulated SparkFlex with ID {device_id}")
    
    def follow(self, leader, invert=False):
        """
        Configure this controller to follow another.
        
        Args:
            leader: The controller to follow
            invert (bool): Whether to invert the follower output
        """
        # In simulation, we'll just remember who we're following
        self._leader = leader
        self._follow_inverted = invert