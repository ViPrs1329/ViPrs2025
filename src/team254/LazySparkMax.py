# src/team254/LazySparkMax.py
import rev

class LazySparkBase:
    """
    Base class for lazy REV motor controllers that reduces CAN bus traffic
    by skipping duplicate commands.
    """
    def __init__(self):
        self.m_last_set = float('nan')
        self.m_last_control_type = None
        self.m_leader = None
        self._pid_controller = None

    @property
    def leader(self):
        return self.m_leader

class LazySparkMax(rev.SparkMax, LazySparkBase):
    """
    Lazy version of SparkMax that reduces CAN bus traffic by skipping duplicate set commands.
    """
    
    def __init__(self, device_number: int, motor_type=rev.SparkMax.MotorType.kBrushless):
        rev.SparkMax.__init__(self, device_number, motor_type)
        LazySparkBase.__init__(self)

    def follow(self, leader: rev.SparkMax, invert=False):
        self.m_leader = leader
        return super().follow(leader, invert)

    def set(self, setpoint: float):
        """
        Sets the motor output only if the value has changed.
        """
        if setpoint != self.m_last_set:
            self.m_last_set = setpoint
            super().set(setpoint)
            
    def setIdleMode(self, mode):
        """Set the idle mode of the motor controller"""
        # In real hardware, this would call the parent method
        # For simulation compatibility, ensure this method exists
        if hasattr(super(), "setIdleMode"):
            # TODO: setIdleMode() isn't a part of the parent class. 
            super().setIdleMode(mode)
            
    def setInverted(self, inverted):
        """Set whether the motor is inverted"""
        if hasattr(super(), "setInverted"):
            super().setInverted(inverted)
            
    def enableVoltageCompensation(self, voltage):
        """Enable voltage compensation"""
        if hasattr(super(), "enableVoltageCompensation"):
            # TODO: Check enableVoltageCompensation() does it exist?
            super().enableVoltageCompensation(voltage)
            
    def disableVoltageCompensation(self):
        """Disable voltage compensation"""
        if hasattr(super(), "disableVoltageCompensation"):
            # TODO: It seems that disableVoltageCompensation() isn't part of the parent class
            super().disableVoltageCompensation()
            
    def setSmartCurrentLimit(self, limit):
        """Set the current limit"""
        if hasattr(super(), "setSmartCurrentLimit"):
            # TODO: setSmartCurrentLimit() isn't part of the parent class
            super().setSmartCurrentLimit(limit)
            
    def burnFlash(self):
        """Burn the configuration to flash memory"""
        if hasattr(super(), "burnFlash"):
            # TODO: burnFlash() isn't part of the parent class
            super().burnFlash()
            
    def getAbsoluteEncoder(self, encoder_type=None):
        """
        Get an absolute encoder object if supported, otherwise create a simulation version.
        """
        if hasattr(super(), "getAbsoluteEncoder"):
            # TODO: getAbsoluteEncoder() isn't part of the parent class
            return super().getAbsoluteEncoder(encoder_type)
        else:
            # Create a simulated absolute encoder
            return SimSparkMaxAbsoluteEncoder(self)

    def getPIDController(self):
        """
        Get the PID controller, using a cached instance if available.
        """
        if self._pid_controller is None:
            # TODO: getPIDController() isn't part of the parent class
            self._pid_controller = super().getPIDController()
        return self._pid_controller
    
    def getVelocity(self):
        """Get the velocity of the motor."""
        if hasattr(self, "_encoder") and self._encoder is not None:
            return self._encoder.getVelocity()
        return 0.0  # Default value for simulation

class LazySparkFlex(rev.SparkFlex, LazySparkBase):
    """
    Lazy version of SparkFlex that reduces CAN bus traffic by skipping duplicate set commands.
    """
    def __init__(self, device_number: int, motor_type=rev.SparkFlex.MotorType.kBrushless):
        rev.SparkFlex.__init__(self, device_number, motor_type)
        LazySparkBase.__init__(self)

    def follow(self, leader, invert=False):
        self.m_leader = leader
        # TODO: follow() doesn't appear to be a part of the parent class
        return super().follow(leader, invert)

    def set(self, setpoint: float):
        """
        Sets the motor output only if the value has changed.
        """
        if setpoint != self.m_last_set:
            self.m_last_set = setpoint
            # TODO: set() doesn't appear to be a part of the 
            super().set(setpoint)
            
    def setIdleMode(self, mode):
        """Set the idle mode of the motor controller"""
        if hasattr(super(), "setIdleMode"):
            # TODO: setIdleMode() doesn't appear to be a part of the parent class
            super().setIdleMode(mode)
            
    def setInverted(self, inverted):
        """Set whether the motor is inverted"""
        if hasattr(super(), "setInverted"):
            # TODO: setInverted() doesn't appear to be a part of the parent class
            super().setInverted(inverted)
            
    def enableVoltageCompensation(self, voltage):
        """Enable voltage compensation"""
        if hasattr(super(), "enableVoltageCompensation"):
            # TODO: enableVoltageCompensation() doesn't appear to be a part of the parent class. 
            super().enableVoltageCompensation(voltage)
            
    def disableVoltageCompensation(self):
        """Disable voltage compensation"""
        if hasattr(super(), "disableVoltageCompensation"):
            # TODO: disableVoltageCompensation() ditto
            super().disableVoltageCompensation()
            
    def setSmartCurrentLimit(self, limit):
        """Set the current limit"""
        if hasattr(super(), "setSmartCurrentLimit"):
            # TODO: setSmartCurrentLimit() ditto
            super().setSmartCurrentLimit(limit)
            
    def burnFlash(self):
        """Burn the configuration to flash memory"""
        if hasattr(super(), "burnFlash"):
            # TODO: burnFlass() ditto
            super().burnFlash()
            
    def getAbsoluteEncoder(self, encoder_type=None):
        """
        Get an absolute encoder object if supported, otherwise create a simulation version.
        """
        if hasattr(super(), "getAbsoluteEncoder"):
            # TOEDO: getAbsoluteEncoder() ditto
            
            return super().getAbsoluteEncoder(encoder_type)
        else:
            # Create a simulated absolute encoder
            return SimSparkMaxAbsoluteEncoder(self)

    def getPIDController(self):
        """
        Get the PID controller, using a cached instance if available.
        """
        if self._pid_controller is None:
            self._pid_controller = super().getPIDController()
        return self._pid_controller

# Add a simulated absolute encoder for use in simulation
class SimSparkMaxAbsoluteEncoder:
    """Simulation of a SparkMax absolute encoder."""
    
    def __init__(self, spark_max):
        self.spark_max = spark_max
        self._position = 0.0
        self._velocity = 0.0
        self._position_conversion_factor = 1.0
        self._velocity_conversion_factor = 1.0
        self._zero_offset = 0.0
        self._inverted = False
        
    def getPosition(self):
        """Get the position of the encoder."""
        # Apply conversion factor and zero offset in simulation
        position = (self._position * self._position_conversion_factor)
        if self._inverted:
            position = -position
        return position + self._zero_offset
        
    def getVelocity(self):
        """Get the velocity of the encoder."""
        velocity = (self._velocity * self._velocity_conversion_factor)
        return -velocity if self._inverted else velocity
        
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
        
    # Simulate a changing position in simulation
    def simulate_position(self, position):
        """Set a simulated position for testing."""
        self._position = position