# src/team254/SparkFactory.py
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

class LazySparkMax(rev.CANSparkMax, LazySparkBase):
    """
    Lazy version of CANSparkMax that reduces CAN bus traffic by skipping duplicate set commands.
    """
    def __init__(self, device_number: int, motor_type=rev.CANSparkMax.MotorType.kBrushless):
        rev.CANSparkMax.__init__(self, device_number, motor_type)
        LazySparkBase.__init__(self)

    def follow(self, leader: rev.CANSparkMax, invert=False):
        self.m_leader = leader
        return super().follow(leader, invert)

    def set(self, setpoint: float):
        """
        Sets the motor output only if the value has changed.
        """
        if setpoint != self.m_last_set:
            self.m_last_set = setpoint
            super().set(setpoint)

    def setReference(self, value: float, ctrl_type=rev.CANSparkMax.ControlType.kDutyCycle, 
                     pidSlot=0, arbFeedforward=0.0):
        """
        Sets the reference point for PID control only if the value or control type has changed.
        """
        if value != self.m_last_set or ctrl_type != self.m_last_control_type:
            self.m_last_set = value
            self.m_last_control_type = ctrl_type
            
            # Use the cached PID controller instance
            if self._pid_controller is None:
                self._pid_controller = super().getPIDController()
                
            self._pid_controller.setReference(value, ctrl_type, pidSlot, arbFeedforward)

    def getPIDController(self):
        """
        Get the PID controller, using a cached instance if available.
        """
        if self._pid_controller is None:
            self._pid_controller = super().getPIDController()
        return self._pid_controller

class LazySparkFlex(rev.SparkFlex, LazySparkBase):
    """
    Lazy version of SparkFlex that reduces CAN bus traffic by skipping duplicate set commands.
    """
    def __init__(self, device_number: int, motor_type=rev.SparkFlex.MotorType.kBrushless):
        rev.SparkFlex.__init__(self, device_number, motor_type)
        LazySparkBase.__init__(self)

    def follow(self, leader: rev.SparkFlex, invert=False):
        self.m_leader = leader
        return super().follow(leader, invert)

    def set(self, setpoint: float):
        """
        Sets the motor output only if the value has changed.
        """
        if setpoint != self.m_last_set:
            self.m_last_set = setpoint
            super().set(setpoint)

    def setReference(self, value: float, ctrl_type=rev.SparkFlex.ControlType.kDutyCycle, 
                     pidSlot=0, arbFeedforward=0.0):
        """
        Sets the reference point for PID control only if the value or control type has changed.
        """
        if value != self.m_last_set or ctrl_type != self.m_last_control_type:
            self.m_last_set = value
            self.m_last_control_type = ctrl_type
            
            # Use the cached PID controller instance
            if self._pid_controller is None:
                self._pid_controller = super().getPIDController()
                
            self._pid_controller.setReference(value, ctrl_type, pidSlot, arbFeedforward)

    def getPIDController(self):
        """
        Get the PID controller, using a cached instance if available.
        """
        if self._pid_controller is None:
            self._pid_controller = super().getPIDController()
        return self._pid_controller