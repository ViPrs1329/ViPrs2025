from rev import CANSparkMax

"""
This class is a thin wrapper around the CanSparkMax that reduces CAN bus / CPU
overhead by skipping duplicate set commands.
"""

class LazySparkMax(CANSparkMax):
    def __init__(self, device_number: int):
        super().__init__(device_number, CANSparkMax.MotorType.kBrushless)
        self.m_last_set = float('nan')
        self.m_last_control_type = None
        self.m_leader = None
        self._pid_controller = None  # Cache the PID controller instance

    @property
    def leader(self):
        return self.m_leader

    def follow(self, leader: CANSparkMax):
        self.m_leader = leader
        return super().follow(leader)

    def set(self, setpoint: float, type=CANSparkMax.ControlType.kDutyCycle, pidSlot=0, arbitraryFF=0):
        # print(f"LazySpark.set({self}, {setpoint}, {type})")
        if setpoint != self.m_last_set or type != self.m_last_control_type:
            self.m_last_set = setpoint
            self.m_last_control_type = type
            # Use the cached PID controller instance
            if self._pid_controller is None:
                self._pid_controller = super().getPIDController()
            self._pid_controller.setReference(value=setpoint, ctrl=type, pidSlot=pidSlot, arbFeedforward=arbitraryFF)