class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, F=0.0):
        """Initialize PID constants.

        Args:
            P (float): Proportional gain. Applied to error.
            I (float): Integral gain. Applied to sum of errors.
            D (float): Derivative gain. Applied to the change in error.
            F (float): Feedforward gain. Applied to setpoint, often used to compensate for system's steady-state error.
        """
        self.P = P
        self.I = I
        self.D = D
        self.F = F