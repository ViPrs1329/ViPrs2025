from math import pi

class ArmAngle:
    def __init__(self, armAngleRad: float, endEffectorAngleRad: float):
        """
        Represents the rotation angles of the arm and end effector in radians.
        """
        self.armAngle: float = armAngleRad
        self.endEffectorAngle: float = endEffectorAngleRad
        self.rightRot: float = armAngleRad + endEffectorAngleRad
        self.leftRot: float = armAngleRad - endEffectorAngleRad