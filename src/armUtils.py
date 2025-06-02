from math import pi

class ArmAngle:
    def __init__(self, armAngleRad: float, endEffectorAngleRad: float) -> None:
        """
        Represents the rotation angles of the arm and end effector in radians.
        For a Hero's differential:
        - Left motor = armAngle + endEffectorAngle
        - Right motor = armAngle - endEffectorAngle
        Where:
        - armAngle is the elevation angle from horizontal
        - endEffectorAngle is the rotation of the end effector
        """
        self.armAngle: float = armAngleRad
        self.endEffectorAngle: float = endEffectorAngleRad
        
        # Calculate the left and right rotations based on the arm and end effector angles
        # These are the angles for the left and right motors
        self.leftRot: float = armAngleRad + endEffectorAngleRad
        self.rightRot: float = armAngleRad - endEffectorAngleRad
    
    @staticmethod
    def fromMotorAngles(leftAngle: float, rightAngle: float) -> 'ArmAngle':
        """
        Create an ArmAngle from left and right motor angles.
        
        Args:
            leftAngle: Left motor angle in radians
            rightAngle: Right motor angle in radians
            
        Returns:
            ArmAngle: New instance with calculated arm and end effector angles
        """
        armAngle = (leftAngle + rightAngle) / 2.0
        endEffectorAngle = (leftAngle - rightAngle) / 2.0
        return ArmAngle(armAngle, endEffectorAngle)
    
    def __str__(self) -> str:
        """String representation for debugging"""
        return (f"ArmAngle(arm={self.armAngle:.2f}rad, "
                f"endEffector={self.endEffectorAngle:.2f}rad)")
    
    def isWithinLimits(self, minArm: float, maxArm: float, 
                       minEnd: float, maxEnd: float) -> bool:
        """
        Check if angles are within safe operating limits.
        
        Args:
            minArm: Minimum safe arm angle (radians)
            maxArm: Maximum safe arm angle (radians)
            minEnd: Minimum safe end effector angle (radians)
            maxEnd: Maximum safe end effector angle (radians)
            
        Returns:
            bool: True if within limits
        """
        return (minArm <= self.armAngle <= maxArm and 
                minEnd <= self.endEffectorAngle <= maxEnd)
    
    def adjustAngles(self, armDelta: float = 0.0, endEffectorDelta: float = 0.0) -> None:
        """
        Adjust arm and end effector angles in place.
        
        Args:
            armDelta: Change in arm angle (radians)
            endEffectorDelta: Change in end effector angle (radians)
        """
        self.armAngle += armDelta
        self.endEffectorAngle += endEffectorDelta
        # Update motor angles after changing arm/end effector angles
        self.leftRot = self.armAngle + self.endEffectorAngle
        self.rightRot = self.armAngle - self.endEffectorAngle
    
    def withArmAngle(self, newArmAngle: float) -> 'ArmAngle':
        """Create new ArmAngle with updated arm angle."""
        return ArmAngle(newArmAngle, self.endEffectorAngle)
    
    def withEndEffectorAngle(self, newEndAngle: float) -> 'ArmAngle':
        """Create new ArmAngle with updated end effector angle."""
        return ArmAngle(self.armAngle, newEndAngle)