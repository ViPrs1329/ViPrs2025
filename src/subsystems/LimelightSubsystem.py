from commands2 import Subsystem
from ntcore import NetworkTableInstance
from ntcore import NetworkTable

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Transform2d

from constants import Limelight

class LimelightSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        
        self.tables: list[NetworkTable] = []
        for name in Limelight.Consts.tableNames:
            table: NetworkTable = NetworkTableInstance.getDefault().getTable(name)
            self.tables.append(table)

    def canSeeTarget(self) -> bool:
        """
        Check if the limelight can see a target.
        """
        return any(table.getNumber("tv", 0) == 1 for table in self.tables)
    
    def getRobotPositionFieldRelative(self) -> Pose2d:
        """
        Get the robot's position relative to the field.
        """
        if not self.canSeeTarget():
            return Pose2d(0, 0, Rotation2d(0))
        
        robotPose: Pose2d = Pose2d(0, 0, Rotation2d(0))

        count: int = 0
        # Iterate through all tables to accumulate the robot's position
        for table in self.tables:
            
            # botpose_orb uses MT2 compared to botpose which uses MT1
            botPose = table.getNumberArray("botpose_orb", [0, 0, 0, 0, 0, 0]) 
            if table.getNumber("tv", 0) == 1:
                count += 1
                x = botPose[0]
                y = botPose[1]
                rotation = Rotation2d.fromDegrees(botPose[5])
                robotPose += Transform2d(x, y, rotation) # yes, this is a Transform2d, not a Pose2d
        
        robotPose /= count if count > 0 else 1  # Average the position if multiple positions are used
        return robotPose
    
    def getRobotPositionRelativeToTarget(self, target: int) -> Pose2d:
        """
        Get the robot's position relative to the specified targets.
        
        :param targets: List of target indices to consider.
        :return: Pose2d representing the robot's position relative to the targets.
        """
        if not self.canSeeTarget():
            return Pose2d(0, 0, Rotation2d(0))
        
        robotPose: Pose2d = Pose2d(0, 0, Rotation2d(0))

        count: int = 0

        # Iterate through all tables to accumulate the robot's position
        for table in self.tables:
            if table.getNumber("tv", 0) == 1 and table.getNumber("tid", -1) == target:
                count += 1
                botPose = table.getNumberArray("botpose_targetspace", [0, 0, 0, 0, 0, 0])
                x = botPose[0]
                z = botPose[2]
                rotation = Rotation2d.fromDegrees(botPose[5])
                robotPose += Transform2d(x, z, rotation)

        robotPose /= count if count > 0 else 1  # Average the position if multiple positions are used
        return robotPose