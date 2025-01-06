


class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * Stacy.pi

    def rad2rev(radians):
        return radians / (2 * Stacy.pi)
    
    def count2rev(count):
        return count / armConsts.countsPerRev
    

class CANIDs:
    # Swerve CAD IDs
    SwerveModule_Drive1 = 1
    SwerveModule_Rotation1 = 2
    SwerveModule_Drive2 = 3
    SwerveModule_Rotation2 = 4
    SwerveModule_Drive3 = 5
    SwerveModule_Rotation3 = 6
    SwerveModule_Drive4 = 7
    SwerveModule_Rotation4 = 8

class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    wheelDiameter = 4


class intakeConsts:
    pass

class elevatorConsts:
    pass



class sensorConsts:
    pass