import numpy
from wpimath.geometry import Translation2d
from pathplannerlib.config import PIDConstants

# wpilib has us covered for simple conversions
# class convert:
#     def in2m(inches):
#         return 0.0254 * inches
    
#     def rev2rad(rev):
#         return rev * 2 * numpy.pi

#     def rad2rev(radians):
#         return radians / (2 * numpy.pi)
    
    

class CANIDs:
    # Swerve CAD IDs

    # this naming sucks
    # SwerveModule_Drive1 = 1
    # SwerveModule_Rotation1 = 2
    # SwerveModule_Drive2 = 3
    # SwerveModule_Rotation2 = 4
    # SwerveModule_Drive3 = 5
    # SwerveModule_Rotation3 = 6
    # SwerveModule_Drive4 = 7
    # SwerveModule_Rotation4 = 8

    # this is much better
    flDrive: int = 1
    flRotation: int = 2
    frDrive: int = 3
    frRotation: int = 4
    blDrive: int = 5
    blRotation: int = 6
    brDrive: int = 7
    brRotation: int = 8

    pigeon: int = 100

class Input:
    class Consts:
        inputScale: float = 0.8
        inputDeadZone: float = 0.1
        rampRate: float = 0.5

    class States:
        pass

class Drive:
    class Consts:
        flModuleOffset: Translation2d = Translation2d()
        frModuleOffset: Translation2d = Translation2d()
        blModuleOffset: Translation2d = Translation2d()
        brModuleOffset: Translation2d = Translation2d()

        translationConstants: PIDConstants = PIDConstants(0, 0, 0, 0)
        rotationConstants: PIDConstants = PIDConstants(0, 0, 0, 0)

        maxModuleSpeed = 1
    class States:
        pass

class Intake:
    class Consts:
        pass
    class States:
        pass

class Elevator:
    class Consts:
        pass
    class States:
        pass

class Arm:
    class Consts:
        pass
    class States:
        pass

class Sensor:
    class Consts:
        pass

    class States:
        pass