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
    # Swerve CAN IDs

    flDrive: int = 1
    flRotation: int = 2
    flEncoder: int = 3

    frDrive: int = 4
    frRotation: int = 5
    frEncoder: int = 6

    blDrive: int = 7
    blRotation: int = 8
    blEncoder: int = 9

    brDrive: int = 10
    brRotation: int = 11
    brEncoder: int = 12

    pigeon: int = 100

class Input:
    class Consts:
        inputScale: float = 0.8
        inputDeadZone: float = 0.1

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

        maxModuleSpeed: float = 1

        driveCurrentLimit: int = 30
        rotCurrentLimit: int = 30

        rampRate: float = 0.5

        rotP: int = 1
        rotI: int = 0
        rotD: int = 0
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