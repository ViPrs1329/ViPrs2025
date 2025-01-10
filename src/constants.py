
import numpy

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * numpy.pi

    def rad2rev(radians):
        return radians / (2 * numpy.pi)
    
    

class CANIDs:
    # Swerve CAD IDs
    SwerveModuleDrive1 = 1
    SwerveModuleRotation1 = 2
    SwerveModuleDrive2 = 3
    SwerveModuleRotation2 = 4
    SwerveModuleDrive3 = 5
    SwerveModuleRotation3 = 6
    SwerveModuleDrive4 = 7
    SwerveModuleRotation4 = 8

    # Encoders

    EncoderModuleRotation1 = 100
    EncoderModuleRotation2 = 100
    EncoderModuleRotation3 = 100
    EncoderModuleRotation4 = 100

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