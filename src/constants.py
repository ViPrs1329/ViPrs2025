
import numpy

class controller:
    scale = 0.5
    tscale = 1
    XYdeadzone = 0.03
    Tdeadzone = 0.1

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * numpy.pi

    def rad2rev(radians):
        return radians / (2 * numpy.pi)
    
    

class CANIDs:
    # Swerve CAD IDs
    SwerveModuleDrive1 = 11
    SwerveModuleRotation1 = 10
    SwerveModuleDrive2 = 13
    SwerveModuleRotation2 = 12
    SwerveModuleDrive3 = 5
    SwerveModuleRotation3 = 4
    SwerveModuleDrive4 = 9
    SwerveModuleRotation4 = 8

    # Encoders

    EncoderModuleRotation1 = 20
    EncoderModuleRotation2 = 17
    EncoderModuleRotation3 = 19
    EncoderModuleRotation4 = 18

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