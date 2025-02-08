
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
    # Swerve CAN IDs
    SwerveModuleDrive1 = 1
    SwerveModuleRotation1 = 2
    SwerveModuleDrive2 = 3
    SwerveModuleRotation2 = 4
    SwerveModuleDrive3 = 5
    SwerveModuleRotation3 = 6
    SwerveModuleDrive4 = 7
    SwerveModuleRotation4 = 8

    # Encoders

    EncoderModuleRotation1 = 10
    EncoderModuleRotation2 = 11
    EncoderModuleRotation3 = 9
    EncoderModuleRotation4 = 12

    # End Effector CAN IDs
    arm = 100
    intake1 = 101
    intake2 = 102
    algae = 103

    # Elevator CAN IDs
    elevator1 = 100
    elevator2 = 101

class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class driveConsts:
    wheelDiameter = 4
    slowScale = 0.3

class endEffectorConsts:
    pass

class elevatorConsts:
    pass



class sensorConsts:
    pass