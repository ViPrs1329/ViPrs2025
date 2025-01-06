# DriveSubsystem.py
#
# 

import wpilib
import wpilib.drive
import commands2
import rev
import math
import constants

from team254.SparkMaxFactory import SparkMaxFactory
from team254.LazySparkMax import LazySparkMax


class DriveSubsystem(commands2.Subsystem):
    class Cache:
        def __init__(self):
            pass

    def __init__(self):
        super().__init__()

        self.cache = self.Cache()

        # Initialize swerve modules and other things...


    def updateHardware(self):
        # This method gets called periodically to update hardware state
        pass

    def cacheSensors(self):
        # This is called periodically to cache sensor data
        # so that we don't clog up the CAN network
        pass