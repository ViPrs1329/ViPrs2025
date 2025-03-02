# test_imports.py
# Simple test script to verify module imports

import wpilib
import commands2
import rev
from rev import SparkLowLevel


def main():
    help(rev.SparkLowLevel.MotorType)
    print(dir(rev))
    print(dir(rev.SparkLowLevel))
    motor = rev.SparkLowLevel(1, rev.SparkLowLevel.MotorType.kBrushless)
    print(dir(motor))



if __name__ == "__main__":
    main()