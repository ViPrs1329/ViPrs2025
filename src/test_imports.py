# test_imports.py
# Simple test script to verify module imports

import wpilib
import commands2
from subsystems.SwerveDriveSubsystem import DriveTrain
from subsystems.EndEffector import EndEffector
from commands.IntakeCommands import IntakeCoralCommand

def main():
    print("Import test successful!")
    print("All required modules imported correctly.")

if __name__ == "__main__":
    main()