import numpy
from wpimath.geometry import Translation2d
from pathplannerlib.config import PIDConstants
from armUtils import ArmAngle

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

    # Elevator CAN IDs

    leftElevatorMotor: int = 200
    rightElevatorMotor: int = 201

    # Arm CAN IDs

    leftArmMotor: int = 300
    rightArmMotor: int = 301

    # Intake CAN IDs

    intakeMotor: int = 400

class Input:

    class Consts:
        pass

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

        driveGearRatio: float = 6.75
        rotGearRatio: float = 12.8

        rampRate: float = 0.5

        rotP: int = 1
        rotI: int = 0
        rotD: int = 0

        driveP: int = 1
        driveI: int = 0
        driveD: int = 0

        maxSpeed = 0.5
        maxAngularSpeed = 0.5
        inputDeadzone = 0.1

        coralScoringTime: float = 0.5
        algaeScoringTime: float = 0.5

        scoringDriveSpeed: float = 0.2

    class States:

        autonomous: int = 0
        teleop: int = 1

class Elevator:

    class Consts:

        currentLimit: int = 30

        p: int = 1
        i: int = 0
        d: int = 0
        ff: int = 0

        # these are in revs not meters
        groundIntakeAlgae: float = 0
        l2IntakeAlgae: float = 1
        l3IntakeAlgae: float = 2

        groundIntakeCoral: float = 3
        feederIntakeCoral: float = 4

        scoreAlgaeNet: float = 5
        scoreAlgaeProcessor: float = 6

        scoreCoralL1: float = 7
        scoreCoralL2: float = 8
        scoreCoralL3: float = 9
        scoreCoralL4: float = 10

        default: int = 11

        scoringCoralL1: int = 12
        scoringCoralL2: int = 13
        scoringCoralL3: int = 14
        scoringCoralL4: int = 15

        # tollerance for the elevator
        tollerance: float = 0.1

        coralScoringTime: float = Drive.Consts.coralScoringTime
        algaeScoringTime: float = Drive.Consts.algaeScoringTime

    class States:

        groundIntakeAlgae: int = 0
        l2IntakeAlgae: int = 1
        l3IntakeAlgae: int = 2

        groundIntakeCoral: int = 3
        feederIntakeCoral: int = 4

        scoreAlgaeNet: int = 5
        scoreAlgaeProcessor: int = 6

        scoreCoralL1: int = 7
        scoreCoralL2: int = 8
        scoreCoralL3: int = 9
        scoreCoralL4: int = 10

        default: int = 11

        scoringCoralL1: int = 12
        scoringCoralL2: int = 13
        scoringCoralL3: int = 14
        scoringCoralL4: int = 15

class Intake:

    class Consts:

        armCurrentLimit: int = 30
        armP: int = 1
        armI: int = 0
        armD: int = 0
        armFF: int = 0

        intakeCurrentLimit: int = 30

        intakeSpeed = 0.5

        # these are in revs not meters
        groundIntakeAlgae: ArmAngle = ArmAngle(0, 0)
        l2IntakeAlgae: ArmAngle = ArmAngle(0, 0)
        l3IntakeAlgae: ArmAngle = ArmAngle(0, 0)

        groundIntakeCoral: ArmAngle = ArmAngle(0, 0)
        feederIntakeCoral: ArmAngle = ArmAngle(0, 0)

        scoreAlgaeNet: ArmAngle = ArmAngle(0, 0)
        scoreAlgaeProcessor: ArmAngle = ArmAngle(0, 0)

        scoreCoralL1: ArmAngle = ArmAngle(0, 0)
        scoreCoralL2: ArmAngle = ArmAngle(0, 0)
        scoreCoralL3: ArmAngle = ArmAngle(0, 0)
        scoreCoralL4: ArmAngle = ArmAngle(0, 0)

        default: ArmAngle = ArmAngle(0, 0)

        scoringCoralL1: ArmAngle = ArmAngle(0, 0)
        scoringCoralL2: ArmAngle = ArmAngle(0, 0)
        scoringCoralL3: ArmAngle = ArmAngle(0, 0)
        scoringCoralL4: ArmAngle = ArmAngle(0, 0)

        coralScoringTime: float = Elevator.Consts.coralScoringTime
        algaeScoringTime: float = Elevator.Consts.algaeScoringTime

    class States:

        groundIntakeAlgae: int = 0
        l2IntakeAlgae: int = 1
        l3IntakeAlgae: int = 2

        groundIntakeCoral: int = 3
        feederIntakeCoral: int = 4

        scoreAlgaeNet: int = 5
        scoreAlgaeProcessor: int = 6

        scoreCoralL1: int = 7
        scoreCoralL2: int = 8
        scoreCoralL3: int = 9
        scoreCoralL4: int = 10

        default: int = 11

        scoringCoralL1: int = 12
        scoringCoralL2: int = 13
        scoringCoralL3: int = 14
        scoringCoralL4: int = 15

class Sensor:

    class Consts:
        pass

    class States:
        pass

class Limelight:

    class Consts:
        tableNames: list[str] = ["limelight-lside", "limelight-rside"]

    class States:
        pass