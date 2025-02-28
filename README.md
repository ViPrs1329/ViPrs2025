# Team 1329 Robot Code 2025

## Robot Overview
Our 2025 robot features a swerve drive base with an elevator and specialized end effector for manipulating game pieces (Coral and Algae).

### Hardware Configuration
- **Drivetrain**: Swerve drive with Rev NEO motors and Spark MAX controllers
  - CANcoder absolute encoders for wheel orientation
  - Pigeon 2.0 IMU for field-oriented control
- **Elevator**: Cascading two-stage design powered by NEO Vortex motors
  - 16:1 planetary gearboxes for mechanical advantage
- **End Effector**:
  - Coral Manipulator: Dual NEO motors with compliant wheels
    - LaserCAN sensors for precise positioning
  - Algae Manipulator: Rotatable intake system with NEO motors
    - Absolute encoder for position feedback

## Code Structure
- `robot.py`: Main robot class and program entry point
- `robotContainer.py`: Wires together subsystems, controllers, and commands
- `/subsystems/`: Core robot subsystem implementations
  - `SwerveDriveSubsystem.py`: Swerve drive implementation
  - `ElevatorSubsystem.py`: Elevator control system
  - `EndEffector.py`: Combined end effector for game piece manipulation
- `/commands/`: Command-based operations
  - `IntakeCommands.py`: Coral intake/ejection
  - `ElevatorCommands.py`: Elevator positioning
  - `AlgaeCommands.py`: Algae collection
  - `ScoringCommands.py`: Scoring sequences
  - `AutonomousCommands.py`: Autonomous routines
- `/team254/`: Performance optimized motor control
  - `LazySparkMax.py`: Reduces CAN bus traffic
  - `SparkMaxFactory.py`: Consistent motor configuration
- `/constants.py`: Central repository for robot constants
- `/sim/`: Simulation support for testing without hardware

## Key Features
- Command-based programming paradigm
- Field-oriented swerve drive control
- Sensor call caching to reduce CAN bus overhead
- Mode-based operator control scheme
- Comprehensive simulation support
- Autonomous path following and scoring

## Control Scheme
- **Driver Controller**: Swerve drive, field orientation toggle, emergency stop
- **Operator Controller**: Mode selection, game piece manipulation
  - Base Mode: Home positions
  - Coral Mode: Coral intake/scoring
  - Algae Mode: Algae collection positions

## Build & Deploy
```bash
python -m robotpy deploy --skip-tests
python -m robotpy sync
```

## Testing
python -m robotpy sim