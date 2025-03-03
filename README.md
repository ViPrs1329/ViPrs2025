# Team 1329 FRC Robot Code (2025 Season)

## Overview

This repository contains the robot code for Team 1329's 2025 season robot. The code is structured using WPILib's command-based framework and follows a standardized approach to subsystem management for improved maintainability and reliability.

## Robot Features

- **Swerve Drive**: Four-module swerve drive with field-oriented control
- **Elevator**: Two-stage cascading elevator with precise position control
- **End Effector**: Dual-purpose end effector for manipulating Coral and Algae game pieces

## Code Structure

### Core Architecture

- `robot.py`: Main robot class that manages robot lifecycle and modes
- `robotContainer.py`: Container for subsystems, commands, and controllers
- `constants.py`: Centralized robot constants and configuration
- `/subsystems/`: Robot subsystem implementations
- `/commands/`: Command implementations for robot actions
- `/sim/`: Simulation support for testing without hardware

### Standardized Subsystem Pattern

All subsystems follow a standardized pattern that includes:

- Error handling
- Sensor caching to reduce CAN traffic
- Simulation support
- Status reporting

### Primary Subsystems

- `SwerveDriveSubsystem.py`: Swerve drive implementation
- `ElevatorSubsystem.py`: Elevator control
- `EndEffectorSubsystem.py`: Coral and Algae game piece manipulation

## Hardware Configuration

- **Drive Motors**: NEO brushless motors with SparkMax controllers
- **Rotation Motors**: NEO brushless motors with SparkMax controllers
- **Rotation Encoders**: CANcoders for absolute position feedback
- **IMU**: Pigeon 2.0 for robot orientation
- **Elevator Motors**: NEO Vortex motors with SparkFlex controllers
- **End Effector Motors**: NEO motors for Coral and Algae manipulation
- **Distance Sensors**: LaserCAN for game piece detection and positioning

## Development Setup

### Prerequisites

- Python 3.9+
- RobotPy 2025.3.1.1+
- Visual Studio Code with Python and WPILib extensions (recommended)

### Installation

1. Clone this repository:
   ```
   git clone https://github.com/team1329/2025-robot-code.git
   ```

2. Install RobotPy and dependencies:
   ```
   python -m pip install robotpy[all]
   ```

3. Install robot-specific dependencies:
   ```
   cd 2025-robot-code
   pip install -r requirements.txt
   ```

### Building and Deploying

Deploy to the robot:
```
python -m robotpy deploy
```

Run in simulation:
```
python -m robotpy sim
```

## Testing

To run tests:
```
python -m robotpy test
```

## Simulation

The code includes comprehensive simulation support for testing without hardware. To run the robot in simulation:

```
python -m robotpy sim
```

Simulation provides:
- Simulated motor controllers and encoders
- Simulated sensors
- NetworkTables for simulated values
- Visualization of robot state

## Contributing

Please follow these guidelines when contributing to the codebase:

1. Follow the standardized subsystem pattern
2. Use consistent error handling
3. Add sensor caching where appropriate
4. Include simulation support for new features
5. Add proper documentation and comments

## License

This project is licensed under the MIT License - see the LICENSE file for details.