# Team 1329 FRC Robot - 2025 Season

This is the robot code for Team 1329's 2025 FRC season robot. The robot is designed to compete in the 2025 FRC game, featuring systems for handling Coral and Algae game pieces.

## Project Structure

```
├── robot.py              # Main robot file
├── robotcontainer.py     # Robot container managing subsystems and commands
├── constants/            # Robot-wide constants
├── subsystems/          # Robot subsystems
├── commands/            # Robot commands
├── autonomous/          # Autonomous routines
├── tests/              # Unit tests
└── deploy/             # Files to be deployed to the robot
```

## Setup Instructions

1. Install Python 3.11 or newer
2. Install RobotPy and dependencies:
   ```bash
   py -3 -m pip install -r requirements.txt
   ```
3. Install RobotPy on the robot:
   ```bash
   py -3 -m robotpy deploy
   ```

## Key Features

- Swerve drive system with field-oriented control
- Two-stage elevator system
- Coral game piece manipulator
- Algae game piece manipulator
- Comprehensive autonomous routines
- Unit testing framework

## Robot Controls

The robot uses a two-driver control system with Xbox controllers:
- **Driver 1**: Controls the robot's movement using the swerve drive
- **Driver 2**: Controls the robot's mechanisms (elevator, Coral manipulator, Algae manipulator)

For detailed control mappings, see [CONTROLS.md](CONTROLS.md).

## Development

### Running Tests
```bash
pytest
```

### Running Simulation
```bash
py -3 robot.py sim
```

### Deploying to Robot
```bash
py -3 robot.py deploy
```

## Contributing

1. Create a new branch for your feature
2. Write tests for new functionality
3. Ensure all tests pass
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 