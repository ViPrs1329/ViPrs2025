# Team 1329 Robot Code 2025 🤖

Welcome to Team 1329's robot code for the 2025 FRC season! This repository contains all the code that powers our competition robot.

## Documentation 📚

We've organized our documentation to help you understand and contribute to our robot code:

### Getting Started
- [Robot Overview](ROBOT_OVERVIEW.md) - High-level explanation of how our robot works
- [Contributing Guide](CONTRIBUTING.md) - How to contribute to the codebase
- [Architecture Guide](ARCHITECTURE.md) - Understanding our code organization

### Technical Details
- [Controls Guide](CONTROLS.md) - Detailed explanation of robot controls
- [Sensors Guide](SENSORS.md) - Information about our robot's sensors
- [Constants Reference](CONSTANTS.md) - Important robot configuration values

## Quick Start 🚀

1. **Set Up Environment**
```bash
# Clone repository
git clone https://github.com/your-team/robot-code-2025.git
cd robot-code-2025

# Create and activate virtual environment
python -m venv venv
.\venv\Scripts\activate  # Windows
source venv/bin/activate # Linux/Mac

# Install dependencies
pip install -r requirements.txt
```

2. **Run Tests**
```bash
python -m pytest
```

3. **Start Simulator**
```bash
python robot.py sim
```

## Project Structure 📁

```
robot-code-2025/
├── commands/          # Robot commands
├── subsystems/       # Robot subsystems
├── constants/        # Configuration values
├── autonomous/       # Autonomous routines
├── tests/           # Unit tests
└── docs/            # Additional documentation
```

## Features ✨

- Swerve drive system
  - Advanced field-relative control with Pigeon 2.0 IMU
  - Optimized sensor caching
  - Synchronized module updates
  - Precise orientation tracking
  - Tilt detection and monitoring
- Advanced elevator control
  - Precise position management
  - Efficient motor control
  - Smart motion profiling
- Dual manipulator system
  - Coral game piece handler
  - Algae game piece handler
- Vision processing
- Autonomous routines
- Safety systems
- Performance Optimization
  - Smart sensor caching
  - Reduced CAN bus traffic
  - Consistent 20ms loop timing
  - Efficient hardware updates

## Contributing 🤝

We welcome contributions from all team members! Please read our [Contributing Guide](CONTRIBUTING.md) to get started.

Key points:
1. Create a new branch for your changes
2. Write clear, documented code
3. Test thoroughly
4. Submit a pull request

## Need Help? 🆘

- Check our documentation
- Ask in our Discord server
- Talk to a mentor
- Open an issue

## License 📄

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments 👏

- WPILib team
- Our mentors
- All team members
- FRC community

## VIPRS 2025 Robot Code

## Overview

This repository contains the robot code for Team VIPRS's 2025 FRC Crescendo competition robot. The code is written in Python using WPILib and follows a command-based structure.

## Features

- **Swerve Drive System**
  - Field-relative control
  - Advanced path following
  - Optimized sensor caching
  - Synchronized module updates

- **Game-Specific Mechanisms**
  - Elevator with precise position control
  - Note handling end effector
  - Speaker and Amp scoring capabilities
  - Stage climbing mechanism

- **Advanced Telemetry**
  - Real-time visualization with AdvantageScope
  - Game-specific data tracking
  - Performance monitoring and statistics
  - System health tracking
  - Comprehensive event logging

## Documentation

- [Architecture](docs/ARCHITECTURE.md) - Overall code structure and design patterns
- [Controls](docs/CONTROLS.md) - Control mappings and driver interface
- [Subsystems](docs/SUBSYSTEMS.md) - Detailed subsystem documentation
- [Telemetry](docs/TELEMETRY.md) - Telemetry system and AdvantageScope integration
- [Sensors](docs/SENSORS.md) - Sensor configuration and usage
- [Constants](docs/CONSTANTS.md) - Robot constants and calibration values

## Getting Started

1. **Prerequisites**
   - Python 3.10 or newer
   - WPILib 2025
   - AdvantageScope (for telemetry visualization)
   - Required Python packages:
     ```bash
     pip install -r requirements.txt
     ```

2. **Setup**
   - Clone this repository
   - Install dependencies
   - Copy `advantagescope-config.json` to your AdvantageScope configuration directory

3. **Development**
   - Use VS Code with the WPILib extension
   - Run simulation: `python robot.py sim`
   - Deploy to robot: `python robot.py deploy`

4. **Telemetry**
   - Start AdvantageScope
   - Connect to robot (NT4)
   - Load VIPRS2025 configuration
   - View real-time data and logs

## Project Structure

```
robot/
├── subsystems/        # Robot subsystems
├── commands/          # Robot commands
├── constants/         # Configuration constants
├── utils/            # Utility classes
│   ├── telemetry.py  # Telemetry system
│   └── ...
├── autonomous/        # Autonomous routines
├── docs/             # Documentation
├── tests/            # Unit tests
├── robot.py          # Main robot class
└── robotcontainer.py # Robot component wiring
```

## Contributing

1. Follow the [Python Style Guide](docs/STYLE.md)
2. Write unit tests for new features
3. Update documentation as needed
4. Submit pull requests for review

## Testing

Run unit tests:
```bash
python -m pytest tests/
```

Run simulation tests:
```bash
python robot.py sim
```

## Telemetry Features

The robot includes a comprehensive telemetry system that provides:

1. **Real-time Visualization**
   - Field position and orientation
   - Swerve module states
   - Mechanism positions
   - Game piece tracking

2. **Performance Monitoring**
   - System health metrics
   - Scoring statistics
   - Autonomous path tracking
   - Event logging

3. **Debug Tools**
   - Event console
   - Statistics tracking
   - System health monitoring
   - Network diagnostics

For detailed information about the telemetry system, see [Telemetry Documentation](docs/TELEMETRY.md).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- WPILib Team
- Mechanical Advantage (AdvantageScope)
- FIRST Robotics Competition 