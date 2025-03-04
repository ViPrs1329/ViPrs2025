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
- Advanced elevator control
- Dual manipulator system
  - Coral game piece handler
  - Algae game piece handler
- Vision processing
- Autonomous routines
- Safety systems

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