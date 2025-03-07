# VIPRS 2025 Robot Code

This repository contains the robot code for Team VIPRS' 2025 FRC Reefscape competition robot.

## Project Structure
- `docs/` - Documentation including design documents
- `robot/` - Robot code
  - `subsystems/` - Robot subsystem implementations
  - `commands/` - Command implementations
  - `constants.py` - Robot-wide constants
  - `robot.py` - Main robot implementation

## Setup Instructions

1. Install Python 3.11 or newer
2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   # On Windows:
   .\venv\Scripts\activate
   # On Unix/macOS:
   source venv/bin/activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Install RobotPy on the robot:
   ```bash
   python -m robotpy_installer download-python
   python -m robotpy_installer install-python
   python -m robotpy_installer download
   python -m robotpy_installer install
   ```

## Development

### Running the Robot Code
- **Simulation**: `python robot.py sim`
- **Deploy to Robot**: `python robot.py deploy`
- **Test Mode**: `python robot.py test`

### Code Structure
The robot code follows the Command-Based programming paradigm:
- Each major mechanism is a Subsystem
- Actions are implemented as Commands
- The RobotContainer class handles binding commands to controller inputs

## Contributing
1. Create feature branches from main
2. Follow PEP 8 style guidelines
3. Document new features in the design document
4. Test all changes in simulation before deploying to the robot 