# Contributing to Our Robot Code 🤖

Welcome to the coding team! This guide will help you get started with contributing to our robot's code.

## Getting Started 🚀

### 1. Set Up Your Development Environment
- Install Python 3.11 or newer
- Install Visual Studio Code
- Install Git
- Clone our repository:
```bash
git clone https://github.com/your-team/robot-code-2025.git
cd robot-code-2025
```

### 2. Install Dependencies
```bash
# Create a virtual environment
python -m venv venv

# Activate it (Windows)
.\venv\Scripts\activate

# Install requirements
pip install -r requirements.txt
```

## Making Changes 🛠️

### Step 1: Create a New Branch
```bash
# Get latest changes
git pull origin main

# Create and switch to new branch
git checkout -b feature/your-feature-name
```

### Step 2: Write Your Code
1. **Follow Our Style**
   - Use clear variable names
   - Add comments explaining WHY, not WHAT
   - Keep functions short and focused
   - Use constants for magic numbers

2. **Example of Good Code**:
```python
def move_elevator_to_position(self, target_height: float) -> None:
    """Moves elevator to specified height with safety checks.
    
    Args:
        target_height: Target height in meters
    """
    # Check if movement is safe
    if not self.is_movement_safe(target_height):
        self.logger.warning(f"Unsafe movement to {target_height}m blocked")
        return
        
    # Move to position
    self.elevator_motor.set_position(target_height)
```

### Step 3: Test Your Changes
1. Run the simulator
2. Test all related functions
3. Check for errors in SmartDashboard
4. Have a teammate review your code

## Code Organization 📁

### Where to Put New Code
- **New Commands**: `commands/` folder
- **New Subsystems**: `subsystems/` folder
- **New Constants**: `constants/constants.py`
- **New Tests**: `tests/` folder

### Naming Conventions
- **Files**: lowercase_with_underscores.py
- **Classes**: CapitalizedWords
- **Functions**: lowercase_with_underscores()
- **Constants**: UPPERCASE_WITH_UNDERSCORES

## Common Tasks 📝

### Adding a New Command
1. Create new file in `commands/`
2. Inherit from `Command` class
3. Implement required methods
4. Add to `RobotContainer`

Example:
```python
from commands2 import Command

class MyNewCommand(Command):
    def __init__(self, subsystem):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(subsystem)
    
    def initialize(self):
        # Setup code here
        pass
        
    def execute(self):
        # Running code here
        pass
        
    def end(self, interrupted: bool):
        # Cleanup code here
        pass
        
    def isFinished(self) -> bool:
        # Return True when done
        return False
```

### Adding a New Subsystem
1. Create new file in `subsystems/`
2. Inherit from `SubsystemBase`
3. Add to `RobotContainer`

Example:
```python
from commands2 import SubsystemBase

class MyNewSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        # Initialize hardware here
        
    def periodic(self):
        # Update dashboard here
        pass
```

## Best Practices 🌟

### 1. Safety First
- Add limit checks
- Handle errors gracefully
- Test in simulation first
- Document safety features

### 2. Keep It Simple
- Write clear, focused code
- Use meaningful names
- Break complex tasks into smaller parts
- Comment unclear sections

### 3. Be a Team Player
- Update documentation
- Help review code
- Share knowledge
- Ask for help when stuck

## Common Issues & Solutions 🔧

### "Git says 'merge conflict'"
1. Open the conflicting files
2. Look for <<<<<<< and >>>>>>>
3. Choose which changes to keep
4. Save and commit

### "Import error when running code"
1. Check virtual environment is active
2. Verify requirements.txt is up to date
3. Try `pip install -r requirements.txt`

### "Robot simulator won't start"
1. Check Python version
2. Verify WPILib installation
3. Look for error messages
4. Ask for help in Discord

## Getting Help 🆘

1. **Check Documentation**
   - Read relevant docs
   - Look at similar code
   - Search commit history

2. **Ask Team**
   - Post in Discord
   - Ask at meetings
   - Pair program

3. **External Resources**
   - WPILib docs
   - Chief Delphi
   - Python docs

## Review Process 👀

1. **Before Submitting**
   - Code works in simulator
   - All tests pass
   - No style issues
   - Documentation updated

2. **Creating Pull Request**
   - Clear description
   - Link related issues
   - Request reviewers
   - Add test results

3. **After Review**
   - Address feedback
   - Update tests
   - Re-request review

## Thank You! 🙏

Your contributions help make our robot better! Remember:
- Start small
- Ask questions
- Help others
- Have fun! 