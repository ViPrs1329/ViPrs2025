# Robot Control Scheme Reference

## Driver Controller (Primary)

### Movement Controls
- **Left Joystick**: Swerve drive translation (forward/backward, left/right)
- **Right Joystick X-axis**: Swerve drive rotation
- **Left Bumper**: Precision mode (50% speed)
- **Right Bumper**: Boost mode (150% speed)

### System Controls
- **Start Button**: Reset gyro/heading (important for field-oriented control)
- **Back/Select Button**: Toggle field-oriented control on/off
- **Y Button**: Emergency stop all subsystems

## Operator Controller (Secondary)

### Mode Selection
- **A Button**: Base Mode (Resets to home position)
- **X Button**: Coral Mode (For coral manipulation)
- **Y Button**: Algae Mode (For algae manipulation)

### When in Coral Mode
- **Left Bumper**: Intake coral (until positioned)
- **Right Bumper**: Eject coral
- **D-Pad Up**: Elevator to high position (L3)
- **D-Pad Left**: Elevator to medium position (L2)
- **D-Pad Right**: Elevator to low position (L1)
- **D-Pad Down**: Elevator to home position

### When in Algae Mode
- **Left Bumper**: Intake algae (hold to run)
- **Right Bumper**: Eject algae (hold to run)
- **D-Pad Up**: Algae mechanism to top pickup position
- **D-Pad Down**: Algae mechanism to bottom pickup position
- **D-Pad Left**: Algae mechanism to retracted position

### Universal Controls (All Modes)
- **Right Trigger + Right Joystick Y-axis**: Manual elevator control

## General Notes

1. The robot uses a mode-based approach for the operator controller. First, select the mode (Base, Coral, or Algae), then use the controls specific to that mode.

2. Field-oriented driving can be toggled on/off. When enabled, the robot will drive relative to the field regardless of its orientation.

3. Emergency stop (Driver Controller Y button) will immediately stop all subsystems.

4. The manual elevator control (Right Trigger + Right Joystick) works in all modes.

5. The elevator position commands work only in Coral Mode.

6. The algae position and intake commands work only in Algae Mode.

7. Base Mode resets the elevator to its home position and is a good starting point for operations.