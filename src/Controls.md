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
- **B Button + D-Pad Up**: Quick score high (moves elevator to high position, ejects coral, returns to home)
- **B Button + D-Pad Left**: Quick score medium (moves elevator to medium position, ejects coral, returns to home)
- **B Button + D-Pad Right**: Quick score low (moves elevator to low position, ejects coral, returns to home)

### When in Algae Mode
- **Left Bumper**: Intake algae (hold to run)
- **Right Bumper**: Eject algae (hold to run)
- **D-Pad Up**: Algae mechanism to top pickup position (90 degrees)
- **D-Pad Down**: Algae mechanism to bottom pickup position (-90 degrees)
- **D-Pad Left**: Algae mechanism to retracted position (0 degrees)

### Universal Controls (All Modes)
- **Right Trigger + Right Joystick Y-axis**: Manual elevator control (hold right trigger and move joystick up/down)

## Feedback Features
- **Controller Rumble**: Activates when motors exceed temperature thresholds (90°C) to alert the driver

## Autonomous Mode
The robot is equipped with autonomous capabilities that can be selected before the match starts. The autonomous routines leverage the PathPlanner library for trajectory following with the swerve drive.

## General Notes

1. **Mode-Based Controls**: The robot uses a mode-based approach for the operator controller. First, select the mode (Base, Coral, or Algae), then use the controls specific to that mode.

2. **Field-Oriented Driving**: Can be toggled on/off. When enabled, the robot will drive relative to the field regardless of its orientation.

3. **Emergency Stop**: Driver Controller Y button will immediately stop all subsystems.

4. **Manual Elevator Control**: Right Trigger + Right Joystick works in all modes.

5. **Elevator Positions**:
   - **Home**: Fully retracted/stowed position (0.0 encoder counts)
   - **Low**: Position for scoring in the low goal (20.0 encoder counts)
   - **Medium**: Position for scoring in the medium goal (50.0 encoder counts)
   - **High**: Position for scoring in the high goal (95.0 encoder counts)

6. **Algae Positions**:
   - **Retracted**: Mechanism at home position (0 degrees)
   - **Top Pickup**: Mechanism rotated upward for top pickup (90 degrees)
   - **Bottom Pickup**: Mechanism rotated downward for bottom pickup (-90 degrees)

7. **Base Mode**: Resets the elevator to its home position and is a good starting point for operations.