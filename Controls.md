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
- **B Button**: Context-dependent action (eject/score based on current mode)

### When in Coral Mode
- **Left Bumper**: Intake coral (until positioned)
- **Right Bumper**: Eject coral
- **X + A**: Elevator to low position
- **X + X**: Elevator to medium position
- **X + Y**: Elevator to high position
- **X + B**: Quick score (eject coral, wait, return to home)

### When in Algae Mode
- **Left Bumper**: Intake algae (hold to run)
- **Right Bumper**: Eject algae (hold to run)
- **Y + A**: Algae mechanism to retracted position
- **Y + X**: Algae mechanism to bottom pickup position
- **Y + Y**: Algae mechanism to top pickup position

### Universal Controls
- **Right Trigger + Right Joystick Y-axis**: Manual elevator control (hold right trigger and move joystick up/down)
- **Left Trigger + Right Joystick Y-axis**: Manual algae rotation control (hold left trigger and move joystick up/down)

## Feedback Features
- **Controller Rumble**: Activates when motors exceed temperature thresholds (90°C) to alert the driver

## Autonomous Mode
The robot is equipped with autonomous capabilities that can be selected before the match starts. The autonomous routines leverage the PathPlanner library for trajectory following with the swerve drive.

## General Notes

1. **Mode-Based Controls**: The robot uses a mode-based approach for the operator controller. First, select the mode (Base, Coral, or Algae), then use the controls specific to that mode.

2. **Button Combinations**: Many functions use button combinations where you hold one button (the mode button) and press another. For example, in Coral Mode, hold X and press A to set the elevator to the low position.

3. **Field-Oriented Driving**: Can be toggled on/off. When enabled, the robot will drive relative to the field regardless of its orientation.

4. **Emergency Stop**: Driver Controller Y button will immediately stop all subsystems.

5. **Manual Controls**: Right Trigger + Right Joystick works for manual elevator control, and Left Trigger + Right Joystick works for manual algae rotation control in all modes.

6. **Elevator Positions**:
   - **Home**: Fully retracted/stowed position (0.0 encoder counts)
   - **Low**: Position for scoring in the low goal (20.0 encoder counts)
   - **Medium**: Position for scoring in the medium goal (50.0 encoder counts)
   - **High**: Position for scoring in the high goal (95.0 encoder counts)

7. **Algae Positions**:
   - **Retracted**: Mechanism at home position (0 degrees)
   - **Top Pickup**: Mechanism rotated upward for top pickup (90 degrees)
   - **Bottom Pickup**: Mechanism rotated downward for bottom pickup (-90 degrees)

8. **Base Mode**: Resets the elevator to its home position and is a good starting point for operations.