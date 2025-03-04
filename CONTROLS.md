# Robot Control Scheme

This document outlines the control scheme for Team 1329's 2025 FRC robot. The robot uses a two-driver control system with Xbox controllers.

## Controller Layout Reference

### Xbox Controller Button Map
![Xbox Controller Layout](https://user-images.githubusercontent.com/580022/45268303-10a03e80-b4ce-11e8-883c-1f586566c040.png)

| Button/Axis | Number/Name |
|-------------|-------------|
| A Button | 1 |
| B Button | 2 |
| X Button | 3 |
| Y Button | 4 |
| Left Bumper | 5 |
| Right Bumper | 6 |
| Back Button | 7 |
| Start Button | 8 |
| Left Stick Button | 9 |
| Right Stick Button | 10 |
| Left Trigger | Axis 2 |
| Right Trigger | Axis 3 |
| Left Stick X | Axis 0 |
| Left Stick Y | Axis 1 |
| Right Stick X | Axis 4 |
| Right Stick Y | Axis 5 |

## Driver 1 (Driving)

Driver 1 is responsible for controlling the robot's movement using the swerve drive system.

### Movement Controls
| Control | Function |
|---------|----------|
| Left Stick Y-Axis | Forward/Backward Movement |
| Left Stick X-Axis | Left/Right Movement |
| Right Stick X-Axis | Rotation |

### Special Functions
| Button | Function | Description |
|--------|----------|-------------|
| A Button | X-Formation | Positions wheels in an X pattern for stability |
| Left Bumper | Precision Mode | Reduces speed for precise movements |
| Right Bumper | Boost Mode | Increases speed for faster movements |
| Back Button | Reset Gyro | Resets the robot's heading to zero |
| Start Button | Toggle Field-Relative | Switches between field-relative and robot-relative control |
| Y Button | Test Swerve | Runs the swerve drive test sequence (temporary) |

## Driver 2 (Mechanisms)

Driver 2 is responsible for controlling the robot's mechanisms, including the elevator, Coral manipulator, and Algae manipulator.

### Elevator Controls
| Button | Function | Description |
|--------|----------|-------------|
| A Button | Base Position | Moves elevator to the base position (fully retracted) |
| B Button | L1 Position | Moves elevator to the L1 height (0.5m) |
| X Button | L2 Position | Moves elevator to the L2 height (1.0m) |
| Y Button | L3 Position | Moves elevator to the L3 height (1.5m) |
| Right Stick Button | L4 Position | Moves elevator to the L4 height (2.0m) |

### Coral Manipulator Controls
| Button | Function | Description |
|--------|----------|-------------|
| Left Bumper | Intake | Runs the Coral intake (while held) |
| Right Bumper | Outtake | Runs the Coral outtake (while held) |

### Algae Manipulator Controls
| Control | Function | Description |
|---------|----------|-------------|
| Back Button | Retracted Position | Moves Algae arm to retracted position (0.0 rad) |
| Start Button | Top Pickup | Moves Algae arm to top pickup position (2.1 rad, ~120°) |
| Left Stick Button | Bottom Pickup | Moves Algae arm to bottom pickup position (-0.52 rad, ~-30°) |
| Left Trigger | Intake | Runs Algae intake (proportional to trigger pull) |
| Right Trigger | Outtake | Runs Algae outtake (proportional to trigger pull) |

## Control Diagram

```
Driver 1 (Driving)                    Driver 2 (Mechanisms)
┌───────────────────────┐             ┌───────────────────────┐
│    ┌───┐     ┌───┐    │             │    ┌───┐     ┌───┐    │
│    │LB │     │RB │    │             │    │LB │     │RB │    │
│    └───┘     └───┘    │             │    └───┘     └───┘    │
│                       │             │                       │
│    ┌───┐     ┌───┐    │             │    ┌───┐     ┌───┐    │
│    │LT │     │RT │    │             │    │LT │     │RT │    │
│    └───┘     └───┘    │             │    └───┘     └───┘    │
│                       │             │                       │
│  ┌─────┐     ┌─────┐  │             │  ┌─────┐     ┌─────┐  │
│  │     │     │     │  │             │  │     │     │     │  │
│  │  ←→ │     │  ←→ │  │             │  │  ←→ │     │  ←→ │  │
│  │  ↑↓ │     │  ↑↓ │  │             │  │  ↑↓ │     │  ↑↓ │  │
│  └─────┘     └─────┘  │             │  └─────┘     └─────┘  │
│                       │             │                       │
│    ┌───┐     ┌───┐    │             │    ┌───┐     ┌───┐    │
│    │Bck│     │Str│    │             │    │Bck│     │Str│    │
│    └───┘     └───┘    │             │    └───┘     └───┘    │
│                       │             │                       │
│      ┌───┐ ┌───┐      │             │      ┌───┐ ┌───┐      │
│      │ X │ │ Y │      │             │      │ X │ │ Y │      │
│      └───┘ └───┘      │             │      └───┘ └───┘      │
│      ┌───┐ ┌───┐      │             │      ┌───┐ ┌───┐      │
│      │ A │ │ B │      │             │      │ A │ │ B │      │
│      └───┘ └───┘      │             │      └───┘ └───┘      │
└───────────────────────┘             └───────────────────────┘
```

## Notes for Drivers

- **Field-Relative Control**: When enabled, the robot moves relative to the field, not its orientation. Forward on the stick is always away from the driver, regardless of robot orientation.
- **Precision Mode**: Use this for fine adjustments or when near other robots/field elements.
- **Boost Mode**: Use this for quick traversal across the field.
- **X-Formation**: Use this to lock the robot in place when stability is needed.
- **Elevator Positions**: The heights are approximate and may be adjusted during competition.
- **Trigger Controls**: The Algae intake/outtake speed is proportional to how far the trigger is pressed.

## Updating Controls

If you need to modify the control scheme, update the following files:
- `constants/constants.py` - `OIConstants` class
- `robotcontainer.py` - `configureButtonBindings()` method 