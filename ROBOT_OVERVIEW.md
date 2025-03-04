# Robot Overview - VIPRS 2025

## What Does Our Robot Do?

Our robot is designed to compete in the 2025 FIRST Robotics Competition. It has three main systems:

1. **Swerve Drive** 🚗
   - Allows the robot to move in ANY direction and rotate at the same time
   - Uses 8 motors (4 for driving, 4 for turning the wheels)
   - Think of it like a shopping cart wheel, but powered and smart!

2. **Elevator** ⬆️
   - Moves up and down to different heights (we call them L1, L2, L3, and L4)
   - Uses 2 motors working together
   - Has safety features to prevent it from going too high or too low

3. **End Effector** (Our Game Piece Handler) 🦾
   - Has two different mechanisms:
     - **Coral Manipulator**: Handles coral game pieces using two motors and special laser sensors
     - **Algae Manipulator**: Handles algae game pieces with a rotating arm and intake

## How Do We Control It?

We use two Xbox controllers:
- **Driver Controller**: Controls how the robot moves around the field
  - Left stick: Makes the robot move forward/backward/left/right
  - Right stick: Makes the robot rotate
  - Bumpers: Precision mode (slower) and boost mode (faster)
  
- **Operator Controller**: Controls the elevator and game piece handlers
  - A/B/X/Y buttons: Move elevator to different heights
  - Bumpers: Control coral intake/outtake
  - Triggers: Control algae intake/outtake
  - Other buttons: Move algae arm to different positions

## Cool Features! 🌟

1. **Smart Sensors**
   - LaserCan sensors that can detect game pieces within 4 inches
   - Absolute encoders to always know where the elevator is
   - CANcoders to know exactly how our wheels are pointed

2. **Safety First**
   - Software limits to prevent the elevator from going too high/low
   - Current limiting to protect our motors
   - Emergency "X" formation for stability

3. **Precision Control**
   - PID control for accurate elevator positioning
   - Motion profiling for smooth movement
   - Field-relative driving option (robot moves relative to field, not itself)

## Want to Learn More?

Check out our other documentation files:
- `SUBSYSTEMS.md` - Detailed info about each major system
- `CONTROLS.md` - Complete control mapping and features
- `CONSTANTS.md` - All the numbers that make everything work
- `SENSORS.md` - How we use sensors to make the robot smart 