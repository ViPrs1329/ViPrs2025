# Robot Controls Guide 🎮

## Overview 🌟

Our robot uses two Xbox controllers:
- **Driver Controller** (Port 0) - Controls robot movement
- **Operator Controller** (Port 1) - Controls mechanisms (elevator, end effector)

## Driver Controls 🚗

### Left Stick
- **X-Axis**: Strafe left/right
- **Y-Axis**: Drive forward/backward

### Right Stick
- **X-Axis**: Turn robot left/right

### Triggers & Bumpers
- **Left Trigger**: Precision mode (slower, more accurate)
- **Right Trigger**: Boost mode (faster movement)
- **Left Bumper**: Quick turn left 90°
- **Right Bumper**: Quick turn right 90°

### Buttons
- **A**: Reset gyro (face robot forward)
- **B**: Emergency stop
- **X**: Lock wheels (X pattern)
- **Y**: Auto-align with target

### D-Pad
- **Up/Down**: Fine-tune forward/backward
- **Left/Right**: Fine-tune strafe

## Operator Controls 🦾

### Elevator Controls
- **Left Stick Y**: Manual elevator control
- **A**: Move to base position
- **B**: Move to L1 height
- **X**: Move to L2 height
- **Y**: Move to L3 height

### End Effector Controls
- **Right Trigger**: Coral manipulator grab
- **Left Trigger**: Algae manipulator grab
- **Right Bumper**: Release Coral manipulator
- **Left Bumper**: Release Algae manipulator

### D-Pad
- **Up**: Elevator to max height
- **Down**: Elevator to min height
- **Left**: Retract end effector
- **Right**: Extend end effector

## Safety Features ⚠️

1. **Emergency Stop**
   - Driver's B button stops ALL robot movement
   - Use this if something goes wrong!

2. **Soft Limits**
   - Elevator won't go past safe heights
   - End effector has position limits
   - Drive speeds are capped

3. **Precision Mode**
   - Use for careful alignment
   - Reduces all speeds by 50%

## Special Moves 🌠

### Auto-Align
1. Face roughly toward target
2. Press Y on driver controller
3. Robot aligns automatically

### Quick Pickup
1. Drive near game piece
2. Press appropriate trigger
3. End effector automatically grabs

### Fast Scoring
1. Use preset heights (A/B/X/Y)
2. Release with bumper when ready

## Tips & Tricks 💡

1. **Practice Mode**
   - Start in precision mode
   - Get comfortable before using full speed
   - Practice emergency stops

2. **Smooth Control**
   - Small stick movements
   - Don't slam controls
   - Use precision mode for final adjustments

3. **Communication**
   - Driver calls out movements
   - Operator confirms actions
   - Both watch for obstacles

## Common Issues 🔧

### Robot Won't Move?
1. Check if emergency stop is active
2. Verify controller is on port 0
3. Check battery voltage
4. Look for wheel lockup

### Elevator Stuck?
1. Check if at soft limit
2. Try manual mode
3. Look for obstructions
4. Reset if necessary

### End Effector Problems?
1. Check sensor readings
2. Verify pneumatic pressure
3. Try manual release
4. Reset manipulator

## Practice Exercises 🎯

1. **Basic Movement**
   - Drive in a square
   - Practice precise stops
   - Try different speed modes

2. **Elevator Control**
   - Move between heights smoothly
   - Practice quick height changes
   - Test manual adjustments

3. **Game Piece Handling**
   - Pick up and place objects
   - Use presets effectively
   - Practice quick releases

## Need Help? 🆘

- Ask drive team veterans
- Check diagnostic dashboard
- Practice in a safe area
- Start slow and build up speed 