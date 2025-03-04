# Robot Constants Explained 🔢

## What Are Constants? 

Constants are like the measurements and settings that make our robot work correctly. Think of them like:
- The recipe measurements in cooking 🧑‍🍳
- The rules in a game 🎮
- The settings on your phone 📱

## Important Numbers for Each System

### Drive System Constants 🚗

**Motor IDs** (Think of these like phone numbers for our motors)
```python
FRONT_LEFT_DRIVE_MOTOR = 1
FRONT_LEFT_TURN_MOTOR = 2
FRONT_RIGHT_DRIVE_MOTOR = 3
FRONT_RIGHT_TURN_MOTOR = 4
# ... and so on for back wheels
```

**Drive Characteristics**
- Wheel Diameter: 4 inches (0.1016 meters)
- Drive Gear Ratio: 8.14:1 (motor turns 8.14 times for wheel to turn once)
- Turn Gear Ratio: 150/7 (for precise steering)

**Speed Settings**
- Normal Speed: 100% (1.0)
- Precision Mode: 50% (0.5) - for careful movements
- Boost Mode: 150% (1.5) - for quick movements

### Elevator Constants ⬆️

**Motor Setup**
- Left Motor ID: 20
- Right Motor ID: 21
- Gear Ratio: 16:1 (motor turns 16 times for one drum rotation)

**Heights** (in meters)
```python
BASE_HEIGHT = 0.0      # Ground level
L1_HEIGHT = 0.5       # First scoring level
L2_HEIGHT = 1.0       # Second scoring level
L3_HEIGHT = 1.5       # Third scoring level
L4_HEIGHT = 2.0       # Maximum height
```

**Safety Limits**
- Minimum Height: -0.05m (slightly below ground)
- Maximum Height: 2.1m (slightly above max needed)
- Position Tolerance: ±0.02m (how close is "close enough")

**Motor Protection**
- Current Limit: 40 amps (maximum power draw)
- Trigger Threshold: 35 amps (warning level)
- Trigger Time: 0.1 seconds (how long before limiting)

### End Effector Constants 🦾

**Coral Manipulator**
- Motor IDs: 30 and 31
- Sensor IDs: 40 and 41
- Detection Range: 4 inches (100mm)
- Intake Speed: 70% power
- Outtake Speed: -70% power

**Algae Manipulator**
- Rotation Motor: 32
- Intake Motor: 33
- Positions (in radians):
  - Retracted: 0.0
  - Top Pickup: 2.1 (~120 degrees)
  - Bottom Pickup: -0.52 (~-30 degrees)

## Why Are Constants Important? 🤔

1. **Consistency**
   - Every part of the code uses the same numbers
   - Easy to change one number and update everywhere
   - No confusion about what numbers to use

2. **Safety**
   - Prevents motors from drawing too much power
   - Keeps mechanisms from going too far
   - Protects the robot from damage

3. **Easy Updates**
   - Need to adjust a speed? Change one number
   - Moving a motor to a different port? Update one constant
   - Want to tune performance? Adjust PID values

## How to Use Constants in Code

Instead of writing numbers directly in your code:
```python
# DON'T do this:
motor.set(0.7)  # What does 0.7 mean?

# DO this instead:
motor.set(CoralManipulatorConstants.INTAKE_SPEED)  # Clear what this means!
```

## Need Help?

- Not sure what a constant means? Ask a mentor!
- Want to change a value? Test small changes first
- Remember: Constants are in ALL_CAPS with underscores
- Check the code comments for more details about each constant 