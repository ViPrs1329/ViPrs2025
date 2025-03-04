# Robot Subsystems Explained

## What's a Subsystem? 🤔

A subsystem is like a part of your body - it has a specific job to do and works with other parts to make the whole robot work. Just like how your arms, legs, and brain all work together to play basketball, our robot's subsystems work together to play the game.

## Our Main Subsystems

### 1. Elevator Subsystem 🛗

**What it Does:**
- Moves game pieces up and down to different heights
- Can move to exact positions (like an elevator in a building)
- Knows exactly where it is at all times

**Key Components:**
- Two SparkFlex motors (left and right) working together
  - Advanced motor controllers for precise movement
  - Built-in motion profiling
  - Smart current limiting
- One motor follows the other (they're linked)
- Absolute encoder to track position
- PID controller for precise movement
- Soft limits for safety

**Cool Features:**
- Can move to preset heights (L1, L2, L3, L4)
- Smooth acceleration and deceleration
- Won't try to go past its limits
- Shows its position on the dashboard

### 2. End Effector Subsystem 🦾

**What it Does:**
- Handles game pieces (coral and algae)
- Has two different mechanisms in one subsystem
- Uses smart sensors to know when it has game pieces

**Algae Manipulator:**
- SparkMax motor for rotation with through bore encoder
  - Absolute position tracking
  - Precise angle control
  - Zero offset calibration
- SparkMax motor for intake/outtake
- PID control for accurate positioning

**Coral Manipulator:**
- Two SparkMax motors for intake/outtake
- LaserCan sensors that can detect pieces within 4 inches
- Keeps track of whether it has a game piece

**Example of How We Use Sensors:**
```python
def get_coral_intake_sensor(self):
    measurement = self.coral_intake_sensor.getMeasurement()
    if measurement and measurement.status == 0:
        # Is something closer than 4 inches (100mm)?
        return measurement.distance_mm < 100
    return False
```

### 3. Drive Subsystem 🚗

**What it Does:**
- Makes the robot move around the field
- Can drive in ANY direction while rotating
- Keeps track of where it is on the field

**How Swerve Drive Works:**
- Each wheel can turn 360 degrees (like a shopping cart wheel)
- Each wheel module has:
  - One motor to make the wheel spin (drive)
  - One motor to make the wheel turn (steer)
  - One CANcoder to know which way the wheel is pointing
- Uses CTRE Pigeon 2.0 IMU for:
  - Precise robot orientation (yaw)
  - Field-relative driving
  - Robot tilt detection (pitch/roll)
  - Odometry updates

**Cool Features:**
- Field-relative driving (forward is always field forward)
- X-pattern for stability (wheels make an X)
- Different speed modes:
  - Normal: Regular driving
  - Precision: Slower for careful movements
  - Boost: Faster for quick movements
- Advanced Sensor Integration:
  - CANcoders for absolute wheel angles
  - Pigeon 2.0 for accurate robot orientation
  - Cached sensor readings for consistency

## How They Work Together 🤝

1. **Driver moves stick forward:**
   - Drive subsystem makes all wheels point forward
   - Drive motors spin to move robot forward

2. **Operator presses elevator button:**
   - Elevator subsystem moves to that height
   - Shows position on dashboard

3. **Operator uses coral intake:**
   - End effector spins intake motors
   - LaserCan sensors detect when piece is grabbed
   - Dashboard updates to show we have a piece

## Want to Try Programming? 💻

Here's a simple example of how we might program a game piece pickup:

```python
def pickup_coral_from_ground():
    # First, move elevator to ground position
    elevator.setPosition(ElevatorConstants.BASE_HEIGHT)
    
    # Wait until elevator is in position
    while not elevator.isAtPosition():
        pass
        
    # Start coral intake
    end_effector.set_coral_intake_speed(CoralManipulatorConstants.INTAKE_SPEED)
    
    # Wait until we detect a game piece
    while not end_effector.get_coral_intake_sensor():
        pass
        
    # Stop intake
    end_effector.set_coral_intake_speed(0)
```

## Need More Info?

- Check out `CONTROLS.md` for how we control everything
- Look at `CONSTANTS.md` for all the numbers we use
- See `SENSORS.md` for more about our sensors 