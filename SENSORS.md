# Understanding Our Robot's Sensors 🔍

## Why Do We Need Sensors? 

Sensors are like the robot's senses - they help it understand:
- Where it is 📍
- What it's doing 🔄
- If it has game pieces 🎮
- How fast it's moving 🏃‍♂️

## Our Main Sensors

### 1. LaserCan Sensors 📏

**What They Do:**
- Measure exact distance to objects
- Can detect game pieces up to 4 inches away
- Used in the Coral Manipulator

**How They Work:**
```python
# Example of reading a LaserCan sensor:
measurement = sensor.getMeasurement()
if measurement and measurement.status == 0:
    distance = measurement.distance_mm  # Distance in millimeters
```

**Cool Features:**
- Super precise (measures in millimeters!)
- Can tell if something is actually there
- Shows measurements on SmartDashboard
- Configurable for different situations:
  - Short range mode for accurate close measurements
  - Adjustable timing for faster/slower updates
  - Configurable region of interest

### 2. CANcoders 🎡

**What They Do:**
- Tell us exactly which way our swerve wheels are pointing
- Measure rotation in degrees/radians
- Remember their position even when robot is turned off

**Where We Use Them:**
- One on each swerve module (4 total)
- Connected to the turning motors
- Help us drive accurately

**Important Settings:**
- Absolute position tracking
- Zero offset calibration
- Built-in direction configuration

### 3. NEO Motor Encoders 🔄

**What They Do:**
- Built into our SparkMax motors
- Count motor rotations
- Measure speed

**Where We Use Them:**
- Drive motors (how far we've driven)
- Elevator motors (backup position tracking)
- Algae arm rotation (position feedback)

### 4. Through Bore Encoder ⚙️

**What They Do:**
- Super accurate rotation measurement
- Absolute position tracking
- Used on the elevator for precise height control

**Features:**
- Never loses its position
- Very precise measurements
- Directly measures mechanism movement

## How We Use Sensor Data 📊

### Dashboard Display
```python
# Example of showing sensor data:
SmartDashboard.putNumber("Elevator Height", encoder.getPosition())
SmartDashboard.putNumber("Coral Distance", laser_sensor.getMeasurement().distance_mm)
```

### Safety Checks
```python
# Example of using sensors for safety:
if elevator_height > MAX_HEIGHT:
    stop_motors()
```

### Automatic Controls
```python
# Example of sensor-based automation:
def wait_for_game_piece():
    while not coral_sensor.is_piece_detected():
        continue
    stop_intake()
```

## Troubleshooting Sensors 🔧

### Common Issues:
1. **Sensor Shows Wrong Values**
   - Check wiring
   - Verify CAN ID/port
   - Look for loose connections

2. **Sensor Not Responding**
   - Check power
   - Verify network connection
   - Reset the sensor

3. **Inconsistent Readings**
   - Clean sensor surface
   - Check for interference
   - Verify sensor configuration

### When to Ask for Help:
- Sensor values seem very wrong
- Sensor stops working completely
- Unexpected behavior
- Configuration issues

## Want to Learn More? 🤓

- Check out the sensor datasheets
- Look at example code in WPILib
- Ask mentors about sensor calibration
- Try writing your own sensor code! 