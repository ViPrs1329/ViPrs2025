# Robot Sensors Guide 🔍

This guide explains all the sensors used on our robot and how they help us control it accurately.

## IMU (Inertial Measurement Unit) 🧭

### CTRE Pigeon 2.0
- **Purpose**: Measures robot orientation and motion
- **Key Features**:
  - Highly accurate yaw (rotation) measurement
  - Pitch and roll detection for tilt monitoring
  - Fast update rate (100Hz)
  - CAN bus communication
  - Phoenix 6 library integration
- **Usage**:
  ```python
  # In DriveSubsystem
  self.gyro = Pigeon2(DriveConstants.PIGEON_ID)
  
  # Getting values
  yaw = self.gyro.get_yaw().value    # Robot rotation
  pitch = self.gyro.get_pitch().value # Forward/backward tilt
  roll = self.gyro.get_roll().value   # Side-to-side tilt
  ```
- **Applications**:
  - Field-relative driving
  - Auto-balancing
  - Odometry updates
  - Tilt safety monitoring

## Absolute Encoders 🔄

### CTRE CANcoders
- **Purpose**: Measure absolute wheel angles for swerve drive
- **Key Features**:
  - Absolute position tracking
  - High precision
  - Maintains position through power cycles
  - CAN bus communication
- **Usage**:
  ```python
  # In SwerveModule
  self.cancoder = CANcoder(cancoder_id)
  position = self.cancoder.get_position().value
  ```
- **Applications**:
  - Swerve module angle control
  - Module calibration
  - Absolute position tracking

## Through Bore Encoders 📏

### REV Through Bore Encoder
- **Purpose**: Measure elevator height
- **Key Features**:
  - Absolute position tracking
  - Direct shaft mounting
  - High resolution
- **Usage**:
  ```python
  # In ElevatorSubsystem
  self.absolute_encoder = self.left_motor.getAbsoluteEncoder(
      SparkAbsoluteEncoder.Type.kDutyCycle
  )
  ```
- **Applications**:
  - Elevator height control
  - Position limits
  - Motion profiling

## Distance Sensors 📏

### LaserCan Sensors
- **Purpose**: Detect game pieces and measure distances
- **Key Features**:
  - High accuracy distance measurement
  - Fast update rate
  - CAN bus communication
- **Usage**:
  ```python
  # In EndEffector
  measurement = self.coral_intake_sensor.getMeasurement()
  if measurement.status == 0:
      distance = measurement.distance_mm
  ```
- **Applications**:
  - Game piece detection
  - Intake control
  - Position verification

## Motor Encoders 🔄

### NEO Built-in Encoders
- **Purpose**: Measure motor rotation and velocity
- **Key Features**:
  - Integrated into NEO motors
  - High resolution
  - Velocity and position measurement
- **Usage**:
  ```python
  # In any motor controller
  self.encoder = self.motor.getEncoder()
  position = self.encoder.getPosition()
  velocity = self.encoder.getVelocity()
  ```
- **Applications**:
  - Motor control
  - Velocity measurement
  - Position tracking

## Sensor Integration 🔗

### Caching System
- All sensor readings are cached once per loop
- Reduces CAN bus traffic
- Ensures consistent values within each cycle
- Example:
  ```python
  def cache_sensors(self):
      self.cache.set_cached("gyro_angle", self.gyro.get_yaw().value)
      self.cache.set_cached("roll", self.gyro.get_roll().value)
      self.cache.set_cached("pitch", self.gyro.get_pitch().value)
  ```

### SmartDashboard Integration
- Sensor values displayed on dashboard
- Real-time monitoring
- Debugging support
- Example:
  ```python
  def periodic_logic(self):
      wpilib.SmartDashboard.putNumber("Robot Heading", 
          self.cache.get_cached("gyro_angle"))
      wpilib.SmartDashboard.putNumber("Robot Roll",
          self.cache.get_cached("roll"))
  ```

## Best Practices 🌟

1. **Always Use Cached Values**
   - Don't read sensors directly in periodic loops
   - Access cached values for consistency
   - Update cache at predictable intervals

2. **Handle Sensor Failures**
   - Check sensor status before using values
   - Have fallback behaviors
   - Log errors appropriately

3. **Regular Calibration**
   - Zero sensors at robot initialization
   - Account for offsets
   - Verify sensor readings

4. **Documentation**
   - Keep track of sensor IDs
   - Document calibration procedures
   - Note any special considerations

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