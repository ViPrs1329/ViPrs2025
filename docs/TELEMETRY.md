# Robot Telemetry System Documentation

## Overview

The VIPRS 2025 robot uses a comprehensive telemetry system that integrates with AdvantageScope for real-time visualization and debugging. This system provides:

- Real-time robot state visualization
- Game-specific data tracking for Crescendo
- Performance monitoring and statistics
- System health tracking
- Event logging and debugging tools

## Setup and Configuration

### 1. Installation

1. Install AdvantageScope from [https://github.com/Mechanical-Advantage/AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
2. Copy `advantagescope-config.json` to your AdvantageScope configuration directory
3. Install required Python dependencies:
   ```bash
   pip install psutil wpilib ntcore
   ```

### 2. Basic Usage

1. Start AdvantageScope
2. Connect to your robot using NT4 (NetworkTables)
3. Load the VIPRS2025 configuration
4. View real-time data during matches or replay log files

## Telemetry Features

### 1. Field Visualization

- Real-time robot position and orientation
- Game element locations (Speakers, Amps, Sources, Stage)
- Autonomous path visualization
- Target tracking

### 2. Swerve Drive Visualization

- Individual module states
- Drive and turn angles
- Velocities and currents
- Module temperatures

### 3. Mechanism Visualization

- Elevator position and state
- End effector configuration
- Note handling mechanisms
- Real-time updates

### 4. Game-Specific Features

#### Note Detection
```python
telemetry.log_note_detection(
    note_detected=True,
    note_distance=1.2,  # meters
    note_angle=-10.0    # degrees
)
```

#### Scoring State
```python
telemetry.log_scoring_state(
    speaker_ready=True,
    speaker_distance=2.5,
    speaker_angle=15.0,
    amp_ready=False,
    stage_position="PARKED"
)
```

#### Scoring Statistics
```python
telemetry.log_scoring_success(
    location="SPEAKER",
    success=True,
    distance=2.5,
    angle=15.0
)
```

### 5. System Health Monitoring

- CPU and memory usage
- Battery voltage
- Motor temperatures
- Automatic warnings for:
  - High CPU usage (>80%)
  - High memory usage (>80%)
  - Low battery voltage
  - High temperatures

## Integration Guide

### 1. Subsystem Integration

To add telemetry to a subsystem:

1. Inherit from `TelemetrySubsystemBase`:
```python
from utils.telemetry_subsystem_base import TelemetrySubsystemBase

class MySubsystem(TelemetrySubsystemBase):
    def __init__(self, telemetry):
        super().__init__(telemetry, "MySubsystem")
```

2. Implement required methods:
```python
def cache_sensors(self) -> None:
    """Cache sensor readings"""
    self.set_cached("sensor1", self.sensor1.getValue())
    self.set_cached("sensor2", self.sensor2.getValue())

def periodic_logic(self) -> None:
    """Process cached values"""
    # Process data and update state
    pass

def update_hardware(self) -> None:
    """Update hardware with cached values"""
    # Update motors, actuators, etc.
    pass
```

3. Set up health monitoring:
```python
def _setup_health_monitoring(self) -> None:
    self.register_health_threshold(
        "temperature",
        max_value=80.0,    # Error if above 80°C
        warning_max=70.0   # Warning if above 70°C
    )
```

### 2. Logging Events

```python
# Log general events
self.log_event("State Change", "Elevator reached target position")

# Log mechanism state
self.telemetry.log_mechanism_state("Elevator", {
    "height": current_height,
    "velocity": current_velocity
})

# Log scoring attempts
self.telemetry.log_scoring_success(
    location="SPEAKER",
    success=True,
    distance=2.5,
    angle=15.0
)
```

### 3. Autonomous Path Logging

```python
# Log planned path
self.telemetry.log_auto_path(path_poses)
```

## Dashboard Layout

The AdvantageScope dashboard is organized into three rows:

1. **Main Visualization** (Top Row)
   - Field View
   - Swerve Module State
   - Mechanism View
   - Gyro Visualization

2. **Performance Monitoring** (Middle Row)
   - System Health
   - Game Piece Tracking
   - Scoring Statistics
   - General Statistics

3. **Event Logging** (Bottom Row)
   - Subsystem Events
   - Vision Events
   - Scoring Events
   - System Events

## Best Practices

1. **Data Caching**
   - Cache sensor values once per loop
   - Use cached values for calculations
   - Update hardware at end of loop

2. **Health Monitoring**
   - Set appropriate thresholds for critical values
   - Monitor temperatures and voltages
   - Log warning events for investigation

3. **Event Logging**
   - Log significant state changes
   - Include relevant details in event messages
   - Use consistent naming conventions

4. **Performance**
   - Cache values to reduce CAN traffic
   - Update statistics periodically (every 50 samples)
   - Monitor system health metrics

## Troubleshooting

### Common Issues

1. **No Data in AdvantageScope**
   - Check NetworkTables connection
   - Verify robot IP address
   - Ensure telemetry manager is initialized

2. **Missing Mechanism Updates**
   - Check mechanism table paths
   - Verify data types match expected values
   - Ensure periodic updates are running

3. **High Resource Usage**
   - Review update frequencies
   - Check for tight loops
   - Monitor system health metrics

### Debug Tools

1. **Event Console**
   - Filter by subsystem
   - Search for specific events
   - Track timing of events

2. **Statistics View**
   - Monitor success rates
   - Track average distances
   - Analyze performance trends

3. **System Health**
   - CPU and memory usage
   - Network statistics
   - Subsystem temperatures

## Additional Resources

- [AdvantageScope Documentation](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/README.md)
- [WPILib NetworkTables Guide](https://docs.wpilib.org/en/stable/docs/software/networktables/index.html)
- [FRC Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system) 