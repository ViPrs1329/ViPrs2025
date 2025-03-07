# FRC 2025 Reefscape Robot Design Document

## Overview
This document outlines the design specifications and requirements for Team VIPRS' 2025 FRC Reefscape competition robot.

## 1. Drive System - Swerve Drive

### Specifications
- **Module Type**: SDS MK4i L2 Gearing
- **Configuration**: 
  - 4 modules
  - 29" x 29" frame
  - 4" wheels
  - Maximum speed: 12 ft/s (limited from 15.1 ft/s)
- **Motors per Module**: 2x NEO motors (Drive + Steering)
- **Controllers**: SPARK MAX
- **Sensors**: CANcoders (absolute encoders) for each module

### Control Features
- PID control for precise movement
- Robot-oriented driving by default
- Precision mode for fine control
- Field-oriented driving available but not default

## 2. Game Piece Manipulation Systems

### A. Passive Funnel (CORAL Intake)
- **Location**: Rear of robot
- **Game Piece**: CORAL (4" PVC pipe segments)
- **Type**: Passive funnel system
- **Function**: 
  - Accepts CORAL from rear
  - Guides CORAL to front-mounted End Effector
  - No sensors required

### B. End Effector

#### CORAL Handling
- **Mechanism**: Two pairs of powered wheels
- **Motors**: 2x NEO with SPARK MAX
- **Sensors**: 
  - Through bore encoder
  - Two CANrange sensors (front and end)
- **Scoring Heights**:
  - Base Position: 25.36" (End Effector at elevator bottom)
  - L1: 18" from ground
  - L2: 31.875" from ground
  - L3: 47.625" from ground
  - L4: 72" from ground
- **Control**: 
  - Button-based level selection
  - Manual CORAL ejection control

#### ALGAE Handling
- **Mechanism**: Rotating arm intake
- **Motors**: 
  - 1x NEO/SPARK MAX for arm rotation
  - 1x NEO/SPARK MAX for intake wheels
- **Positions**:
  - Rest: 0 degrees (straight down)
  - Working: 40 degrees
- **Operation**:
  - Continuous intake until manual expel
  - Only operates at elevator bottom position
  - No automated sequences

### C. Elevator System
- **Type**: Cascading
- **Motors**: 2x NEO Vortex with built-in SPARK Flex
- **Maximum Height**: 70.86 inches
- **Sensors**: 
  - REV Through Bore Encoder (absolute)
  - Current monitoring for soft limits
- **Features**:
  - Software-based soft limits using current monitoring
  - Preset heights for scoring positions
  - PID control for precise positioning

## 3. Vision and Sensing
- **Hardware**: 2x Limelight 2 (front-mounted)
- **Future Capabilities** (not initially implemented):
  - Basic AprilTag detection
  - Vision-assisted alignment
  - Field-aware driving assistance

## 4. Autonomous
- **Initial Requirement**: 4-foot straight drive
- **Control**: PID-based movement
- **Architecture**: 
  - Designed for easy addition of future routines
  - Basic autonomous framework

## 5. Controls and Human Interface

### Driver Controls
- **Hardware**: Xbox Controllers
- **Configuration**:
  - Primary Driver: Robot movement
  - Secondary Driver: Game piece manipulation
  - D-pad buttons reserved (not used)
- **Safety Features**:
  - Interlocks for critical operations
  - Speed limitations during certain operations

### Dashboard (SmartDashboard/Glass)
- **Critical Information Display**:
  - Motor currents
  - Mechanism positions
  - Robot state
  - PID tuning interface
- **Debug Information**:
  - Sensor readings
  - System states
  - Error conditions

## Implementation Notes
- All mechanisms require PID control implementation
- Safety interlocks need to be implemented between subsystems
- Dashboard should prioritize critical debugging information
- Vision processing implementation is low priority 