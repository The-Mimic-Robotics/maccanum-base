# Mecanum Base Controller with Encoder Odometry

ESP32-based motor controller for 4-wheel mecanum drive robot with quadrature encoder feedback. **Fully ROS2 REP-103 compliant** for plug-and-play Nav2 integration.

## Coordinate System (ROS2 REP-103)

**Linear Velocities:**
- +X = Forward | +Y = Left | +Z = Up

**Angular Velocity:**
- +Z (yaw) = Counter-clockwise (right-hand rule)

**Frames:**
- `odom`: Fixed world frame
- `base_link`: Robot body frame
- **No conversions needed** - pure REP-103 passthrough

## Quick Start

```bash
# Build & upload
pio run --target upload
pio device monitor

# Test
python3 examples/simple_twist_test.py COM3
```

## Communication Protocol

**Input (Jetson → ESP32):**
```
TWIST,linear_x,linear_y,angular_z
Example: TWIST,0.5,0.2,0.1 (forward + left + CCW)
```

**Output (ESP32 → Jetson, 20Hz):**
```
ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
Example: ODOM,1.234,0.567,0.785,0.5,0.2,0.1,12450,12398,12467,12410

Fields: x,y(m), theta(rad), vx(forward m/s), vy(left m/s), omega(CCW rad/s)
```

## Hardware

### Robot Specs
- Chassis: 460×360mm (4040 aluminum)
- Wheels: 152mm mecanum (76.2mm radius)
- Wheelbase: 468.5mm (width) × 420mm (length)
- Motors: 24V, 92:1 gear, 60 kg·cm torque
- Encoders: 6256 PPR (17×4×92)

### Pin Assignments (Freenove ESP32-WROOM)

| Motor | Enc A | Enc B | DIR | PWM |
|-------|-------|-------|-----|-----|
| FL | GPIO 32 | 33 | 25 | 26 |
| FR | GPIO 23 | 22 | 21 | 19 |
| BL | GPIO 27 | 13 | 14 | 12 |
| BR | GPIO 18 |  4 | 17 | 16 |

**Avoid:** GPIO 0,2,5,6-11,15 (boot/flash conflicts)

## Configuration

### Physical Parameters
`src/mimic_mecanum_base.cpp` (lines 20-27):
```cpp
const float WHEEL_RADIUS = 0.0762;          // 152mm wheels
const float WHEELBASE_WIDTH = 0.4685;       // 468.5mm
const float WHEELBASE_LENGTH = 0.420;       // 420mm
const int ENCODER_PPR = 6256;               // 17×4×92
```

### Control Parameters
`src/main.cpp` (lines 5-9):
```cpp
const float SPEED_MULTIPLIER = 0.8f;        // Speed limit
const unsigned long CONTROL_INTERVAL = 50;  // 20Hz
const unsigned long ODOM_INTERVAL = 50;     // 20Hz
const unsigned long TIMEOUT_MS = 1000;      // Safety timeout
KinematicsMethod KINEMATICS_METHOD = SIMPLE;
```

## Testing

### Test Commands (REP-103)
```
TWIST,0.5,0.0,0.0    # Forward
TWIST,0.0,0.5,0.0    # Strafe left
TWIST,0.0,-0.5,0.0   # Strafe right
TWIST,0.0,0.0,0.5    # Rotate CCW
TWIST,0.0,0.0,-0.5   # Rotate CW
```

### Diagnostic Tools
```bash
# Encoder test (Arduino IDE)
examples/test_encoders_only.ino

# Pin diagnostic
examples/test_back_encoder_pins.ino

# Movement test
python3 examples/simple_twist_test.py COM3
```

## ROS2 Integration

### Bridge Node
```bash
# On Jetson
python3 examples/mecanum_bridge_node.py

# Subscribes: /cmd_vel
# Publishes: /odom, /tf (odom→base_link)
```

### Test ROS2
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.5}}"

# Rotate CCW
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

# Check odometry
ros2 topic echo /odom

# Visualize
rviz2
```

### Nav2 Config
`nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    odom_topic: "odom"

ekf_filter_node:
  ros__parameters:
    frequency: 20.0
    odom0: odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No encoder counts | Check wiring, power, try INPUT_PULLUP |
| Wrong direction | Verify A/B channels not swapped |
| Erratic odometry | Check PPR, wheel radius, wheelbase |
| Motors not responding | Check 24V supply, wiring |
| Rotation inverted | Hardware correction applied in firmware |

## Project Structure

```
mecanum-base/
├── src/
│   ├── main.cpp                      # Main program
│   └── mimic_mecanum_base.cpp        # Motor & odometry
├── include/
│   └── mimic_mecanum_base.h          # Headers
├── examples/
│   ├── test_encoders_only.ino        # Encoder test
│   ├── test_back_encoder_pins.ino    # Pin diagnostic
│   ├── simple_twist_test.py          # Basic test
│   ├── mecanum_bridge_node.py        # ROS2 bridge
│   └── test_encoder_feedback.py      # Advanced test
└── platformio.ini                    # Build config
```

## Features

✅ Pure ROS2 REP-103 (no conversions)  
✅ Hardware PCNT encoders (zero CPU overhead)  
✅ 20Hz real-time odometry  
✅ Safety timeout  
✅ Plug-and-play Nav2 integration  

---

**Author:** Achal Patel  
**Updated:** January 2026  
**License:** MIT
