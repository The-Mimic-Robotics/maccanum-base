# Mecanum Base Controller with Encoder Odometry

ESP32-based motor controller for 4-wheel mecanum drive robot with quadrature encoder feedback for Nav2 odometry.

## Coordinate System (ROS2 REP-103)

This system follows the **ROS2 REP-103 standard** for all command and odometry data:

**Linear Velocities:**
- **+X** = Forward (robot moves forward)
- **+Y** = Left (robot strafes left)
- **+Z** = Up (not used for ground robots)

**Angular Velocity:**
- **+Z (yaw)** = Counter-clockwise rotation (right-hand rule)

**Frame Conventions:**
- `odom` frame: Fixed world frame where odometry is calculated
- `base_link` frame: Robot body frame (moves with robot)
- All Twist commands and Odometry messages use this coordinate system

## Quick Start

### Build & Upload
```bash
pio run --target upload
pio device monitor
```

### Configuration Parameters
**Location:** `src/mimic_mecanum_base.cpp` - Top of file (lines 20-32)

```cpp
// Robot physical parameters - adjust these to match your robot
const float WHEEL_RADIUS = 0.0762;          // meters (152mm diameter wheels)
const float WHEELBASE_WIDTH = 0.4685;       // meters (468.5mm L/R spacing)
const float WHEELBASE_LENGTH = 0.420;       // meters (420mm F/B spacing)
const int ENCODER_PPR = 6256;               // 17 PPR × 4 × 92 gear ratio

// Motor control constants
const int PWM_MAX = 255;                    // max PWM (8-bit)
const float MS_TO_SECONDS = 1000.0;         // time conversion
const float MIN_DT = 0.001;                 // min delta time (seconds)
```

**Location:** `src/main.cpp`
```cpp
const float SPEED_MULTIPLIER = 0.8f;        // Max speed scale (0.0-1.0)
const unsigned long CONTROL_INTERVAL = 50;  // Motor control loop (ms)
const unsigned long ODOM_INTERVAL = 50;     // Odometry publish rate (ms)
const unsigned long TIMEOUT_MS = 1000;      // Command timeout (ms)
KinematicsMethod KINEMATICS_METHOD = SIMPLE; // SIMPLE or COMPLEX
```

### Communication Protocol

**Jetson → ESP32 (Twist Commands):**
```
Format: TWIST,linear_x,linear_y,angular_z
Example: TWIST,0.5,-0.2,0.1
```

**ESP32 → Jetson (Odometry @ 20Hz):**
```
Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
Example: ODOM,1.234,0.567,0.785,0.5,0.0,0.1,12450,12398,12467,12410

REP-103 Convention:
  vx = forward velocity (m/s)
  vy = left strafe velocity (m/s)
  omega = CCW rotation (rad/s)
```

## Hardware Specifications

### Robot Dimensions
- **Chassis**: 460×360mm (4040 aluminum extrusion)
- **Wheels**: 152mm Mecanum (14095 Right, 14099 Left)
- **Wheelbase Width**: 468.5mm (360mm base + 2×54.25mm flange/wheel offset)
- **Wheelbase Length**: 420mm (460mm base - 40mm motor offset)

### Motors & Encoders
- **Gear Ratio**: 92:1 (high torque)
- **Rated Torque**: 60 kg·cm @ 80 RPM
- **Max Speed**: ~0.46 m/s (90 RPM no-load)
- **Encoder**: AB quadrature, 17 PPR base × 92 gear = 6256 effective PPR
- **Power**: 24V DC, rated 5A, stall 24.5A

### Pin Assignments

#### Encoders (PCNT Hardware)
| Motor | A | B | Side |
|-------|---|---|------|
| FL (1) | GPIO 32 | GPIO 33 | Left side |
| FR (2) | GPIO 23 | GPIO 22 | Right side |
| BL (3) | GPIO 27 | GPIO 13 | Left side |
| BR (4) | GPIO 18 | GPIO 4 | Right side |

#### Cytron Motor Drivers
| Motor | DIR | PWM | Side |
|-------|-----|-----|------|
| FL | GPIO 25 | GPIO 26 | Left motors |
| FR | GPIO 21 | GPIO 19 | Right motors |
| BL | GPIO 14 | GPIO 12 | Left motors |
| BR | GPIO 17 | GPIO 16 | Right motors |

**Avoid these GPIOs:** 0,2,4,5,6-11,12,15 (boot/flash issues)

## Testing

### Test Encoders Only
```cpp
// Add to main.cpp loop temporarily:
Serial.printf("Enc: %ld,%ld,%ld,%ld\n",
    robot.getEncoders()->getCount(FRONT_LEFT),
    robot.getEncoders()->getCount(FRONT_RIGHT),
    robot.getEncoders()->getCount(BACK_LEFT),
    robot.getEncoders()->getCount(BACK_RIGHT));
```

### Test Movement
```bash
python3 examples/simple_twist_test.py COM3
# 1 = Auto test | 2 = Keyboard control
```

### Movement Patterns (REP-103)
```
Forward:       TWIST,0.5,0.0,0.0   (X+ = forward)
Strafe Left:   TWIST,0.0,0.5,0.0   (Y+ = left)
Strafe Right:  TWIST,0.0,-0.5,0.0  (Y- = right)
Rotate CCW:    TWIST,0.0,0.0,0.5   (Z+ = counter-clockwise)
Rotate CW:     TWIST,0.0,0.0,-0.5  (Z- = clockwise)
```

## ROS2 Integration

### Bridge Node (Jetson)
**File:** `examples/mecanum_bridge_node.py`

```bash
python3 mecanum_bridge_node.py
# Subscribes: /cmd_vel
# Publishes: /odom, /tf (odom→base_link)
```

### Nav2 Config
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    odom_topic: "odom"
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| No encoder counts | Check wiring, power (3.3V/5V), change `NONE` to `UP` in pullup config |
| Wrong direction | Swap A/B channels or negate in code |
| Erratic odometry | Verify PPR (6256), wheel radius, wheelbase dimensions |
| Motors not responding | Check 24V supply, driver wiring, enable pins |

## Calculation Details

### Wheelbase Width
```
360mm base + 2×(26.5mm flange + 27.75mm half-wheel) = 468.5mm
```

### Wheelbase Length
```
460mm base - 20mm front - 20mm back = 420mm
```

### Encoder PPR
```
17 base PPR × 4 (quadrature) × 92 (gear ratio) = 6256
Resolution: ~0.077mm per tick
```

---

**Author:** Achal Patel  
**Updated:** November 2025  
**License:** MIT

## Features

- **4 Motor Control**: Cytron dual motor driver support with DIR/PWM control
- **Quadrature Encoders**: Hardware PCNT-based encoder reading (no CPU cycles wasted)
- **Real-time Odometry**: Position and velocity feedback for ROS2 Nav2 stack
- **Twist Command Input**: Standard ROS2 Twist message format via UART
- **20Hz Control Loop**: Fast, responsive motion control
- **Safety Timeout**: Automatic stop if no commands received

## Robot Specifications

### Mimic Robot Physical Dimensions
- **Chassis**: 460mm (L) × 360mm (W) aluminum extrusion frame (4040 profile)
- **Wheels**: 152mm Mecanum Wheels (76.2mm radius)
  - Part Numbers: 14095 (Right), 14099 (Left)
  - Axia width: 55.5mm
  - Load capacity: 15kg per wheel
- **Wheelbase Width**: 468.5mm (center-to-center between left/right wheels)
  - Calculation: 360mm + 2×(26.5mm flange + 27.75mm half-wheel)
- **Wheelbase Length**: 420mm (center-to-center between front/back wheels)
  - Calculation: 460mm - 40mm (motor mounting offset from 4040 centers)

### Encoder Specifications
- **Type**: AB Dual-phase Incremental Magnetic Hall Encoder
- **Base Pulse**: 17 PPR
- **Gear Ratio**: 92:1
- **Effective PPR**: 6256 (17 × 4 quadrature × 92 gear ratio)
- **Supply Voltage**: 3.3V / 5.0V DC
- **Interface**: PH2.0-4PIN connector
- **Built-in Pull-up**: Yes

### Motor Specifications
- **Gear Ratio**: 92:1
- **No-load Speed**: 90 RPM @ 53.5mm wheel = ~0.5 m/s
- **Rated Speed**: 80 RPM
- **Rated Torque**: 60 kg·cm (5.88 N·m)
- **No-load Current**: ≤0.7A
- **Rated Current**: ≤5.0A
- **Stall Current**: 24.5A
- **Supply Voltage**: 24V DC

## Hardware Requirements

- ESP32 DevKit
- 4x Cytron motor drivers (or compatible DIR/PWM drivers)
- 4x DC motors with AB quadrature encoders (17 PPR base, 92:1 gear ratio)
- Jetson (or any computer running ROS2 Nav2)

## Pin Configuration

### ESP32 DevKit Pin Diagram

```
                                    NORTH
                    ┌───────────────────────────────┐
                    │         Freenove ESP32        │
                    │           WROOM               │
                    │                               │
         LEFT SIDE  │                               │  RIGHT SIDE
                    │                               │
    ┌───────────────┤                               ├───────────────┐
    │               │                               │               │
    │  FL Motor     │                               │     FR Motor  │
    │  (Motor 1)    │                               │    (Motor 2)  │
    │               │                               │               │
    │  Enc A: 32 ───┤                               ├─── 23 :Enc A  │
    │  Enc B: 33 ───┤                               ├─── 22 :Enc B  │
    │  DIR:   25 ───┤                               ├─── 21 :DIR    │
    │  PWM:   26 ───┤                               ├─── 19 :PWM    │
    │               │                               │               │
    │               │                               │               │
    │  BL Motor     │                               │     BR Motor  │
    │  (Motor 3)    │                               │    (Motor 4)  │
    │               │                               │               │
    │  Enc A: 27 ───┤                               ├─── 18 :Enc A  │
    │  Enc B: 14 ───┤                               ├───  4 :Enc B  │
    │  DIR:   13 ───┤                               ├─── 17 :DIR    │
    │  PWM:   12 ───┤                               ├─── 16 :PWM    │
    │               │                               │               │
    └───────────────┤                               ├───────────────┘
                    │                               │
                    │           ▓▓▓▓▓▓              │
                    │         USB-C Port            │
                    │          (SOUTH)              │
                    └───────────────────────────────┘

```

## Communication Protocol

### Incoming (Jetson → ESP32)
**Twist Commands** - Control robot motion
```
Format: TWIST,linear_x,linear_y,angular_z
Example: TWIST,0.5,-0.2,0.1
Rate: As needed (timeout safety at 1 second)
```

### Outgoing (ESP32 → Jetson)
**Odometry Data** - Position and velocity feedback
```
Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
Example: ODOM,1.2345,0.5678,0.7854,0.5,0.0,0.1,12450,12398,12467,12410
Rate: 20Hz (50ms)

Fields (ROS2 REP-103):
  x, y       - Position in meters (global frame)
  theta      - Heading in radians (CCW from X-axis)
  vx         - Forward velocity in m/s (X-axis)
  vy         - Left strafe velocity in m/s (Y-axis)
  omega      - Angular velocity in rad/s (CCW positive)
  enc1-enc4  - Raw encoder counts (FL, FR, BL, BR)
```

## Imp Configuration Parameters

Edit these in `mecanum_drive_controller.cpp` if any value is changed:

```cpp
// Robot physical parameters
const float WHEEL_RADIUS = 0.0762;          // meters (152mm diameter wheels)
const float WHEELBASE_WIDTH = 0.4685;       // meters (468.5mm center-to-center)
const float WHEELBASE_LENGTH = 0.420;       // meters (420mm front-to-back)
const int ENCODER_PPR = 6256;               // pulses per revolution (17×4×92)
   
```

**`src/main.cpp` (lines 7-11):**
```cpp
// Control configuration
const float SPEED_MULTIPLIER = 0.8f;        // Scale commanded velocities
const unsigned long CONTROL_INTERVAL = 50;  // Control loop period (ms) - 20Hz
const unsigned long ODOM_INTERVAL = 50;     // Odometry publish period (ms) - 20Hz
const unsigned long TIMEOUT_MS = 1000;      // Safety timeout (ms)
```

## Testing

### If you want to test Encoders Only
```cpp
// In main.cpp loop, add temporarily:
Serial.printf("Encoders: %ld, %ld, %ld, %ld\n",
    robot.getEncoders()->getCount(FRONT_LEFT),
    robot.getEncoders()->getCount(FRONT_RIGHT),
    robot.getEncoders()->getCount(BACK_LEFT),
    robot.getEncoders()->getCount(BACK_RIGHT));
```


### Erratic Odometry
- Verify `ENCODER_PPR` constant matches your encoder specs (line 23 in mimic_mecanum_base.cpp)
- Check `WHEEL_RADIUS` measurement (line 20)
- Ensure `WHEELBASE_WIDTH` and `WHEELBASE_LENGTH` are accurate (lines 21-22)
- Look for mechanical slippage



## Some more Features



### Changing Kinematics Method
```cpp
// In main.cpp:
KinematicsMethod KINEMATICS_METHOD = SIMPLE;   // or COMPLEX
```

### Odometry Reset
Add to main.cpp to reset odometry on command:
```cpp
// In parseTwistMessage or add new command parser:
if (message.startsWith("RESET")) {
    robot.getEncoders()->reset();
    Serial.println("Odometry reset");
}
```

## Nav2 Configuration

Add to your `nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    odom_topic: "odom"
    
ekf_localization:
  ros__parameters:
    frequency: 20.0
    odom0: odom
    odom0_config: [true, true, false,    # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   true, true, false,     # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```

**Last Updated**: November 2025  
**Author**: Achal Patel

## 🔧 Hardware Configuration

### Pin Selection Make sure you are aware of these
Try to stay away from
**GPIO0, GPIO2, GPIO4, GPIO5, GPIO12, GPIO14, GPIO15, GPIO16, GPIO17, GPIO34–39**
because they are either boot strapping, taken by flash, or input only.
The only things you must avoid are:

- gpio6–11 because they belong to the flash chip

- gpio34–39 because they are input-only and don’t support pullups

- boot-strap pins that get confused during reset if they’re pulled low or high incorrectly

- weird behavior on gpio12 that changes flash voltage at boot

##  Project Architecture

### Core Files Structure
```
mecanum-base/
├── src/
│   ├── main.cpp                      #  ESP32 main program (UART receiver)
│   └── mimic_mecanum_base.cpp        #  Motor drivers & encoder odometry
├── include/
│   └── mimic_mecanum_base.h          #  Class definitions and interfaces
├── examples/
│   ├── simple_twist_test.py          #  No-dependency testing script
│   ├── test_twist_commands.py        #  Advanced testing with keyboard
│   └── motor_test.cpp                #  Individual motor testing
├── platformio.ini                    #  Build configuration
└── README.md                         #  This documentation
```

### Key Software Components

#### 1. **main.cpp** - The Heart of the System
**Purpose**: UART communication receiver with real-time motor control

**Key Features**:
```cpp
// Configurable kinematics method
KinematicsMethod KINEMATICS_METHOD = SIMPLE;  // or COMPLEX

// Safety parameters
const float SPEED_MULTIPLIER = 0.8f;          // Max speed limit
const unsigned long TIMEOUT_MS = 1000;        // Emergency timeout
const unsigned long CONTROL_INTERVAL = 50;    // 20Hz control loop
```

**Message Protocol**:
```
Format: "TWIST,linear_x,linear_y,angular_z\n"
Example: "TWIST,0.5,-0.2,0.1\n"
  - linear_x: Forward/backward (-1.0 to 1.0)
  - linear_y: Strafe left/right (-1.0 to 1.0)  
  - angular_z: Rotation (-1.0 to 1.0)
```

**Driver Types Supported**:
```cpp
enum MotorDriverType {
    BTS7960,        // Current implementation (4 individual drivers)
    CYTRON_DUAL     // Future support (2 dual-channel drivers)
};

enum KinematicsMethod {
    SIMPLE,         // Fast direct mapping (recommended)
    COMPLEX         // Advanced geometric calculations
};
```

**Key Classes**:
- `MotorDriver` - Abstract base class
- `BTS7960Driver` - Current implementation with dual PWM control
- `CytronDriver` - Placeholder for future dual-channel drivers
- `MecanumBase` - Main robot controller with kinematics

#### 3. **mimic_mecanum_base.cpp** - Implementation Details

**Robot Parameters (lines 20-32)**:
```cpp
const float WHEEL_RADIUS = 0.0762;          // meters (152mm wheels)
const float WHEELBASE_WIDTH = 0.4685;       // meters (468.5mm L/R)
const float WHEELBASE_LENGTH = 0.420;       // meters (420mm F/B)
const int ENCODER_PPR = 6256;               // 17×4×92 gear ratio
const int PWM_MAX = 255;                    // 8-bit PWM max
const float MS_TO_SECONDS = 1000.0;         // time conversion
const float MIN_DT = 0.001;                 // min delta time
```

**Motor Direction Correction**:
```cpp
void MecanumBase::move(float x, float y, float rotation) {
            // Right side motors need inversion for proper mecanum movement
            if (i == FRONT_RIGHT || i == BACK_RIGHT) {
                motors[i]->setSpeed(-speeds[i]);  // ⭐ Critical fix
            } else {
                motors[i]->setSpeed(speeds[i]);
            }
        }
```

**Simple Kinematics** (Default - Fast & Reliable):
```cpp
// Direct mapping - perfect for most applications
speeds[FRONT_LEFT]  = y + x + rotation;   // Forward + Strafe + Turn
speeds[FRONT_RIGHT] = y - x - rotation;   // Forward - Strafe - Turn  
speeds[BACK_LEFT]   = y - x + rotation;   // Forward - Strafe + Turn
speeds[BACK_RIGHT]  = y + x - rotation;   // Forward + Strafe - Turn
```

**Complex Kinematics** (Advanced - For Precision Applications):
- Uses trigonometric calculations with theta and power
- Accounts for robot geometry (wheelbase dimensions)
- Includes proper velocity scaling for maximum efficiency

##  Testing & Debugging Tools

### Test Scripts Available

#### 1. **simple_twist_test.py** - Primary Testing Tool

```bash
python3 simple_twist_test.py COM3
# Choose:
# 1 = Automated test sequence (recommended for first test)
# 2 = Interactive keyboard control

# Interactive shortcuts:
# w/s = forward/back    a/d = strafe left/right
# q/e = rotate left/right    x = stop    quit = exit
```


### Kinematics Selection
```cpp
// In main.cpp - choose your calculation method:
KinematicsMethod KINEMATICS_METHOD = SIMPLE;    // Fast, reliable
KinematicsMethod KINEMATICS_METHOD = COMPLEX;   // Advanced, precise
```

### Performance Tuning
```cpp
// Adjust these constants in main.cpp:
const float SPEED_MULTIPLIER = 0.8f;          // Reduce for safety testing
const unsigned long CONTROL_INTERVAL = 50;    // 20Hz (50ms) is optimal
const unsigned long TIMEOUT_MS = 1000;        // Safety timeout duration
```

by: Achal Patel

