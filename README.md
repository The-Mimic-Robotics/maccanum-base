# Mecanum Base Controller with Encoder Odometry

ESP32-based motor controller for 4-wheel mecanum drive robot with quadrature encoder feedback for Nav2 odometry.

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
Example: ODOM,1.234,0.567,0.785,0.1,0.0,0.05,12450,12398,12467,12410
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
| Motor | A | B | Notes |
|-------|---|---|-------|
| FL (1) | GPIO 18 | GPIO 19 | Right side cluster |
| FR (2) | GPIO 21 | GPIO 22 | Right side cluster |
| BL (3) | GPIO 23 | GPIO 25 | Right side cluster |
| BR (4) | GPIO 26 | GPIO 27 | Right side cluster |

#### Cytron Motor Drivers
| Motor | DIR | PWM | Side |
|-------|-----|-----|------|
| FL | GPIO 32 | GPIO 33 | Left motors |
| BL | GPIO 12 | GPIO 13 | Left motors |
| FR | GPIO 14 | GPIO 15 | Right motors |
| BR | GPIO 16 | GPIO 17 | Right motors |

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

### Movement Patterns
```
Forward:  TWIST,0.5,0.0,0.0
Strafe R: TWIST,0.0,-0.5,0.0
Rotate L: TWIST,0.0,0.0,0.5
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
                                  ESP32 DEVKIT (38 pins)
                           ┌────────────────────────────────┐
                           │                                │
                       3V3 │ 1                          38 │ GND
                       GND │ 2                          37 │ GPIO 23  ──→ ENC3_A (BL Motor)
                 GPIO 15 ← │ 3  FR_PWM                  36 │ GPIO 22  ──→ ENC2_B (FR Motor)
                  GPIO 2   │ 4                          35 │ TXD0
                  GPIO 0   │ 5                          34 │ RXD0
                  GPIO 4   │ 6                          33 │ GPIO 21  ──→ ENC2_A (FR Motor)
              FL_PWM → GPIO 16 │ 7                          32 │ GPIO 19  ──→ ENC1_B (FL Motor)
              BR_PWM → GPIO 17 │ 8                          31 │ GPIO 18  ──→ ENC1_A (FL Motor)
                  GPIO 5   │ 9                          30 │ GPIO 5
                  GPIO 18 ─│10  (ENC1_A)                29 │ GPIO 17  ──→ BR_PWM
                  GPIO 19 ─│11  (ENC1_B)                28 │ GPIO 16  ──→ FL_PWM
                  GPIO 21 ─│12  (ENC2_A)                27 │ GPIO 4
                       GND │13                          26 │ GPIO 0
                  GPIO 22 ─│14  (ENC2_B)                25 │ GPIO 2
                  GPIO 23 ─│15  (ENC3_A)                24 │ GPIO 15  ──→ FR_PWM
                       GND │16                          23 │ GND
                       3V3 │17                          22 │ GPIO 13  ──→ BL_PWM
                       EN  │18                          21 │ GPIO 12  ──→ BL_DIR
          SENSOR_VP/GPIO 36│19                          20 │ GPIO 14  ──→ FR_DIR
          SENSOR_VN/GPIO 39│20                          19 │ GPIO 27  ──→ ENC4_B (BR Motor)
                       GND │21                          18 │ GPIO 26  ──→ ENC4_A (BR Motor)
              ENC3_B → GPIO 25 │22                          17 │ GPIO 25  ──→ ENC3_B (BL Motor)
              FL_DIR → GPIO 32 │23                          16 │ GPIO 33  ──→ FL_DIR
              BR_DIR → GPIO 33 │24                          15 │ GPIO 32  ──→ FL_PWM
                       GND │25                          14 │ GND
                           └────────────────────────────────┘
```

### Pin Assignments Summary

#### Encoder Pins (Fixed - using PCNT hardware)
| Motor | Encoder A | Encoder B | Location |
|-------|-----------|-----------|----------|
| **Motor 1 (Front Left)**  | GPIO 18 | GPIO 19 | Right side pins |
| **Motor 2 (Front Right)** | GPIO 21 | GPIO 22 | Right side pins |
| **Motor 3 (Back Left)**   | GPIO 23 | GPIO 25 | Right side pins |
| **Motor 4 (Back Right)**  | GPIO 26 | GPIO 27 | Right side pins |

#### Cytron Motor Driver Pins (Left Side Motors)
| Motor | DIR Pin | PWM Pin | Location |
|-------|---------|---------|----------|
| **Front Left (FL)**  | GPIO 32 | GPIO 33 | Bottom left |
| **Back Left (BL)**   | GPIO 12 | GPIO 13 | Right side |

#### Cytron Motor Driver Pins (Right Side Motors)
| Motor | DIR Pin | PWM Pin | Location |
|-------|---------|---------|----------|
| **Front Right (FR)** | GPIO 14 | GPIO 15 | Right side |
| **Back Right (BR)**  | GPIO 16 | GPIO 17 | Right side |

### Wiring Layout Benefits
- **Encoder pins clustered** on right side (18,19,21,22,23,25,26,27) for clean wiring
- **Left motor controls** (FL, BL) use pins 32,33,12,13
- **Right motor controls** (FR, BR) use pins 14,15,16,17
- Logical grouping reduces wire tangles and makes debugging easier

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
Example: ODOM,1.2345,0.5678,0.7854,0.1,0.0,0.05,12450,12398,12467,12410
Rate: 20Hz (50ms)

Fields:
  x, y       - Position in meters (global frame)
  theta      - Heading in radians
  vx, vy     - Linear velocity in m/s (robot frame: vx=strafe, vy=forward)
  omega      - Angular velocity in rad/s
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

### ESP32 Pin Assignments (Optimized for Stability)
Our pin configuration **avoids problematic boot-strapping pins** that can cause motor twitching:

```cpp
// Front Left Motor (BTS7960)
RPWM: GPIO 16    LPWM: GPIO 17    R_EN: GPIO 21    L_EN: GPIO 19

// Front Right Motor (BTS7960) 
RPWM: GPIO 25    LPWM: GPIO 26    R_EN: GPIO 34    L_EN: GPIO 35

// Back Left Motor (BTS7960)
RPWM: GPIO 22    LPWM: GPIO 23    R_EN: GPIO 5     L_EN: GPIO 18

// Back Right Motor (BTS7960)
RPWM: GPIO 33    LPWM: GPIO 32    R_EN: GPIO 12    L_EN: GPIO 13
```

⚠️ **Critical**: Pins **0, 2, 9, 10, 11, 13, 15** can cause boot issues and motor instability. Our configuration avoids these.


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


##  Movement Mechanics

### Mecanum Wheel Movement Patterns
```
Forward:     linear_x=0.5,  linear_y=0.0,  angular_z=0.0
Backward:    linear_x=-0.5, linear_y=0.0,  angular_z=0.0
Strafe Left: linear_x=0.0,  linear_y=0.5,  angular_z=0.0
Strafe Right:linear_x=0.0,  linear_y=-0.5, angular_z=0.0
Rotate Left: linear_x=0.0,  linear_y=0.0,  angular_z=0.5
Rotate Right:linear_x=0.0,  linear_y=0.0,  angular_z=-0.5

Diagonal:    linear_x=0.5,  linear_y=0.5,  angular_z=0.0
Circle:      linear_x=0.3,  linear_y=0.0,  angular_z=0.3
```

### Safety Features
- **Timeout Protection**: Robot stops if no commands received for 1 second
- **Speed Limiting**: `SPEED_MULTIPLIER` prevents dangerous speeds
- **Watchdog Timer**: Control loop runs at precise 20Hz
- **Emergency Stop**: Send all-zero command to immediate stop

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

## ROS2 Integration

### On Jetson Side

Create a node to bridge UART and ROS2 topics:

```python
#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MecanumBridge(Node):
    def __init__(self):
        super().__init__('mecanum_bridge')
        
        # Serial connection to ESP32
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        
        # ROS2 publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS2 subscribers
        self.twist_sub = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)
        
        # Timer for reading odometry
        self.create_timer(0.05, self.read_odometry)  # 20Hz
        
    def twist_callback(self, msg):
        # Send twist command to ESP32
        cmd = f"TWIST,{msg.linear.x},{msg.linear.y},{msg.angular.z}\n"
        self.ser.write(cmd.encode())
        
    def read_odometry(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line.startswith('ODOM,'):
                self.parse_and_publish_odom(line)
                
    def parse_and_publish_odom(self, line):
        # Parse: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
        parts = line.split(',')
        if len(parts) != 11:
            return
            
        x = float(parts[1])
        y = float(parts[2])
        theta = float(parts[3])
        vx = float(parts[4])
        vy = float(parts[5])
        omega = float(parts[6])
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Velocity (in robot frame for mecanum)
        odom.twist.twist.linear.x = vy  # forward
        odom.twist.twist.linear.y = vx  # strafe
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MecanumBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```


by: Achal Patel

