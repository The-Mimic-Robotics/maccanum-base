# Mecanum Robot Base

ESP32-based mecanum wheel robot controller with Xbox controller support via Bluetooth.

## Hardware Setup

### Motor Drivers
- Currently using 4x BTS 7960 motor drivers
- Future migration planned to 2x Cytron dual-channel motor drivers
- Connected to 24V DC motors with mecanum wheels

### Pin Configuration (ESP32 DevKit)

**Front Left Motor (BTS7960)**
- RPWM: GPIO 22
- LPWM: GPIO 23  
- R_EN: GPIO 21
- L_EN: GPIO 19

**Front Right Motor (BTS7960)**
- RPWM: GPIO 18
- LPWM: GPIO 5
- R_EN: GPIO 17
- L_EN: GPIO 16

**Back Left Motor (BTS7960)**
- RPWM: GPIO 4
- LPWM: GPIO 0
- R_EN: GPIO 2
- L_EN: GPIO 15

**Back Right Motor (BTS7960)**
- RPWM: GPIO 13
- LPWM: GPIO 12
- R_EN: GPIO 14
- L_EN: GPIO 27

## Software Architecture

### Class Structure

**MecanumBase** - Main robot controller class
- Manages all 4 motors through driver abstraction
- Implements mecanum wheel kinematics
- Provides high-level movement commands (x, y, rotation)
- Supports driver switching between BTS7960 and Cytron

**MotorDriver (Abstract Base Class)**
- Common interface for different motor driver types
- Methods: init(), setSpeed(), stop()

**BTS7960Driver** - BTS7960 motor driver implementation
- Handles dual PWM pins and enable pins
- Speed control from -1.0 to 1.0

**CytronDriver** - Placeholder for future Cytron driver implementation

**XboxController** - Simple Bluetooth Xbox controller interface
- Uses BLEController from BLE-Gamepad-Client library directly
- No wrapper classes - straightforward API usage
- Left stick: strafe (X) and forward/back (Y)
- Right stick: rotation (X)

### File Structure

```
include/
└── mecanum_drive_controller.h  # motor drivers and mecanum base classes

src/
├── main.cpp                    # main program with direct controller integration
└── mecanum_drive_controller.cpp # motor driver and base implementations

examples/
└── motor_test.cpp              # individual motor and movement testing
```

## Usage

### Basic Movement Control
```cpp
MecanumBase robot(BTS7960);
robot.init();

// move forward at 50% speed
robot.move(0, 0.5f, 0);

// strafe right at 50% speed  
robot.move(0.5f, 0, 0);

// rotate clockwise at 50% speed
robot.move(0, 0, 0.5f);

// diagonal movement with rotation
robot.move(0.5f, 0.5f, 0.2f);
```

### Xbox Controller Integration
The main program uses the simple BLE-Gamepad-Client API directly:
```cpp
BLEController controller;
controller.begin();

if (controller.isConnected()) {
    BLEControlsEvent e;
    controller.readControls(e);
    
    float strafe = e.leftStickX;    // left stick x
    float forward = e.leftStickY;   // left stick y  
    float rotation = e.rightStickX; // right stick x
    
    robot.move(strafe, forward, rotation);
}
```

### Motor Driver Switching
To switch from BTS7960 to Cytron drivers:
1. Uncomment Cytron library in platformio.ini
2. Update pin definitions in mecanum_drive_controller.cpp
3. Complete CytronDriver implementation
4. Change driver type: `MecanumBase robot(CYTRON_DUAL);`

## Future ROS2 Integration
The current Xbox controller input can be easily replaced with ROS2 twist messages:
- Subscribe to geometry_msgs/Twist
- Extract linear.x, linear.y, angular.z
- Call `robot.move(twist.linear.y, twist.linear.x, twist.angular.z)`

## Libraries Used
- tbekas/BLE-Gamepad-Client@^0.4.0 (Xbox controller)
- cytrontechnologies/Cytron Motor Drivers Library@^1.0.1 (future use)