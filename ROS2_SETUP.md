# Mecanum Robot with ROS2 Twist Control

## Architecture Overview

```
Xbox Controller → Jetson Nano (ROS2) → ESP32 (UART) → Mecanum Motors
```

## Setup Instructions

### 1. ESP32 Setup
- Upload the main.cpp code to your ESP32
- Connect ESP32 to Jetson via USB
- ESP32 will appear as `/dev/ttyUSB0` (or similar)

### 2. Jetson Nano Setup

#### Install Required ROS2 Packages:
```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

#### Copy Python Script:
```bash
# Copy twist_to_serial.py to your ROS2 workspace
cp twist_to_serial.py ~/ros2_ws/src/your_package/scripts/
chmod +x ~/ros2_ws/src/your_package/scripts/twist_to_serial.py
```

#### Find Your Devices:
```bash
# Find Xbox controller
ls /dev/input/js*

# Find ESP32 port  
ls /dev/ttyUSB*
```

### 3. Usage

#### Method 1: Manual Launch (Simple)
```bash
# Terminal 1: Xbox controller input
ros2 run joy joy_node --ros-args -p device:=/dev/input/js0

# Terminal 2: Convert joy to twist
ros2 run teleop_twist_joy teleop_node --ros-args -p axis_linear.x:=1 -p axis_linear.y:=0 -p axis_angular.yaw:=3 -p enable_button:=4

# Terminal 3: Send to ESP32
python3 twist_to_serial.py
```

#### Method 2: Launch File (Recommended)
```bash
# Copy launch file to your package
cp mecanum_launch.py ~/ros2_ws/src/your_package/launch/

# Build workspace
cd ~/ros2_ws && colcon build

# Launch everything
ros2 launch your_package mecanum_launch.py joy_device:=/dev/input/js0 esp32_port:=/dev/ttyUSB0
```

## Controller Mapping

| Xbox Control | Function |
|--------------|----------|
| Left Stick Up/Down | Forward/Backward |
| Left Stick Left/Right | Strafe Left/Right |
| Right Stick Left/Right | Rotate Left/Right |
| LB Button | Enable movement |
| RB Button | Turbo mode |

## Serial Protocol

**Format:** `TWIST,linear_x,linear_y,angular_z\n`
**Example:** `TWIST,0.500,-0.200,0.100\n`

- `linear_x`: Forward/backward speed (-1.0 to 1.0)
- `linear_y`: Strafe speed (-1.0 to 1.0)  
- `angular_z`: Rotation speed (-1.0 to 1.0)

## Troubleshooting

### ESP32 Not Receiving Commands:
```bash
# Check if ESP32 is connected
ls /dev/ttyUSB*

# Monitor ESP32 output
screen /dev/ttyUSB0 115200

# Test manual commands
echo "TWIST,0.5,0.0,0.0" > /dev/ttyUSB0
```

### Xbox Controller Issues:
```bash
# Test controller input
ros2 topic echo /joy

# Check device permissions
sudo chmod 666 /dev/input/js0
```

### Python Script Issues:
```bash
# Install required Python packages
pip3 install pyserial rclpy

# Check serial port permissions
sudo usermod -a -G dialout $USER
# (logout and login after this)
```

## Advantages of This Approach

1. **Simple ESP32 Code** - Just serial communication
2. **Flexible** - Easy to modify Python script
3. **Reliable** - UART is very stable
4. **Debuggable** - Can monitor serial traffic
5. **Reusable** - Works with any ROS2 twist source
6. **Fast** - Low latency communication

## Future Extensions

- Add autonomous navigation (just publish to `/cmd_vel`)
- Add safety features (emergency stop, obstacle avoidance)
- Log movement data
- Add web interface for remote control
