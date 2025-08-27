#!/usr/bin/env python3
"""
ESP32 Twist Command Test Script

This script sends twist commands directly to the ESP32 via serial
for testing without needing ROS2 setup.

Usage:
python3 test_twist_commands.py [serial_port]

Default serial port: /dev/ttyUSB0 (Linux) or COM3 (Windows)
"""

import serial
import time
import sys
import threading
import keyboard  # pip install keyboard

class ESP32TwistTester:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        
        # Current twist values
        self.linear_x = 0.0    # forward/backward
        self.linear_y = 0.0    # strafe left/right
        self.angular_z = 0.0   # rotation
        
        # Control settings
        self.speed_step = 0.1
        self.max_speed = 1.0
        
    def connect(self):
        """Connect to ESP32"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Give ESP32 time to reset
            print(f"✅ Connected to ESP32 on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to ESP32: {e}")
            return False
    
    def send_twist(self, linear_x, linear_y, angular_z):
        """Send twist command to ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
            
        # Format: TWIST,linear_x,linear_y,angular_z
        command = f"TWIST,{linear_x:.3f},{linear_y:.3f},{angular_z:.3f}\n"
        
        try:
            self.serial_conn.write(command.encode('utf-8'))
            return True
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            return False
    
    def update_twist(self):
        """Update current twist values and send to ESP32"""
        self.send_twist(self.linear_x, self.linear_y, self.angular_z)
        
        # Display current values
        print(f"\rForward: {self.linear_x:+.2f} | Strafe: {self.linear_y:+.2f} | Rotate: {self.angular_z:+.2f}", end="", flush=True)
    
    def keyboard_control(self):
        """Handle keyboard input for manual control"""
        print("\n=== KEYBOARD CONTROLS ===")
        print("Movement:")
        print("  W/S - Forward/Backward")
        print("  A/D - Strafe Left/Right") 
        print("  Q/E - Rotate Left/Right")
        print("  SPACE - Stop all movement")
        print("  ESC - Exit")
        print("\nPress keys to control robot...")
        print("-" * 50)
        
        while self.running:
            try:
                if keyboard.is_pressed('w'):  # Forward
                    self.linear_x = min(self.linear_x + self.speed_step, self.max_speed)
                elif keyboard.is_pressed('s'):  # Backward
                    self.linear_x = max(self.linear_x - self.speed_step, -self.max_speed)
                else:
                    self.linear_x *= 0.9  # Gradual stop
                    
                if keyboard.is_pressed('a'):  # Strafe left
                    self.linear_y = min(self.linear_y + self.speed_step, self.max_speed)
                elif keyboard.is_pressed('d'):  # Strafe right
                    self.linear_y = max(self.linear_y - self.speed_step, -self.max_speed)
                else:
                    self.linear_y *= 0.9  # Gradual stop
                    
                if keyboard.is_pressed('q'):  # Rotate left
                    self.angular_z = min(self.angular_z + self.speed_step, self.max_speed)
                elif keyboard.is_pressed('e'):  # Rotate right
                    self.angular_z = max(self.angular_z - self.speed_step, -self.max_speed)
                else:
                    self.angular_z *= 0.9  # Gradual stop
                    
                if keyboard.is_pressed('space'):  # Emergency stop
                    self.linear_x = 0.0
                    self.linear_y = 0.0
                    self.angular_z = 0.0
                    
                if keyboard.is_pressed('esc'):  # Exit
                    break
                
                # Clean up small values
                if abs(self.linear_x) < 0.01: self.linear_x = 0.0
                if abs(self.linear_y) < 0.01: self.linear_y = 0.0
                if abs(self.angular_z) < 0.01: self.angular_z = 0.0
                
                self.update_twist()
                time.sleep(0.05)  # 20Hz update rate
                
            except KeyboardInterrupt:
                break
    
    def run_predefined_tests(self):
        """Run a series of predefined movement tests"""
        tests = [
            ("Forward", 0.5, 0.0, 0.0),
            ("Backward", -0.5, 0.0, 0.0),
            ("Strafe Right", 0.0, -0.5, 0.0),
            ("Strafe Left", 0.0, 0.5, 0.0),
            ("Rotate Clockwise", 0.0, 0.0, 0.5),
            ("Rotate Counter-clockwise", 0.0, 0.0, -0.5),
            ("Diagonal Forward-Right", 0.5, -0.5, 0.0),
            ("Circle Right", 0.3, 0.0, 0.3),
            ("Stop", 0.0, 0.0, 0.0),
        ]
        
        print("\n=== RUNNING PREDEFINED TESTS ===")
        
        for name, x, y, z in tests:
            print(f"\n🔄 {name}: forward={x:.1f}, strafe={y:.1f}, rotate={z:.1f}")
            self.send_twist(x, y, z)
            time.sleep(2)  # Run each test for 2 seconds
        
        print("\n✅ All tests completed!")
    
    def run(self):
        """Main run function"""
        if not self.connect():
            return
            
        self.running = True
        
        try:
            print("\nChoose test mode:")
            print("1. Keyboard control (interactive)")
            print("2. Predefined movement tests")
            choice = input("Enter choice (1 or 2): ").strip()
            
            if choice == "1":
                self.keyboard_control()
            elif choice == "2":
                self.run_predefined_tests()
            else:
                print("Invalid choice")
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
        finally:
            # Send stop command
            self.send_twist(0.0, 0.0, 0.0)
            print("\n🛑 Sent stop command to robot")
            
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print("✅ Serial connection closed")

def main():
    # Default serial ports
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        # Auto-detect common ports
        import platform
        if platform.system() == "Windows":
            port = "COM3"  # Common Windows port
        else:
            port = "/dev/ttyUSB0"  # Common Linux port
    
    print("=== ESP32 Twist Command Tester ===")
    print(f"Using serial port: {port}")
    print("Make sure your ESP32 is connected and running the twist receiver code!")
    
    tester = ESP32TwistTester(port)
    tester.run()

if __name__ == "__main__":
    main()
