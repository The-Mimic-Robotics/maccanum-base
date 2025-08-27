#!/usr/bin/env python3
"""
Simple ESP32 Twist Test Script (No Dependencies)

This script sends predefined twist commands to the ESP32 for testing.
No external libraries required - just built-in Python modules.

Usage:
python3 simple_twist_test.py [serial_port]
"""

import serial
import time
import sys

def test_esp32_twist(port, baud=115200):
    """Test ESP32 twist command reception"""
    
    print(f"🔌 Connecting to ESP32 on {port}...")
    
    try:
        # Connect to ESP32
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Give ESP32 time to reset
        print("✅ Connected!")
        
        # Test commands: (name, linear_x, linear_y, angular_z, duration)
        test_commands = [
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Forward Slow", 0.3, 0.0, 0.0, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Backward", -0.3, 0.0, 0.0, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Strafe Right", 0.0, -0.3, 0.0, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Strafe Left", 0.0, 0.3, 0.0, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Rotate Clockwise", 0.0, 0.0, 0.3, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Rotate Counter-CW", 0.0, 0.0, -0.3, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Diagonal Forward-Right", 0.3, -0.3, 0.0, 2),
            ("Stop", 0.0, 0.0, 0.0, 1),
            ("Circle Right", 0.2, 0.0, 0.2, 3),
            ("Final Stop", 0.0, 0.0, 0.0, 1),
        ]
        
        print("\n=== STARTING TWIST COMMAND TESTS ===")
        print("Watch your robot and ESP32 serial monitor!")
        print("-" * 50)
        
        for i, (name, x, y, z, duration) in enumerate(test_commands, 1):
            # Create twist command
            command = f"TWIST,{x:.3f},{y:.3f},{z:.3f}\n"
            
            print(f"Test {i:2d}: {name:<20} | x={x:+.1f} y={y:+.1f} z={z:+.1f} | {duration}s")
            
            # Send command
            ser.write(command.encode('utf-8'))
            
            # Wait for duration
            time.sleep(duration)
        
        print("\n✅ All tests completed!")
        
        # Final stop command
        ser.write("TWIST,0.000,0.000,0.000\n".encode('utf-8'))
        time.sleep(0.5)
        
        ser.close()
        print("🔌 Serial connection closed")
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print("Make sure:")
        print("  - ESP32 is connected to the correct port")
        print("  - No other programs are using the serial port")
        print("  - ESP32 is running the twist receiver code")
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.write("TWIST,0.000,0.000,0.000\n".encode('utf-8'))
            ser.close()
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

def interactive_test(port, baud=115200):
    """Interactive command sender"""
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to ESP32 on {port}")
        print("\n=== INTERACTIVE MODE ===")
        print("Enter commands in format: x,y,z (e.g., 0.5,0.0,0.0)")
        print("Or use shortcuts:")
        print("  w = forward    s = back      a = strafe left   d = strafe right")
        print("  q = turn left  e = turn right  x = stop         quit = exit")
        print("-" * 60)
        
        while True:
            cmd = input("Command: ").strip().lower()
            
            if cmd == "quit" or cmd == "exit":
                break
            elif cmd == "w":
                x, y, z = 0.5, 0.0, 0.0
            elif cmd == "s": 
                x, y, z = -0.5, 0.0, 0.0
            elif cmd == "a":
                x, y, z = 0.0, 0.5, 0.0
            elif cmd == "d":
                x, y, z = 0.0, -0.5, 0.0
            elif cmd == "q":
                x, y, z = 0.0, 0.0, 0.5
            elif cmd == "e":
                x, y, z = 0.0, 0.0, -0.5
            elif cmd == "x":
                x, y, z = 0.0, 0.0, 0.0
            else:
                # Try to parse as x,y,z
                try:
                    parts = cmd.split(',')
                    if len(parts) == 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    else:
                        print("❌ Invalid format. Use: x,y,z or shortcuts")
                        continue
                except ValueError:
                    print("❌ Invalid numbers. Use: x,y,z")
                    continue
            
            # Send command
            command = f"TWIST,{x:.3f},{y:.3f},{z:.3f}\n"
            ser.write(command.encode('utf-8'))
            print(f"➤ Sent: forward={x:+.2f}, strafe={y:+.2f}, rotate={z:+.2f}")
        
        # Send final stop
        ser.write("TWIST,0.000,0.000,0.000\n".encode('utf-8'))
        ser.close()
        print("✅ Connection closed")
        
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    # Get serial port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        import platform
        if platform.system() == "Windows":
            port = "COM3"
        else:
            port = "/dev/ttyUSB0"
    
    print("=== ESP32 Twist Command Tester ===")
    print(f"Serial port: {port}")
    print("\nChoose mode:")
    print("1. Automated test sequence")
    print("2. Interactive command mode")
    
    try:
        choice = input("Enter choice (1 or 2): ").strip()
        
        if choice == "1":
            test_esp32_twist(port)
        elif choice == "2":
            interactive_test(port)
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")

if __name__ == "__main__":
    main()
