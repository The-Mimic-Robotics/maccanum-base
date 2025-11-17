#!/usr/bin/env python3
"""
Cytron Motor Driver Test

This script tests the Cytron motor driver implementation.
Each Cytron driver uses 2 pins: DIR (direction) and PWM (speed).
"""

import serial
import time
import sys

def test_cytron_drivers(port='COM3', baud=115200):
    """Test Cytron motor driver implementation"""

    try:
        # Connect to ESP32
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Allow connection to establish

        print(f"🔄 Testing Cytron motor drivers on {port}")
        print("=" * 50)
        print("Expected behavior:")
        print("- Forward commands should spin motors in one direction")
        print("- Reverse commands should spin motors in opposite direction")
        print("- Each motor should respond individually")
        print()

        # Test sequence for Cytron drivers
        tests = [
            ("STOP ALL", "TWIST,0.0,0.0,0.0", "All motors should stop"),

            # Test forward movement (all wheels forward)
            ("FORWARD", "TWIST,0.5,0.0,0.0", "All wheels should spin forward"),

            # Test backward movement (all wheels backward)
            ("BACKWARD", "TWIST,-0.5,0.0,0.0", "All wheels should spin backward"),

            # Test strafe right (left wheels forward, right wheels backward)
            ("STRAFE RIGHT", "TWIST,0.0,-0.5,0.0", "Left wheels forward, right wheels backward"),

            # Test strafe left (left wheels backward, right wheels forward)
            ("STRAFE LEFT", "TWIST,0.0,0.5,0.0", "Left wheels backward, right wheels forward"),

            # Test rotation clockwise (left wheels forward, right wheels backward)
            ("ROTATE CW", "TWIST,0.0,0.0,-0.5", "Left wheels forward, right wheels backward"),

            # Test rotation counter-clockwise (left wheels backward, right wheels forward)
            ("ROTATE CCW", "TWIST,0.0,0.0,0.5", "Left wheels backward, right wheels forward"),

            ("STOP ALL", "TWIST,0.0,0.0,0.0", "All motors should stop")
        ]

        for test_name, command, expected in tests:
            print(f"\n🧪 {test_name}")
            print(f"   Command: {command}")
            print(f"   Expected: {expected}")

            # Send command
            ser.write((command + '\n').encode())

            # Read response if any
            time.sleep(0.1)
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"   ESP32: {response}")

            # Wait for user observation
            input("   Press Enter when you've observed the behavior...")

        print("\n✅ Cytron driver test complete!")
        print("\n🔧 If motors don't respond:")
        print("   - Check DIR and PWM pin connections")
        print("   - Verify Cytron driver power (5V)")
        print("   - Confirm motor power supply (24V)")
        print("   - Test individual motors with simple PWM")

    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print("💡 Make sure ESP32 is connected and port is correct")
        return False
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            # Send stop command before closing
            ser.write("TWIST,0.0,0.0,0.0\n".encode())
            ser.close()

    return True

def main():
    """Main test function"""

    # Get COM port from user if needed
    port = input("Enter ESP32 COM port (default COM3): ").strip()
    if not port:
        port = "COM3"

    print("🤖 CYTRON MOTOR DRIVER TEST")
    print("=" * 50)
    print("This test verifies Cytron motor driver functionality:")
    print("- DIR pin controls direction (HIGH=forward, LOW=reverse)")
    print("- PWM pin controls speed (0-255)")
    print("- Each motor should respond to twist commands")
    print()

    success = test_cytron_drivers(port)

    if success:
        print("\n🎯 If all tests passed, your Cytron drivers are working!")
        print("🚀 Ready for full mecanum robot operation")
    else:
        print("\n❌ Test failed. Check connections and try again.")

if __name__ == "__main__":
    main()