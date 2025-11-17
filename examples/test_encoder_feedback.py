#!/usr/bin/env python3
"""
ESP32 Encoder Feedback Test Script
Tests the odometry data coming from ESP32 encoders

Usage:
    python3 test_encoder_feedback.py COM3         # Windows
    python3 test_encoder_feedback.py /dev/ttyUSB0 # Linux (Jetson)
"""

import serial
import time
import sys

def parse_odom_message(line):
    """
    Parse ODOM message from ESP32
    Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
    """
    try:
        if not line.startswith('ODOM,'):
            return None
        
        parts = line.split(',')
        if len(parts) != 11:
            return None
        
        odom_data = {
            'x': float(parts[1]),
            'y': float(parts[2]),
            'theta': float(parts[3]),
            'vx': float(parts[4]),
            'vy': float(parts[5]),
            'omega': float(parts[6]),
            'enc1': int(parts[7]),
            'enc2': int(parts[8]),
            'enc3': int(parts[9]),
            'enc4': int(parts[10])
        }
        return odom_data
    except Exception as e:
        print(f"Parse error: {e}")
        return None

def send_twist_command(ser, linear_x, linear_y, angular_z):
    """Send TWIST command to ESP32"""
    cmd = f"TWIST,{linear_x},{linear_y},{angular_z}\n"
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_encoder_feedback.py <serial_port>")
        print("Example: python3 test_encoder_feedback.py COM3")
        print("Example: python3 test_encoder_feedback.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print("=" * 60)
    print("ESP32 Encoder Feedback Test")
    print("=" * 60)
    print(f"Connecting to {port} at 115200 baud...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Wait for ESP32 to stabilize
        print("Connected!")
        print()
        
        print("Monitoring odometry data...")
        print("Press Ctrl+C to stop")
        print()
        print("Available commands:")
        print("  1 - Move forward")
        print("  2 - Move backward")
        print("  3 - Strafe right")
        print("  4 - Strafe left")
        print("  5 - Rotate CCW")
        print("  6 - Rotate CW")
        print("  0 - Stop")
        print("  Enter - Just monitor (no commands)")
        print()
        
        # Clear any buffered data
        ser.reset_input_buffer()
        
        last_odom_time = time.time()
        odom_count = 0
        
        while True:
            # Read odometry data
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line.startswith('ODOM,'):
                    odom = parse_odom_message(line)
                    if odom:
                        odom_count += 1
                        current_time = time.time()
                        dt = current_time - last_odom_time
                        
                        # Display odometry data every message
                        print(f"\r[{odom_count:4d}] ", end='')
                        print(f"Pos: ({odom['x']:7.3f}, {odom['y']:7.3f}) m  ", end='')
                        print(f"θ: {odom['theta']:6.3f} rad  ", end='')
                        print(f"Vel: ({odom['vx']:5.2f}, {odom['vy']:5.2f}, ω:{odom['omega']:5.2f})  ", end='')
                        print(f"Enc: [{odom['enc1']:6d}, {odom['enc2']:6d}, {odom['enc3']:6d}, {odom['enc4']:6d}]  ", end='')
                        print(f"dt: {dt*1000:5.1f}ms", end='', flush=True)
                        
                        last_odom_time = current_time
                elif line:
                    # Print other messages on new line
                    print(f"\n{line}")
            
            # Non-blocking check for user input (simple version)
            # For production, use threading or select
            time.sleep(0.01)
    
    except serial.SerialException as e:
        print(f"Error: Could not open port {port}")
        print(f"Details: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nStopping robot...")
        send_twist_command(ser, 0.0, 0.0, 0.0)
        time.sleep(0.1)
        ser.close()
        print("Disconnected.")
    except Exception as e:
        print(f"\nError: {e}")
        ser.close()
        sys.exit(1)

if __name__ == '__main__':
    main()
