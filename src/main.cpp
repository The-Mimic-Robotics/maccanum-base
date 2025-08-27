#include <Arduino.h>
#include "mecanum_drive_controller.h"

// ================= CONTROL VARIABLES =================
KinematicsMethod KINEMATICS_METHOD = SIMPLE;  // SIMPLE or COMPLEX

// global objects
MecanumBase robot(BTS7960, KINEMATICS_METHOD);
String inputBuffer = "";

// twist message structure
struct TwistMsg {
    float linear_x;   // forward/backward
    float linear_y;   // strafe left/right  
    float angular_z;  // rotation
};

TwistMsg currentTwist = {0.0, 0.0, 0.0};

// control parameters
const float SPEED_MULTIPLIER = 0.8f;
const unsigned long CONTROL_INTERVAL = 50;  // 20Hz control loop
const unsigned long TIMEOUT_MS = 1000;      // stop if no commands for 1 second
unsigned long last_control_time = 0;
unsigned long last_command_time = 0;

bool parseTwistMessage(String message, TwistMsg &twist) {
    // Expected format: "TWIST,linear_x,linear_y,angular_z"
    // Example: "TWIST,0.5,-0.2,0.1"
    
    if (!message.startsWith("TWIST,")) {
        return false;
    }
    
    // Remove "TWIST," prefix
    message = message.substring(6);
    
    int firstComma = message.indexOf(',');
    int secondComma = message.indexOf(',', firstComma + 1);
    
    if (firstComma == -1 || secondComma == -1) {
        return false;
    }
    
    twist.linear_x = message.substring(0, firstComma).toFloat();
    twist.linear_y = message.substring(firstComma + 1, secondComma).toFloat();
    twist.angular_z = message.substring(secondComma + 1).toFloat();
    
    return true;
}

void setup() {
    Serial.begin(115200);  // USB communication with Jetson
    Serial.println("starting mecanum robot with twist control...");
    
    Serial.print("using kinematics method: ");
    if (KINEMATICS_METHOD == SIMPLE) {
        Serial.println("simple");
    } else {
        Serial.println("complex (theta/power)");
    }
    
    // initialize robot base
    robot.init();
    
    Serial.println("robot ready - waiting for twist messages via UART");
    Serial.println("expected format: TWIST,linear_x,linear_y,angular_z");
    
    last_command_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    
    // read incoming serial data
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        
        if (inChar == '\n' || inChar == '\r') {
            // end of message - parse it
            if (inputBuffer.length() > 0) {
                TwistMsg newTwist;
                if (parseTwistMessage(inputBuffer, newTwist)) {
                    currentTwist = newTwist;
                    last_command_time = current_time;
                    
                    // debug output
                    Serial.printf("received: x=%.3f, y=%.3f, rot=%.3f\n", 
                                currentTwist.linear_x, currentTwist.linear_y, currentTwist.angular_z);
                } else {
                    Serial.println("invalid twist message format");
                }
                inputBuffer = "";
            }
        } else {
            inputBuffer += inChar;
        }
    }
    
    // control loop at fixed interval
    if (current_time - last_control_time >= CONTROL_INTERVAL) {
        last_control_time = current_time;
        
        // check for timeout
        if (current_time - last_command_time > TIMEOUT_MS) {
            // no commands received - stop robot for safety
            robot.stop();
            
            static unsigned long last_timeout_msg = 0;
            if (current_time - last_timeout_msg > 2000) {
                Serial.println("timeout - no twist commands received");
                last_timeout_msg = current_time;
            }
        } else {
            // apply twist commands to robot
            float strafe = currentTwist.linear_y * SPEED_MULTIPLIER;    // ROS Y = robot strafe
            float forward = currentTwist.linear_x * SPEED_MULTIPLIER;   // ROS X = robot forward
            float rotation = currentTwist.angular_z * SPEED_MULTIPLIER; // ROS Z = robot rotation
            
            robot.move(strafe, forward, rotation);
        }
    }
    
    delay(5);  // small delay
}