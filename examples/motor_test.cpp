#include <Arduino.h>
#include "mecanum_drive_controller.h"

// motor test example
MecanumBase robot(BTS7960);

void setup() {
    Serial.begin(115200);
    Serial.println("motor test starting...");
    
    robot.init();
    delay(2000);
}

void loop() {
    Serial.println("testing individual motors...");
    
    // test front left motor
    Serial.println("front left forward");
    robot.setMotorSpeed(FRONT_LEFT, 0.5f);
    delay(2000);
    robot.stop();
    delay(1000);
    
    // test front right motor
    Serial.println("front right forward");
    robot.setMotorSpeed(FRONT_RIGHT, 0.5f);
    delay(2000);
    robot.stop();
    delay(1000);
    
    // test back left motor
    Serial.println("back left forward");
    robot.setMotorSpeed(BACK_LEFT, 0.5f);
    delay(2000);
    robot.stop();
    delay(1000);
    
    // test back right motor
    Serial.println("back right forward");
    robot.setMotorSpeed(BACK_RIGHT, 0.5f);
    delay(2000);
    robot.stop();
    delay(1000);
    
    // test movement patterns
    Serial.println("testing movement patterns...");
    
    // forward
    Serial.println("moving forward");
    robot.move(0, 0.5f, 0);
    delay(2000);
    
    // backward
    Serial.println("moving backward");
    robot.move(0, -0.5f, 0);
    delay(2000);
    
    // strafe right
    Serial.println("strafing right");
    robot.move(0.5f, 0, 0);
    delay(2000);
    
    // strafe left
    Serial.println("strafing left");
    robot.move(-0.5f, 0, 0);
    delay(2000);
    
    // rotate clockwise
    Serial.println("rotating clockwise");
    robot.move(0, 0, 0.5f);
    delay(2000);
    
    // rotate counter-clockwise
    Serial.println("rotating counter-clockwise");
    robot.move(0, 0, -0.5f);
    delay(2000);
    
    robot.stop();
    delay(3000);
}