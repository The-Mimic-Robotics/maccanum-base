/*
 * Mimic Mecanum Robot Base - Complete Control System
 * 
 * This file provides a comprehensive mecanum wheel robot platform with:
 *   - Motor driver abstraction (BTS7960, Cytron dual-channel support)
 *   - Hardware-accelerated quadrature encoder reading (ESP32 PCNT)
 *   - Real-time odometry calculation and position tracking
 *   - Forward kinematics for motor control (simple & complex modes)
 *   - Inverse kinematics for velocity feedback
 *   - ROS2-compatible odometry message formatting
 * 
 * Hardware: ESP32 DevKit + 4 DC motors with encoders + motor drivers
 * Use case: Autonomous navigation with Nav2 stack integration
 * 
 * Author: Achal Patel (The Mimic Robotics)
 * Date: November 2025
 */

#ifndef MIMIC_MECANUM_BASE_H
#define MIMIC_MECANUM_BASE_H

#include <Arduino.h>
#include <ESP32Encoder.h>

// motor driver types
enum MotorDriverType {
    BTS7960,
    CYTRON_DUAL
};

// kinematics calculation methods
enum KinematicsMethod {
    SIMPLE,
    COMPLEX
};

// base class for motor drivers
class MotorDriver {
public:
    virtual void init() = 0;
    virtual void setSpeed(float speed) = 0;  // speed: -1.0 to 1.0
    virtual void stop() = 0;
    virtual ~MotorDriver() {}
};

// bts7960 motor driver implementation
class BTS7960Driver : public MotorDriver {
private:
    int rpwm_pin;
    int lpwm_pin;
    int r_en_pin;
    int l_en_pin;
    
public:
    BTS7960Driver(int rpwm, int lpwm, int r_en, int l_en);
    void init() override;
    void setSpeed(float speed) override;
    void stop() override;
};

// placeholder for cytron driver (to be implemented when switching)
class CytronDriver : public MotorDriver {
private:
    int dir_pin;
    int pwm_pin;
    
public:
    CytronDriver(int dir, int pwm);
    void init() override;
    void setSpeed(float speed) override;
    void stop() override;
};

// mecanum wheel positions
enum WheelPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

// encoder manager class for all 4 motor encoders
class EncoderManager {
private:
    ESP32Encoder encoders[4];
    long last_counts[4];
    unsigned long last_time;
    float velocities[4];  // rad/s for each wheel
    
    // robot parameters
    float wheel_radius;      // meters
    float wheelbase_width;   // meters (left to right)
    float wheelbase_length;  // meters (front to back)
    int encoder_ppr;         // pulses per revolution
    
    // odometry state
    float pos_x;     // meters
    float pos_y;     // meters
    float theta;     // radians
    
public:
    EncoderManager(float wheel_r = 0.05, float width = 0.3, float length = 0.3, int ppr = 1600);
    void init();
    void update();
    void reset();
    
    // getters
    long getCount(WheelPosition wheel);
    float getVelocity(WheelPosition wheel);  // rad/s
    void getOdometry(float &x, float &y, float &heading);  // get current pose
    void getVelocities(float &vx, float &vy, float &omega);  // get current twist
    
    // formatting for transmission
    String getOdometryString();  // formatted for ROS2 consumption
};

// main mecanum base controller class
class MecanumBase {
private:
    MotorDriver* motors[4];  // array of motor drivers
    MotorDriverType driver_type;
    KinematicsMethod kinematics_method;  // simple or complex calculations
    EncoderManager* encoders;  // encoder manager
    
    // movement calculations
    void calculateWheelSpeeds(float x, float y, float rotation, float speeds[4]);
    void calculateWheelSpeedsSimple(float x, float y, float rotation, float speeds[4]);
    void calculateWheelSpeedsComplex(float x, float y, float rotation, float speeds[4]);
    
public:
    MecanumBase(MotorDriverType type = BTS7960, KinematicsMethod method = SIMPLE);
    ~MecanumBase();
    
    // initialization
    void init();
    
    // encoder access
    EncoderManager* getEncoders();
    
    // kinematics method switching
    void setKinematicsMethod(KinematicsMethod method);
    KinematicsMethod getKinematicsMethod();
    
    // movement control
    void move(float x, float y, float rotation);  // x: strafe, y: forward, rotation: turn
    void stop();
    
    // individual motor control (for testing)
    void setMotorSpeed(WheelPosition wheel, float speed);
    
    // driver switching support
    void switchDriverType(MotorDriverType new_type);
};

#endif