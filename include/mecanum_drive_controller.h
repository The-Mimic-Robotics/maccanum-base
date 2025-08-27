#ifndef MECANUM_DRIVE_CONTROLLER_H
#define MECANUM_DRIVE_CONTROLLER_H

#include <Arduino.h>

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

// main mecanum base controller class
class MecanumBase {
private:
    MotorDriver* motors[4];  // array of motor drivers
    MotorDriverType driver_type;
    KinematicsMethod kinematics_method;  // simple or complex calculations
    
    // movement calculations
    void calculateWheelSpeeds(float x, float y, float rotation, float speeds[4]);
    void calculateWheelSpeedsSimple(float x, float y, float rotation, float speeds[4]);
    void calculateWheelSpeedsComplex(float x, float y, float rotation, float speeds[4]);
    
public:
    MecanumBase(MotorDriverType type = BTS7960, KinematicsMethod method = SIMPLE);
    ~MecanumBase();
    
    // initialization
    void init();
    
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