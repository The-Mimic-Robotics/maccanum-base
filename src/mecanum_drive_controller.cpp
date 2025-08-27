#include "mecanum_drive_controller.h"
#include <math.h>

// pin definitions for esp32 devkit
// front left motor (motor 1)
#define FL_RPWM 22
#define FL_LPWM 23
#define FL_R_EN 21
#define FL_L_EN 19

// front right motor (motor 2)
#define FR_RPWM 18
#define FR_LPWM 5
#define FR_R_EN 17
#define FR_L_EN 16

// back left motor (motor 3)
#define BL_RPWM 4
#define BL_LPWM 0
#define BL_R_EN 2
#define BL_L_EN 15

// back right motor (motor 4)
#define BR_RPWM 13
#define BR_LPWM 12
#define BR_R_EN 14
#define BR_L_EN 27

// pwm settings
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8

// ================= BTS7960 Driver Implementation =================

BTS7960Driver::BTS7960Driver(int rpwm, int lpwm, int r_en, int l_en) 
    : rpwm_pin(rpwm), lpwm_pin(lpwm), r_en_pin(r_en), l_en_pin(l_en) {
}

void BTS7960Driver::init() {
    // set pin modes
    pinMode(rpwm_pin, OUTPUT);
    pinMode(lpwm_pin, OUTPUT);
    pinMode(r_en_pin, OUTPUT);
    pinMode(l_en_pin, OUTPUT);
    
    // enable the driver
    digitalWrite(r_en_pin, HIGH);
    digitalWrite(l_en_pin, HIGH);
    
    // stop motor initially
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, 0);
}

void BTS7960Driver::setSpeed(float speed) {
    // clamp speed to valid range
    speed = constrain(speed, -1.0f, 1.0f);
    
    int pwm_value = abs(speed) * 255;
    
    if (speed > 0) {
        // forward direction
        analogWrite(rpwm_pin, pwm_value);
        analogWrite(lpwm_pin, 0);
    } else if (speed < 0) {
        // reverse direction
        analogWrite(rpwm_pin, 0);
        analogWrite(lpwm_pin, pwm_value);
    } else {
        // stop
        analogWrite(rpwm_pin, 0);
        analogWrite(lpwm_pin, 0);
    }
}

void BTS7960Driver::stop() {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, 0);
}

// ================= Cytron Driver Implementation (placeholder) =================

CytronDriver::CytronDriver(int dir, int pwm) : dir_pin(dir), pwm_pin(pwm) {
}

void CytronDriver::init() {
    // placeholder - implement when switching to cytron drivers
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, 0);
}

void CytronDriver::setSpeed(float speed) {
    // placeholder - implement when switching to cytron drivers
    speed = constrain(speed, -1.0f, 1.0f);
    
    if (speed >= 0) {
        digitalWrite(dir_pin, HIGH);
        analogWrite(pwm_pin, speed * 255);
    } else {
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, abs(speed) * 255);
    }
}

void CytronDriver::stop() {
    analogWrite(pwm_pin, 0);
}

// ================= MecanumBase Implementation =================

MecanumBase::MecanumBase(MotorDriverType type, KinematicsMethod method) 
    : driver_type(type), kinematics_method(method) {
    // initialize motor driver pointers to null
    for (int i = 0; i < 4; i++) {
        motors[i] = nullptr;
    }
}

MecanumBase::~MecanumBase() {
    // clean up motor drivers
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            delete motors[i];
        }
    }
}

void MecanumBase::init() {
    Serial.println("initializing mecanum base...");
    
    // create motor drivers based on type
    if (driver_type == BTS7960) {
        motors[FRONT_LEFT] = new BTS7960Driver(FL_RPWM, FL_LPWM, FL_R_EN, FL_L_EN);
        motors[FRONT_RIGHT] = new BTS7960Driver(FR_RPWM, FR_LPWM, FR_R_EN, FR_L_EN);
        motors[BACK_LEFT] = new BTS7960Driver(BL_RPWM, BL_LPWM, BL_R_EN, BL_L_EN);
        motors[BACK_RIGHT] = new BTS7960Driver(BR_RPWM, BR_LPWM, BR_R_EN, BR_L_EN);
        Serial.println("using bts7960 motor drivers");
    } else if (driver_type == CYTRON_DUAL) {
        // placeholder pins for cytron drivers - adjust when switching
        motors[FRONT_LEFT] = new CytronDriver(22, 23);
        motors[FRONT_RIGHT] = new CytronDriver(18, 5);
        motors[BACK_LEFT] = new CytronDriver(4, 0);
        motors[BACK_RIGHT] = new CytronDriver(13, 12);
        Serial.println("using cytron dual motor drivers");
    }
    
    // initialize all motors
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            motors[i]->init();
        }
    }
    
    Serial.println("mecanum base initialized");
}

void MecanumBase::calculateWheelSpeeds(float x, float y, float rotation, float speeds[4]) {
    // choose calculation method based on kinematics_method setting
    if (kinematics_method == SIMPLE) {
        calculateWheelSpeedsSimple(x, y, rotation, speeds);
    } else {
        calculateWheelSpeedsComplex(x, y, rotation, speeds);
    }
}

void MecanumBase::calculateWheelSpeedsSimple(float x, float y, float rotation, float speeds[4]) {
    // simple mecanum kinematics - fast and easy to understand
    // positive x = strafe right
    // positive y = forward  
    // positive rotation = clockwise
    
    speeds[FRONT_LEFT] = y + x + rotation;
    speeds[FRONT_RIGHT] = y - x - rotation;
    speeds[BACK_LEFT] = y - x + rotation;
    speeds[BACK_RIGHT] = y + x - rotation;
    
    // normalize speeds
    float max_speed = 0;
    for (int i = 0; i < 4; i++) {
        max_speed = max(max_speed, abs(speeds[i]));
    }
    
    if (max_speed > 1.0f) {
        for (int i = 0; i < 4; i++) {
            speeds[i] /= max_speed;
        }
    }
}

void MecanumBase::calculateWheelSpeedsComplex(float x, float y, float rotation, float speeds[4]) {
    // complex mecanum kinematics with theta and power calculations
    
    // calculate movement vector magnitude (power) and angle (theta)
    float power = sqrt(x * x + y * y);
    float theta = atan2(y, x);  // movement angle in radians
    
    // robot geometry parameters (adjust for your robot dimensions)
    const float L = 0.3f;  // half wheelbase length (front to back)
    const float W = 0.3f;  // half wheelbase width (left to right)
    
    // mecanum wheel equations using theta and power
    // sin and cos calculations for 45-degree roller angles
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    float sin_theta_plus_45 = sin(theta + M_PI/4);  // theta + 45°
    float sin_theta_minus_45 = sin(theta - M_PI/4); // theta - 45°
    
    // calculate wheel speeds using proper mecanum kinematics
    // front left wheel (roller at +45°)
    speeds[FRONT_LEFT] = power * sin_theta_plus_45 + rotation * (L + W);
    
    // front right wheel (roller at -45°)
    speeds[FRONT_RIGHT] = power * sin_theta_minus_45 - rotation * (L + W);
    
    // back left wheel (roller at -45°) 
    speeds[BACK_LEFT] = power * sin_theta_minus_45 + rotation * (L + W);
    
    // back right wheel (roller at +45°)
    speeds[BACK_RIGHT] = power * sin_theta_plus_45 - rotation * (L + W);
    
    // normalize speeds to prevent any wheel from exceeding maximum speed
    float max_speed = 0;
    for (int i = 0; i < 4; i++) {
        max_speed = max(max_speed, abs(speeds[i]));
    }
    
    if (max_speed > 1.0f) {
        for (int i = 0; i < 4; i++) {
            speeds[i] /= max_speed;
        }
    }
}

void MecanumBase::setKinematicsMethod(KinematicsMethod method) {
    kinematics_method = method;
    Serial.print("kinematics method changed to: ");
    if (method == SIMPLE) {
        Serial.println("simple");
    } else {
        Serial.println("complex (theta/power)");
    }
}

KinematicsMethod MecanumBase::getKinematicsMethod() {
    return kinematics_method;
}

void MecanumBase::move(float x, float y, float rotation) {
    float speeds[4];
    calculateWheelSpeeds(x, y, rotation, speeds);
    
    // apply speeds to motors
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            motors[i]->setSpeed(speeds[i]);
        }
    }
}

void MecanumBase::stop() {
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            motors[i]->stop();
        }
    }
}

void MecanumBase::setMotorSpeed(WheelPosition wheel, float speed) {
    if (motors[wheel] != nullptr) {
        motors[wheel]->setSpeed(speed);
    }
}

void MecanumBase::switchDriverType(MotorDriverType new_type) {
    // stop all motors first
    stop();
    
    // delete existing drivers
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            delete motors[i];
            motors[i] = nullptr;
        }
    }
    
    // update driver type and reinitialize
    driver_type = new_type;
    init();
}