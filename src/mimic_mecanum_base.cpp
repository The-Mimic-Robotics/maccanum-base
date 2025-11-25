/*
 * Mimic Mecanum Robot Base - Implementation
 * 
 * Complete implementation of mecanum drive system including:
 *   - Motor driver control (BTS7960 & Cytron)
 *   - Quadrature encoder reading using ESP32 PCNT hardware
 *   - Odometry calculation (position & velocity estimation)
 *   - Mecanum wheel kinematics (forward & inverse)
 * 
 * Pin assignments optimized for clean wiring:
 *   - Encoders: GPIO 18,19,21,22,23,25,26,27 (right side cluster)
 *   - Motors: GPIO 12-17,32-33 (logical left/right grouping)
 * 
 * Author: Achal Patel (The Mimic Robotics)
 */

#include "mimic_mecanum_base.h"
#include <math.h>

// ================= ROBOT PHYSICAL PARAMETERS =================

const float WHEEL_RADIUS = 0.0762;          // meters (152mm diameter wheels) - (76.2mm radius)
const float WHEELBASE_WIDTH = 0.4685;       // meters - 360mm base width + 2×(26.5mm flange + 27.75mm half-wheel) = 468.5mm wheelbase width
const float WHEELBASE_LENGTH = 0.420;       // meters - 460mm base length - 40mm motor offset = 420mm wheelbase length
const int ENCODER_PPR = 6256;               // pulses per revolution (17 PPR * 92:1 gear ratio * 4 edges = 6256 PPR)


// ================= PIN DEFINITIONS FOR ESP32 DEVKIT =================
// ENCODER PINS (using PCNT hardware - these are fixed)
// Motor 1 - Front Left
#define ENC1_A 18
#define ENC1_B 19

// Motor 2 - Front Right  
#define ENC2_A 21
#define ENC2_B 22

// Motor 3 - Back Left
#define ENC3_A 23
#define ENC3_B 25

// Motor 4 - Back Right
#define ENC4_A 26
#define ENC4_B 27

// ================= CYTRON DRIVER PINS =================
// New pins chosen to avoid encoder pins and provide good wiring layout
// Left side motors (FL, BL) - grouped together
#define FL_DIR 32   // Front Left Direction
#define FL_PWM 33   // Front Left PWM

#define BL_DIR 12   // Back Left Direction  
#define BL_PWM 13   // Back Left PWM

// Right side motors (FR, BR) - grouped together
#define FR_DIR 14   // Front Right Direction
#define FR_PWM 15   // Front Right PWM

#define BR_DIR 16   // Back Right Direction
#define BR_PWM 17   // Back Right PWM

// ================= BTS7960 DRIVER PINS (Legacy - keep for compatibility) =================
#define FL_RPWM 16
#define FL_LPWM 17
#define FL_R_EN 21
#define FL_L_EN 19

// front right motor (motor 2) - KEEP THESE (they're good)
#define FR_RPWM 25
#define FR_LPWM 26
#define FR_R_EN 34
#define FR_L_EN 35

// back left motor (motor 3) - FIXED PINS
#define BL_RPWM 22  // Was 4 - now using safe GPIO
#define BL_LPWM 23 // Was 0 - now using safe GPIO
#define BL_R_EN 5   // Was 2 - now using safe GPIO
#define BL_L_EN 18  // Was 15 - now using safe GPIO

// back right motor (motor 4) - FIXED PINS
#define BR_RPWM 33  // Was 10 - now using safe GPIO
#define BR_LPWM 32  // Was 11 - now using safe GPIO
#define BR_R_EN 12  // Was 13 - now using safe GPIO
#define BR_L_EN 13  // Was 9 - now using safe GPIO (GPIO 13 is actually okay for output)



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

// ================= Cytron Driver Implementation =================

CytronDriver::CytronDriver(int dir, int pwm) : dir_pin(dir), pwm_pin(pwm) {
}

void CytronDriver::init() {
    // Configure pins for Cytron motor driver
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    // Initialize to stopped state
    digitalWrite(dir_pin, LOW);  // Default direction
    analogWrite(pwm_pin, 0);     // No speed

    Serial.println("Cytron driver initialized");
}

void CytronDriver::setSpeed(float speed) {
    // Clamp speed to valid range
    speed = constrain(speed, -1.0f, 1.0f);

    if (speed > 0.01f) {
        // Forward direction
        digitalWrite(dir_pin, HIGH);
        analogWrite(pwm_pin, speed * 255);
    } else if (speed < -0.01f) {
        // Reverse direction
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, abs(speed) * 255);
    } else {
        // Stop (very small speeds treated as stop)
        analogWrite(pwm_pin, 0);
    }
}

void CytronDriver::stop() {
    analogWrite(pwm_pin, 0);
    // Direction doesn't matter when stopped
}

// ================= EncoderManager Implementation =================

EncoderManager::EncoderManager(float wheel_r, float width, float length, int ppr)
    : wheel_radius(wheel_r), wheelbase_width(width), wheelbase_length(length), encoder_ppr(ppr) {
    
    // initialize arrays
    for (int i = 0; i < 4; i++) {
        last_counts[i] = 0;
        velocities[i] = 0.0;
    }
    
    // initialize odometry
    pos_x = 0.0;
    pos_y = 0.0;
    theta = 0.0;
    last_time = 0;
}

void EncoderManager::init() {
    Serial.println("Initializing encoders with PCNT hardware...");
    
    // Enable internal pullup resistors for encoders
    // Options: UP, DOWN, NONE
    ESP32Encoder::useInternalWeakPullResistors = NONE;
    
    // Attach encoders to PCNT units
    encoders[FRONT_LEFT].attachFullQuad(ENC1_A, ENC1_B);
    encoders[FRONT_RIGHT].attachFullQuad(ENC2_A, ENC2_B);
    encoders[BACK_LEFT].attachFullQuad(ENC3_A, ENC3_B);
    encoders[BACK_RIGHT].attachFullQuad(ENC4_A, ENC4_B);
    
    // Clear all encoder counts
    for (int i = 0; i < 4; i++) {
        encoders[i].clearCount();
        last_counts[i] = 0;
    }
    
    last_time = millis();
    
    Serial.println("Encoders initialized:");
    Serial.printf("  Motor 1 (FL): GPIO %d (A), GPIO %d (B)\n", ENC1_A, ENC1_B);
    Serial.printf("  Motor 2 (FR): GPIO %d (A), GPIO %d (B)\n", ENC2_A, ENC2_B);
    Serial.printf("  Motor 3 (BL): GPIO %d (A), GPIO %d (B)\n", ENC3_A, ENC3_B);
    Serial.printf("  Motor 4 (BR): GPIO %d (A), GPIO %d (B)\n", ENC4_A, ENC4_B);
}

void EncoderManager::update() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;  // convert to seconds
    
    if (dt < 0.001) return;  // avoid division by very small numbers
    
    // Read encoder counts and calculate velocities
    for (int i = 0; i < 4; i++) {
        long current_count = encoders[i].getCount();
        long delta_count = current_count - last_counts[i];
        
        // Calculate angular velocity (rad/s)
        // velocity = (delta_ticks / ppr) * 2*PI / dt
        velocities[i] = (delta_count / (float)encoder_ppr) * TWO_PI / dt;
        
        last_counts[i] = current_count;
    }
    
    // Calculate robot velocities using mecanum kinematics
    // Convert wheel velocities to robot frame velocities
    float v_fl = velocities[FRONT_LEFT] * wheel_radius;
    float v_fr = velocities[FRONT_RIGHT] * wheel_radius;
    float v_bl = velocities[BACK_LEFT] * wheel_radius;
    float v_br = velocities[BACK_RIGHT] * wheel_radius;
    
    // Inverse mecanum kinematics
    // vx = (v_fl - v_fr + v_bl - v_br) / 4  (strafe)
    // vy = (v_fl + v_fr + v_bl + v_br) / 4  (forward)
    // omega = (v_fl - v_fr + v_bl - v_br) / (4 * (L + W))
    
    float L = wheelbase_length / 2.0;
    float W = wheelbase_width / 2.0;
    
    float vx = (v_fl - v_fr - v_bl + v_br) / 4.0;  // strafe (right positive)
    float vy = (v_fl + v_fr + v_bl + v_br) / 4.0;  // forward
    float omega = (v_fl - v_fr + v_bl - v_br) / (4.0 * (L + W));  // rotation
    
    // Update odometry using robot velocities
    // Transform to global frame
    float vx_global = vx * cos(theta) - vy * sin(theta);
    float vy_global = vx * sin(theta) + vy * cos(theta);
    
    pos_x += vx_global * dt;
    pos_y += vy_global * dt;
    theta += omega * dt;
    
    // Normalize theta to [-PI, PI]
    while (theta > PI) theta -= TWO_PI;
    while (theta < -PI) theta += TWO_PI;
    
    last_time = current_time;
}

void EncoderManager::reset() {
    for (int i = 0; i < 4; i++) {
        encoders[i].clearCount();
        last_counts[i] = 0;
        velocities[i] = 0.0;
    }
    
    pos_x = 0.0;
    pos_y = 0.0;
    theta = 0.0;
    last_time = millis();
    
    Serial.println("Encoders and odometry reset");
}

long EncoderManager::getCount(WheelPosition wheel) {
    return encoders[wheel].getCount();
}

float EncoderManager::getVelocity(WheelPosition wheel) {
    return velocities[wheel];
}

void EncoderManager::getOdometry(float &x, float &y, float &heading) {
    x = pos_x;
    y = pos_y;
    heading = theta;
}

void EncoderManager::getVelocities(float &vx, float &vy, float &omega) {
    // Calculate instantaneous robot velocities
    float v_fl = velocities[FRONT_LEFT] * wheel_radius;
    float v_fr = velocities[FRONT_RIGHT] * wheel_radius;
    float v_bl = velocities[BACK_LEFT] * wheel_radius;
    float v_br = velocities[BACK_RIGHT] * wheel_radius;
    
    float L = wheelbase_length / 2.0;
    float W = wheelbase_width / 2.0;
    
    vx = (v_fl - v_fr - v_bl + v_br) / 4.0;  // strafe
    vy = (v_fl + v_fr + v_bl + v_br) / 4.0;  // forward
    omega = (v_fl - v_fr + v_bl - v_br) / (4.0 * (L + W));  // rotation
}

String EncoderManager::getOdometryString() {
    // Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
    // This format allows the Jetson to reconstruct full odometry
    
    float vx, vy, omega;
    getVelocities(vx, vy, omega);
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "ODOM,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%ld,%ld,%ld,%ld",
             pos_x, pos_y, theta,
             vx, vy, omega,
             encoders[0].getCount(), encoders[1].getCount(),
             encoders[2].getCount(), encoders[3].getCount());
    
    return String(buffer);
}

// ================= MecanumBase Implementation =================

MecanumBase::MecanumBase(MotorDriverType type, KinematicsMethod method) 
    : driver_type(type), kinematics_method(method) {
    // initialize motor driver pointers to null
    for (int i = 0; i < 4; i++) {
        motors[i] = nullptr;
    }
    
    // Create encoder manager with actual robot dimensions
    // Robot specifications:
    //   - 152mm Mecanum Wheels (76.2mm radius)
    //   - 360mm base width + 2×(26.5mm flange + 27.75mm half-wheel) = 468.5mm wheelbase width
    //   - 460mm base length - 40mm motor offset = 420mm wheelbase length
    //   - 17 PPR encoder × 4 (quadrature) × 92 (gear ratio) = 6256 PPR
    encoders = new EncoderManager(WHEEL_RADIUS, WHEELBASE_WIDTH, WHEELBASE_LENGTH, ENCODER_PPR);
}

MecanumBase::~MecanumBase() {
    // clean up motor drivers
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            delete motors[i];
        }
    }
    
    // clean up encoder manager
    if (encoders != nullptr) {
        delete encoders;
    }
}

void MecanumBase::init() {
    Serial.println("initializing mecanum base...");
    
    // create motor drivers based on type
    if (driver_type == BTS7960) {
        // BTS7960 drivers need 4 pins each: RPWM, LPWM, R_EN, L_EN
        // Using safe GPIO pins to avoid boot issues
        motors[FRONT_LEFT] = new BTS7960Driver(16, 17, 21, 19);   // FL
        motors[FRONT_RIGHT] = new BTS7960Driver(25, 26, 34, 35);  // FR
        motors[BACK_LEFT] = new BTS7960Driver(22, 23, 5, 18);     // BL
        motors[BACK_RIGHT] = new BTS7960Driver(33, 32, 12, 13);   // BR
        Serial.println("using bts7960 motor drivers");
    } else if (driver_type == CYTRON_DUAL) {
        // Cytron dual motor drivers - each uses DIR and PWM pins
        motors[FRONT_LEFT] = new CytronDriver(FL_DIR, FL_PWM);
        motors[FRONT_RIGHT] = new CytronDriver(FR_DIR, FR_PWM);
        motors[BACK_LEFT] = new CytronDriver(BL_DIR, BL_PWM);
        motors[BACK_RIGHT] = new CytronDriver(BR_DIR, BR_PWM);
        Serial.println("using Cytron 20A Dual Motor drivers");
    }
    
    // initialize all motors
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            motors[i]->init();
        }
    }
    
    // Initialize encoders
    encoders->init();
    
    Serial.println("mecanum base initialized");
}

EncoderManager* MecanumBase::getEncoders() {
    return encoders;
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
    // positive rotation = counter-clockwise (ROS2 standard)
    
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
            // flip right side motors (front right and back right)
            if (i == FRONT_RIGHT || i == BACK_RIGHT) {
                motors[i]->setSpeed(-speeds[i]);  // negate right side
            } else {
                motors[i]->setSpeed(speeds[i]);   // left side normal
            }
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
    // To basically change the Motor Driver Type at runtime
    
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