/*
 * Mimic Mecanum Robot Base - Implementation
 * 
 * Complete implementation of mecanum drive system including:
 *   - Motor driver control (BTS7960 & Cytron)
 *   - Quadrature encoder reading using ESP32 PCNT hardware
 *   - Odometry calculation (position & velocity estimation)
 *   - Mecanum wheel kinematics (forward & inverse)
 * 
 * COORDINATE SYSTEM: ROS2 REP-103 Standard
 *   Linear: +X=Forward, +Y=Left, +Z=Up
 *   Angular: +Z=CCW rotation (right-hand rule)
 *   All odometry and velocity data follows this convention
 * 
 * TARGET BOARD: Freenove ESP32-WROOM (USB-C port pointing SOUTH)
 * 
 * Pin assignments - LEFT/RIGHT physical separation:
 *   LEFT SIDE (FL + BL motors):
 *     - FL encoder: GPIO 32, 33 | FL driver: GPIO 25, 26
 *     - BL encoder: GPIO 27, 14 | BL driver: GPIO 13, 12
 * 
 *   RIGHT SIDE (FR + BR motors):
 *     - FR encoder: GPIO 23, 22 | FR driver: GPIO 21, 19
 *     - BR encoder: GPIO 18, 4  | BR driver: GPIO 17, 16
 * 
 * All encoder pins are PCNT-compatible and avoid boot/strapping conflicts
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


// ================= PIN DEFINITIONS FOR FREENOVE ESP32-WROOM =================
// USB-C port pointing SOUTH
// LEFT side pins = LEFT motors (FL + BL)
// RIGHT side pins = RIGHT motors (FR + BR)
// All encoders use PCNT-compatible GPIOs, avoiding boot/strapping pins

// ================= LEFT SIDE MOTORS (Physical LEFT of board) =================

// Motor 1 - Front Left (FL) - Uses LEFT side GPIOs
#define ENC1_A 32   // LEFT side - PCNT compatible
#define ENC1_B 33   // LEFT side - PCNT compatible
#define FL_DIR 25   // LEFT side - Cytron DIR
#define FL_PWM 26   // LEFT side - Cytron PWM

// Motor 3 - Back Left (BL) - Uses LEFT side GPIOs
#define ENC3_A 27   // LEFT side - PCNT compatible
#define ENC3_B 13   // LEFT side - PCNT compatible
#define BL_DIR 14   // LEFT side - Cytron DIR
#define BL_PWM 12   // LEFT side - Cytron PWM

// ================= RIGHT SIDE MOTORS (Physical RIGHT of board) =================

// Motor 2 - Front Right (FR) - Uses RIGHT side GPIOs
#define ENC2_A 23   // RIGHT side - PCNT compatible
#define ENC2_B 22   // RIGHT side - PCNT compatible
#define FR_DIR 21   // RIGHT side - Cytron DIR
#define FR_PWM 19   // RIGHT side - Cytron PWM

// Motor 4 - Back Right (BR) - Uses RIGHT side GPIOs
#define ENC4_A 18   // RIGHT side - PCNT compatible
#define ENC4_B 4    // RIGHT side - PCNT compatible
#define BR_DIR 17   // RIGHT side - Cytron DIR
#define BR_PWM 16   // RIGHT side - Cytron PWM

// ================= BTS7960 DRIVER PINS (Legacy - for BTS7960 compatibility) =================
// LEFT side motors (FL, BL)
#define FL_RPWM 26
#define FL_LPWM 25
#define FL_R_EN 33
#define FL_L_EN 32

#define BL_RPWM 12
#define BL_LPWM 13
#define BL_R_EN 14
#define BL_L_EN 27

// RIGHT side motors (FR, BR)
#define FR_RPWM 19
#define FR_LPWM 21
#define FR_R_EN 22
#define FR_L_EN 23

#define BR_RPWM 16
#define BR_LPWM 17
#define BR_R_EN 4
#define BR_L_EN 18



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
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, speed * 255);
    } else if (speed < -0.01f) {
        // Reverse direction
        digitalWrite(dir_pin, HIGH);
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
    vel_forward = 0.0;
    vel_left = 0.0;
    vel_omega = 0.0;
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
    
    Serial.println("Encoders initialized on Freenove ESP32-WROOM:");
    Serial.println("  LEFT SIDE:");
    Serial.printf("    Motor 1 (FL): GPIO %d (A), GPIO %d (B)\n", ENC1_A, ENC1_B);
    Serial.printf("    Motor 3 (BL): GPIO %d (A), GPIO %d (B)\n", ENC3_A, ENC3_B);
    Serial.println("  RIGHT SIDE:");
    Serial.printf("    Motor 2 (FR): GPIO %d (A), GPIO %d (B)\n", ENC2_A, ENC2_B);
    Serial.printf("    Motor 4 (BR): GPIO %d (A), GPIO %d (B)\n", ENC4_A, ENC4_B);
}

void EncoderManager::update() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    
    if (dt < 0.001) return;
    
    // Read encoder counts and calculate wheel velocities
    for (int i = 0; i < 4; i++) {
        long current_count = encoders[i].getCount();
        
        // Invert left side encoders (CCW for forward)
        if (i == FRONT_LEFT || i == BACK_LEFT) {
            current_count = -current_count;
        }
        
        long delta_count = current_count - last_counts[i];
        velocities[i] = (delta_count / (float)encoder_ppr) * TWO_PI / dt;
        last_counts[i] = current_count;
    }
    
    // Convert wheel velocities to linear velocities
    float v_fl = velocities[FRONT_LEFT] * wheel_radius;
    float v_fr = velocities[FRONT_RIGHT] * wheel_radius;
    float v_bl = velocities[BACK_LEFT] * wheel_radius;
    float v_br = velocities[BACK_RIGHT] * wheel_radius;
    
    // Mecanum inverse kinematics → Robot frame velocities
    float L = wheelbase_length / 2.0;
    float W = wheelbase_width / 2.0;
    
    float vx_strafe = (v_fl - v_fr - v_bl + v_br) / 4.0;  // strafe right+
    float vy_forward = (v_fl + v_fr + v_bl + v_br) / 4.0; // forward+
    float omega_ccw = (v_fl - v_fr + v_bl - v_br) / (4.0 * (L + W)); // CCW+
    
    // Convert to ROS2 REP-103 (store these for odometry message)
    vel_forward = vy_forward;    // ROS X = forward
    vel_left = -vx_strafe;       // ROS Y = left (negate right-strafe)
    vel_omega = omega_ccw;       // ROS Z = CCW (already correct)
    
    // Update global position
    float vx_global = vel_forward * cos(theta) - vel_left * sin(theta);
    float vy_global = vel_forward * sin(theta) + vel_left * cos(theta);
    
    pos_x += vx_global * dt;
    pos_y += vy_global * dt;
    theta += vel_omega * dt;
    
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
    vel_forward = 0.0;
    vel_left = 0.0;
    vel_omega = 0.0;
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
    // Return stored ROS2 REP-103 velocities (calculated in update())
    vx = vel_forward;  // X = forward
    vy = vel_left;     // Y = left
    omega = vel_omega; // Z = CCW
}

String EncoderManager::getOdometryString() {
    // Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
    // All values already in ROS2 REP-103 (calculated in update())
    
    // Get encoder counts with left-side inversion
    long enc1 = -encoders[0].getCount();  // FL
    long enc2 = encoders[1].getCount();   // FR
    long enc3 = -encoders[2].getCount();  // BL
    long enc4 = encoders[3].getCount();   // BR
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "ODOM,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%ld,%ld,%ld,%ld",
             pos_x, pos_y, theta,
             vel_forward, vel_left, vel_omega,
             enc1, enc2, enc3, enc4);
    
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
    Serial.println("Initializing mecanum base on Freenove ESP32-WROOM...");
    
    // create motor drivers based on type
    if (driver_type == BTS7960) {
        // BTS7960 drivers: RPWM, LPWM, R_EN, L_EN
        // LEFT side motors use LEFT side pins
        motors[FRONT_LEFT] = new BTS7960Driver(FL_RPWM, FL_LPWM, FL_R_EN, FL_L_EN);   // FL - LEFT pins
        motors[BACK_LEFT] = new BTS7960Driver(BL_RPWM, BL_LPWM, BL_R_EN, BL_L_EN);    // BL - LEFT pins
        // RIGHT side motors use RIGHT side pins
        motors[FRONT_RIGHT] = new BTS7960Driver(FR_RPWM, FR_LPWM, FR_R_EN, FR_L_EN);  // FR - RIGHT pins
        motors[BACK_RIGHT] = new BTS7960Driver(BR_RPWM, BR_LPWM, BR_R_EN, BR_L_EN);   // BR - RIGHT pins
        Serial.println("Using BTS7960 motor drivers with LEFT/RIGHT pin separation");
    } else if (driver_type == CYTRON_DUAL) {
        // Cytron dual motor drivers - DIR and PWM pins
        // LEFT side motors use LEFT side pins
        motors[FRONT_LEFT] = new CytronDriver(FL_DIR, FL_PWM);   // FL - LEFT pins
        motors[BACK_LEFT] = new CytronDriver(BL_DIR, BL_PWM);    // BL - LEFT pins
        // RIGHT side motors use RIGHT side pins
        motors[FRONT_RIGHT] = new CytronDriver(FR_DIR, FR_PWM);  // FR - RIGHT pins
        motors[BACK_RIGHT] = new CytronDriver(BR_DIR, BR_PWM);   // BR - RIGHT pins
        Serial.println("Using Cytron 20A Dual Motor drivers with LEFT/RIGHT pin separation");
    }
    
    // initialize all motors
    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr) {
            motors[i]->init();
        }
    }
    
    // Initialize encoders
    encoders->init();
    
    Serial.println("Mecanum base initialized on Freenove ESP32-WROOM");
    Serial.println("Pin mapping: LEFT motors use LEFT side | RIGHT motors use RIGHT side");
}

EncoderManager* MecanumBase::getEncoders() {
    return encoders;
}

void MecanumBase::calculateWheelSpeeds(float forward, float left, float rotation_ccw, float speeds[4]) {
    // choose calculation method based on kinematics_method setting
    if (kinematics_method == SIMPLE) {
        calculateWheelSpeedsSimple(forward, left, rotation_ccw, speeds);
    } else {
        calculateWheelSpeedsComplex(forward, left, rotation_ccw, speeds);
    }
}

void MecanumBase::calculateWheelSpeedsSimple(float forward, float left, float rotation_ccw, float speeds[4]) {
    // ROS2 REP-103 mecanum kinematics
    // forward = X+ (robot moves forward)
    // left = Y+ (robot strafes left)
    // rotation_ccw = Z+ (robot rotates counter-clockwise)
    
    // HARDWARE CORRECTION: Motor wiring causes reversed rotation
    // Negate rotation here to compensate (REP-103 still maintained at API level)
    float rotation = -rotation_ccw;
    
    speeds[FRONT_LEFT] = forward + left + rotation;
    speeds[FRONT_RIGHT] = forward - left - rotation;
    speeds[BACK_LEFT] = forward - left + rotation;
    speeds[BACK_RIGHT] = forward + left - rotation;
    
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

void MecanumBase::calculateWheelSpeedsComplex(float forward, float left, float rotation_ccw, float speeds[4]) {
    // ROS2 REP-103 convention: forward, left, rotation_ccw
    float x = left;      // Convert to mecanum math frame
    float y = forward;
    float rotation = -rotation_ccw;    // HARDWARE CORRECTION: Motor wiring causes reversed rotation

    
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

void MecanumBase::move(float forward, float left, float rotation_ccw) {
    float speeds[4];
    calculateWheelSpeeds(forward, left, rotation_ccw, speeds);
    
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