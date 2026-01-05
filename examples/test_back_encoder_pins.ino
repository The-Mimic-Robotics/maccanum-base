/*
 * Back Encoder Pin Diagnostics - Raw Digital Read Test
 * 
 * Tests ONLY the back left and back right encoder pins
 * to verify if signals are being received at all.
 * 
 * Purpose: Determine if encoder issues are due to:
 *   - Wiring problems (no signal detected)
 *   - Pin configuration issues (signal detected but erratic)
 * 
 * Upload via Arduino IDE:
 *   1. Tools > Board > ESP32 > ESP32 Dev Module
 *   2. Upload and open Serial Monitor (115200 baud)
 * 
 * Test: Manually rotate BACK LEFT and BACK RIGHT wheels
 *       Watch for state changes and transition counts
 */

// ================= BACK MOTOR ENCODER PINS =================
// Back Left (Motor 3)
#define BL_ENC_A 27   // GPIO 27 - Back Left Encoder A
#define BL_ENC_B 13   // GPIO 14 - Back Left Encoder B

// Back Right (Motor 4)
#define BR_ENC_A 18   // GPIO 18 - Back Right Encoder A
#define BR_ENC_B 4    // GPIO 4  - Back Right Encoder B

// Track previous states to detect transitions
int bl_a_prev = LOW;
int bl_b_prev = LOW;
int br_a_prev = LOW;
int br_b_prev = LOW;

// Count transitions (pulses)
unsigned long bl_a_transitions = 0;
unsigned long bl_b_transitions = 0;
unsigned long br_a_transitions = 0;
unsigned long br_b_transitions = 0;

// Timing for periodic updates
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // Print every 200ms

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== BACK ENCODER TEST ===");
  Serial.printf("BL: GPIO%d/%d | BR: GPIO%d/%d\n", BL_ENC_A, BL_ENC_B, BR_ENC_A, BR_ENC_B);
  
  // Configure pins as INPUT with internal pull-up resistors
  pinMode(BL_ENC_A, INPUT_PULLUP);
  pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP);
  pinMode(BR_ENC_B, INPUT_PULLUP);
  
  // Read initial states
  bl_a_prev = digitalRead(BL_ENC_A);
  bl_b_prev = digitalRead(BL_ENC_B);
  br_a_prev = digitalRead(BR_ENC_A);
  br_b_prev = digitalRead(BR_ENC_B);
  
  Serial.println("Rotate wheels to test...\n");
}

void loop() {
  // Read current pin states
  int bl_a_current = digitalRead(BL_ENC_A);
  int bl_b_current = digitalRead(BL_ENC_B);
  int br_a_current = digitalRead(BR_ENC_A);
  int br_b_current = digitalRead(BR_ENC_B);
  
  // Detect and count transitions
  if (bl_a_current != bl_a_prev) {
    bl_a_transitions++;
    bl_a_prev = bl_a_current;
  }
  if (bl_b_current != bl_b_prev) {
    bl_b_transitions++;
    bl_b_prev = bl_b_current;
  }
  if (br_a_current != br_a_prev) {
    br_a_transitions++;
    br_a_prev = br_a_current;
  }
  if (br_b_current != br_b_prev) {
    br_b_transitions++;
    br_b_prev = br_b_current;
  }
  
  // Print status periodically
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    
    Serial.printf("BL_A[%d]:%lu  BL_B[%d]:%lu | BR_A[%d]:%lu  BR_B[%d]:%lu\n", 
                  bl_a_current, bl_a_transitions, 
                  bl_b_current, bl_b_transitions,
                  br_a_current, br_a_transitions, 
                  br_b_current, br_b_transitions);
    
    // Diagnostic hints
    if (bl_a_transitions == 0 && bl_b_transitions == 0) {
      Serial.println("BL: NO SIGNAL!");
    }
    if (br_a_transitions == 0 && br_b_transitions == 0) {
      Serial.println("BR: NO SIGNAL!");
    }
  }
  
  delay(1);  // Small delay to prevent overwhelming the loop
}
