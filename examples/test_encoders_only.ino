/*
 * ESP32 Encoder Test - PCNT Hardware Only
 * 
 * Quick test for Freenove ESP32-WROOM encoder reading
 * No motors, just prints encoder counts every 100ms
 * 
 * Upload via Arduino IDE:
 *   1. Tools > Board > ESP32 > ESP32 Dev Module
 *   2. Install library: ESP32Encoder by Kevin Harrington
 *   3. Upload and open Serial Monitor (115200 baud)
 * 
 * Manual test: Rotate each wheel by hand and verify counts change
 */

#include <ESP32Encoder.h>

// ================= Freenove ESP32-WROOM Pin Mapping =================
// LEFT side motors
#define ENC1_A 32   // Front Left - Encoder A
#define ENC1_B 33   // Front Left - Encoder B
#define ENC3_A 27   // Back Left - Encoder A
#define ENC3_B 14   // Back Left - Encoder B

// RIGHT side motors
#define ENC2_A 23   // Front Right - Encoder A
#define ENC2_B 22   // Front Right - Encoder B
#define ENC4_A 18   // Back Right - Encoder A
#define ENC4_B 4    // Back Right - Encoder B

// Create encoder objects
ESP32Encoder encoder1;  // Front Left (FL)
ESP32Encoder encoder2;  // Front Right (FR)
ESP32Encoder encoder3;  // Back Left (BL)
ESP32Encoder encoder4;  // Back Right (BR)

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32 Encoder Test - PCNT Hardware");
  Serial.println("Freenove ESP32-WROOM Board");
  Serial.println("========================================\n");
  
  // Configure pull-up resistors (NONE, UP, or DOWN)
  // Try UP if encoders don't count reliably
  // ESP32Encoder::useInternalWeakPullResistors = NONE;
  
  // Attach encoders to PCNT units
  Serial.println("Initializing encoders...");
  
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  Serial.printf("  FL (Motor 1): GPIO %d (A), GPIO %d (B)\n", ENC1_A, ENC1_B);
  
  encoder2.attachFullQuad(ENC2_A, ENC2_B);
  Serial.printf("  FR (Motor 2): GPIO %d (A), GPIO %d (B)\n", ENC2_A, ENC2_B);
  
  encoder3.attachFullQuad(ENC3_A, ENC3_B);
  Serial.printf("  BL (Motor 3): GPIO %d (A), GPIO %d (B)\n", ENC3_A, ENC3_B);
  
  encoder4.attachFullQuad(ENC4_A, ENC4_B);
  Serial.printf("  BR (Motor 4): GPIO %d (A), GPIO %d (B)\n", ENC4_A, ENC4_B);
  
  // Clear all counts
  encoder1.clearCount();
  encoder2.clearCount();
  encoder3.clearCount();
  encoder4.clearCount();
  
  Serial.println("\n✓ Encoders initialized!");
  Serial.println("\nManual Test Instructions:");
  Serial.println("  - Rotate FL wheel → Enc1 should change");
  Serial.println("  - Rotate FR wheel → Enc2 should change");
  Serial.println("  - Rotate BL wheel → Enc3 should change");
  Serial.println("  - Rotate BR wheel → Enc4 should change");
  Serial.println("  - Forward rotation → counts increase");
  Serial.println("  - Backward rotation → counts decrease");
  Serial.println("\nPress RESET to zero counts\n");
  Serial.println("Format: FL | FR | BL | BR (counts)");
  Serial.println("----------------------------------------");
  
  delay(2000);
}

void loop() {
  // Read all encoder counts
  int64_t count1 = encoder1.getCount();
  int64_t count2 = encoder2.getCount();
  int64_t count3 = encoder3.getCount();
  int64_t count4 = encoder4.getCount();
  
  // Print in tabular format
  Serial.printf("FL: %7lld | FR: %7lld | BL: %7lld | BR: %7lld\n", 
                count1, count2, count3, count4);
  
  delay(100);  // Update at 10Hz
}
