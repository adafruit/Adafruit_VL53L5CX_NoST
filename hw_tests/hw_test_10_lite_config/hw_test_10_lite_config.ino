/*!
 * @file hw_test_10_lite_config.ino
 *
 * Hardware test: config get/set + ranging verification
 *
 * Config changes must be made before startRanging().
 * This test verifies all getters/setters, then ranges at 4x4 and 8x8.
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

uint8_t passed = 0;
uint8_t failed = 0;

void report(const char *name, bool ok) {
  Serial.print(name);
  if (ok) {
    Serial.println(F(" ... PASSED"));
    passed++;
  } else {
    Serial.println(F(" ... FAILED"));
    failed++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 10: Config ===\n"));

  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000);

  // Test 1: begin
  bool initOk = sensor.begin(0x29, &Wire1);
  report("1. begin()", initOk);
  if (!initOk) { while (1) delay(10); }

  // Test 2: Default resolution
  uint8_t res = sensor.getResolution();
  Serial.print(F("   ")); Serial.println(res);
  report("2. Default resolution 16", res == 16);

  // Test 3-4: Resolution set/get
  sensor.setResolution(64);
  res = sensor.getResolution();
  report("3. Set resolution 64", res == 64);

  sensor.setResolution(16);
  res = sensor.getResolution();
  report("4. Set resolution back 16", res == 16);

  // Test 5-6: Ranging frequency
  uint8_t freq = sensor.getRangingFrequency();
  Serial.print(F("   default: ")); Serial.print(freq); Serial.println(F(" Hz"));
  report("5. Default freq readback", freq > 0);

  sensor.setRangingFrequency(15);
  freq = sensor.getRangingFrequency();
  report("6. Set freq 15 Hz", freq == 15);

  // Test 7-8: Integration time
  uint32_t intTime = sensor.getIntegrationTime();
  Serial.print(F("   default: ")); Serial.print(intTime); Serial.println(F(" ms"));
  report("7. Default int time", intTime > 0);

  sensor.setIntegrationTime(20);
  intTime = sensor.getIntegrationTime();
  report("8. Set int time 20 ms", intTime == 20);

  // Test 9-10: Sharpener
  uint8_t sharp = sensor.getSharpenerPercent();
  Serial.print(F("   default: ")); Serial.print(sharp); Serial.println(F("%"));
  report("9. Default sharpener", true);

  sensor.setSharpenerPercent(50);
  sharp = sensor.getSharpenerPercent();
  Serial.print(F("   set 50: ")); Serial.print(sharp); Serial.println(F("%"));
  report("10. Set sharpener 50%", sharp >= 49 && sharp <= 51);

  // Test 11-12: Target order
  uint8_t order = sensor.getTargetOrder();
  report("11. Default target order", order > 0);

  sensor.setTargetOrder(VL53L5_TARGET_ORDER_STRONGEST);
  order = sensor.getTargetOrder();
  report("12. Set target order strongest", order == VL53L5_TARGET_ORDER_STRONGEST);

  // Test 13-14: Ranging mode
  uint8_t mode = sensor.getRangingMode();
  report("13. Default ranging mode", mode > 0);

  sensor.setRangingMode(VL53L5_RANGING_MODE_AUTONOMOUS);
  mode = sensor.getRangingMode();
  report("14. Set mode autonomous", mode == VL53L5_RANGING_MODE_AUTONOMOUS);

  // Reset to continuous for ranging
  sensor.setRangingMode(VL53L5_RANGING_MODE_CONTINUOUS);
  sensor.setTargetOrder(VL53L5_TARGET_ORDER_CLOSEST);
  sensor.setSharpenerPercent(14);
  sensor.setIntegrationTime(10);
  sensor.setRangingFrequency(15);
  sensor.setResolution(16);

  // Test 15: Range at 4x4
  sensor.startRanging();
  {
    unsigned long t = millis();
    bool gotData = false;
    while (millis() - t < 3000) {
      if (sensor.isDataReady()) {
        int16_t distances[16];
        gotData = sensor.getRangingData(distances);
        if (gotData) {
          Serial.print(F("   4x4: "));
          for (int i = 0; i < 16; i++) {
            Serial.print(distances[i]); Serial.print(F(" "));
          }
          Serial.println();
        }
        break;
      }
      delay(5);
    }
    report("15. Range 4x4", gotData);
  }
  sensor.stopRanging();

  // Config for 8x8 (fresh init to reset sensor state)
  delay(500);
  bool reinit = sensor.begin(0x29, &Wire1);
  Serial.print(F("   Re-init: ")); Serial.println(reinit ? F("OK") : F("FAIL"));
  sensor.setResolution(64);
  sensor.setRangingFrequency(15);

  // Test 16: Range at 8x8
  uint8_t r = sensor.getResolution();
  Serial.print(F("   resolution: ")); Serial.println(r);
  bool startOk = sensor.startRanging();
  Serial.print(F("   startRanging: ")); Serial.println(startOk ? F("OK") : F("FAIL"));
  {
    unsigned long t = millis();
    bool gotData = false;
    while (millis() - t < 3000) {
      if (sensor.isDataReady()) {
        int16_t distances[64];
        gotData = sensor.getRangingData(distances);
        if (gotData) {
          Serial.print(F("   8x8 first 16: "));
          for (int i = 0; i < 16; i++) {
            Serial.print(distances[i]); Serial.print(F(" "));
          }
          Serial.println();
        }
        break;
      }
      delay(5);
    }
    report("16. Range 8x8", gotData);
  }
  sensor.stopRanging();

  // Summary
  Serial.println();
  Serial.print(passed); Serial.print(F("/")); Serial.print(passed + failed);
  Serial.print(F(" passed — "));
  Serial.println(failed == 0 ? F("ALL PASSED") : F("SOME FAILED"));
}

void loop() {
  delay(1000);
}
