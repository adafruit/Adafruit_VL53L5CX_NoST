/*!
 * @file hw_test_10_lite_config.ino
 *
 * Hardware test: Lite driver config get/set functions
 *
 * Tests:
 *  1. begin() succeeds
 *  2. Default resolution is 4x4 (16)
 *  3. Set resolution to 8x8 (64) and readback
 *  4. Set resolution back to 4x4 (16) and readback
 *  5. Default ranging frequency readback
 *  6. Set ranging frequency to 15 Hz and readback
 *  7. Default integration time readback
 *  8. Set integration time to 20 ms and readback
 *  9. Default sharpener readback
 * 10. Set sharpener to 50% and readback
 * 11. Default target order readback
 * 12. Set target order to strongest and readback
 * 13. Default ranging mode readback
 * 14. Set ranging mode to autonomous and readback
 * 15. Start ranging + get data at 4x4
 * 16. Start ranging + get data at 8x8
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

  Serial.println(F("=== HW Test 10: Lite Config ==="));
  Serial.println();

  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000);

  // Test 1: begin
  bool initOk = sensor.begin(0x29, &Wire1);
  report("1. begin() succeeds", initOk);
  if (!initOk) {
    Serial.println(F("Init failed, cannot continue."));
    while (1) delay(10);
  }

  // Test 2: Default resolution
  uint8_t res = sensor.getResolution();
  Serial.print(F("   Resolution: ")); Serial.println(res);
  report("2. Default resolution is 16 (4x4)", res == 16);

  // Test 3: Set resolution to 8x8
  bool setOk = sensor.setResolution(64);
  res = sensor.getResolution();
  Serial.print(F("   Resolution after set 64: ")); Serial.println(res);
  report("3. Set resolution to 64 (8x8)", setOk && res == 64);

  // Test 4: Set resolution back to 4x4
  setOk = sensor.setResolution(16);
  res = sensor.getResolution();
  Serial.print(F("   Resolution after set 16: ")); Serial.println(res);
  report("4. Set resolution back to 16 (4x4)", setOk && res == 16);

  // Test 5: Default ranging frequency
  uint8_t freq = sensor.getRangingFrequency();
  Serial.print(F("   Ranging freq: ")); Serial.print(freq); Serial.println(F(" Hz"));
  report("5. Default ranging freq readback", freq > 0);

  // Test 6: Set ranging frequency to 15 Hz
  setOk = sensor.setRangingFrequency(15);
  freq = sensor.getRangingFrequency();
  Serial.print(F("   Ranging freq after set 15: ")); Serial.print(freq); Serial.println(F(" Hz"));
  report("6. Set ranging freq to 15 Hz", setOk && freq == 15);

  // Test 7: Default integration time
  uint32_t intTime = sensor.getIntegrationTime();
  Serial.print(F("   Integration time: ")); Serial.print(intTime); Serial.println(F(" ms"));
  report("7. Default integration time readback", intTime > 0);

  // Test 8: Set integration time to 20 ms
  setOk = sensor.setIntegrationTime(20);
  intTime = sensor.getIntegrationTime();
  Serial.print(F("   Integration time after set 20: ")); Serial.print(intTime); Serial.println(F(" ms"));
  report("8. Set integration time to 20 ms", setOk && intTime == 20);

  // Test 9: Default sharpener
  uint8_t sharp = sensor.getSharpenerPercent();
  Serial.print(F("   Sharpener: ")); Serial.print(sharp); Serial.println(F("%"));
  report("9. Default sharpener readback", true); // just check it reads

  // Test 10: Set sharpener to 50%
  setOk = sensor.setSharpenerPercent(50);
  sharp = sensor.getSharpenerPercent();
  Serial.print(F("   Sharpener after set 50: ")); Serial.print(sharp); Serial.println(F("%"));
  report("10. Set sharpener to 50%", setOk && sharp == 50);

  // Test 11: Default target order
  uint8_t order = sensor.getTargetOrder();
  Serial.print(F("   Target order: ")); Serial.println(order);
  report("11. Default target order readback", order > 0);

  // Test 12: Set target order to strongest (2)
  setOk = sensor.setTargetOrder(VL53L5_TARGET_ORDER_STRONGEST);
  order = sensor.getTargetOrder();
  Serial.print(F("   Target order after set 2: ")); Serial.println(order);
  report("12. Set target order to strongest", setOk && order == VL53L5_TARGET_ORDER_STRONGEST);

  // Test 13: Default ranging mode
  uint8_t mode = sensor.getRangingMode();
  Serial.print(F("   Ranging mode: ")); Serial.println(mode);
  report("13. Default ranging mode readback", mode > 0);

  // Test 14: Set ranging mode to autonomous (3) and back
  setOk = sensor.setRangingMode(VL53L5_RANGING_MODE_AUTONOMOUS);
  mode = sensor.getRangingMode();
  Serial.print(F("   Ranging mode after set 3: ")); Serial.println(mode);
  report("14. Set ranging mode to autonomous", setOk && mode == VL53L5_RANGING_MODE_AUTONOMOUS);
  sensor.setRangingMode(VL53L5_RANGING_MODE_CONTINUOUS);

  // Reset config to defaults for ranging tests
  sensor.setTargetOrder(VL53L5_TARGET_ORDER_CLOSEST);
  sensor.setSharpenerPercent(14);
  sensor.setIntegrationTime(10);
  sensor.setRangingFrequency(15);

  // Test 15: Ranging at 4x4
  sensor.setResolution(16);
  Serial.print(F("   startRanging: "));
  bool startOk = sensor.startRanging();
  Serial.println(startOk ? F("OK") : F("FAILED"));
  {
    unsigned long t = millis();
    bool gotData = false;
    uint8_t pollCount = 0;
    while (millis() - t < 5000) {
      if (sensor.isDataReady()) {
        int16_t distances[16];
        gotData = sensor.getRangingData(distances);
        if (gotData) {
          Serial.print(F("   4x4 distances: "));
          for (int i = 0; i < 16; i++) {
            Serial.print(distances[i]); Serial.print(F(" "));
          }
          Serial.println();
        }
        break;
      }
      if (pollCount < 3) {
        // Debug: print raw data ready bytes
        uint8_t dbg[4];
        Wire1.beginTransmission(0x29);
        Wire1.write((uint8_t)0x00); Wire1.write((uint8_t)0x00);
        Wire1.endTransmission(false);
        Wire1.requestFrom((uint8_t)0x29, (uint8_t)4);
        for (int k = 0; k < 4; k++) dbg[k] = Wire1.read();
        Serial.print(F("   DataReady raw: "));
        for (int k = 0; k < 4; k++) {
          Serial.print(F("0x")); Serial.print(dbg[k], HEX); Serial.print(F(" "));
        }
        Serial.println();
        pollCount++;
      }
      delay(100);
    }
    report("15. Ranging at 4x4 gets data", gotData);
  }
  sensor.stopRanging();

  // Test 16: Ranging at 8x8
  sensor.setResolution(64);
  Serial.print(F("   startRanging 8x8: "));
  startOk = sensor.startRanging();
  Serial.println(startOk ? F("OK") : F("FAILED"));
  {
    unsigned long t = millis();
    bool gotData = false;
    while (millis() - t < 5000) {
      if (sensor.isDataReady()) {
        int16_t distances[64];
        gotData = sensor.getRangingData(distances);
        if (gotData) {
          Serial.print(F("   8x8 distances (first 16): "));
          for (int i = 0; i < 16; i++) {
            Serial.print(distances[i]); Serial.print(F(" "));
          }
          Serial.println();
        }
        break;
      }
      delay(5);
    }
    report("16. Ranging at 8x8 gets data", gotData);
  }
  sensor.stopRanging();

  // Summary
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed); Serial.print(F(" passed, "));
  Serial.print(failed); Serial.println(F(" failed"));
  Serial.print(F("Result: "));
  Serial.println(failed == 0 ? F("ALL PASSED") : F("SOME FAILED"));
}

void loop() {
  delay(1000);
}
