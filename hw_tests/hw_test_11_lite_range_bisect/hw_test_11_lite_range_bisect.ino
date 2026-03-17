/*!
 * @file hw_test_11_lite_range_bisect.ino
 *
 * Bisect which config setter breaks ranging — test each one in isolation
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

bool tryRange(const char *label) {
  Serial.print(label);
  if (!sensor.startRanging()) {
    Serial.println(F(" startRanging FAILED"));
    return false;
  }
  unsigned long t = millis();
  while (millis() - t < 2000) {
    if (sensor.isDataReady()) {
      int16_t d[64];
      sensor.getRangingData(d);
      Serial.print(F(" OK: "));
      Serial.println(d[0]);
      sensor.stopRanging();
      return true;
    }
    delay(10);
  }
  uint8_t dbg[4];
  Wire1.beginTransmission(0x29);
  Wire1.write((uint8_t)0x00); Wire1.write((uint8_t)0x00);
  Wire1.endTransmission(false);
  Wire1.requestFrom((uint8_t)0x29, (uint8_t)4);
  for (int k = 0; k < 4; k++) dbg[k] = Wire1.read();
  Serial.print(F(" FAIL raw: "));
  for (int k = 0; k < 4; k++) {
    Serial.print(F("0x")); Serial.print(dbg[k], HEX); Serial.print(F(" "));
  }
  Serial.println();
  sensor.stopRanging();
  return false;
}

bool initSensor() {
  return sensor.begin(0x29, &Wire1);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 11: Range Bisect (isolated) ===\n"));

  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000);

  // Test each setter in isolation (re-init between each)

  // 1. Baseline
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  tryRange("1. Baseline (no config)");

  // 2. Sharpener only
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setSharpenerPercent(50);
  tryRange("2. setSharpenerPercent(50) only");

  // 3. Sharpener 14 (near default 85*100/255=33... default is 85 raw)
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setSharpenerPercent(14);
  tryRange("3. setSharpenerPercent(14) only");

  // 4. Target order only
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setTargetOrder(VL53L5_TARGET_ORDER_STRONGEST);
  tryRange("4. setTargetOrder(strongest) only");

  // 5. Ranging mode autonomous
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setRangingMode(VL53L5_RANGING_MODE_AUTONOMOUS);
  tryRange("5. setRangingMode(autonomous) only");

  // 6. Freq + integration + sharpener (typical config)
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setRangingFrequency(15);
  sensor.setIntegrationTime(20);
  sensor.setSharpenerPercent(50);
  tryRange("6. freq+intTime+sharpener combo");

  // 7. All config
  if (!initSensor()) { Serial.println(F("Init fail")); while(1); }
  sensor.setResolution(64);
  sensor.setRangingFrequency(15);
  sensor.setIntegrationTime(20);
  sensor.setSharpenerPercent(50);
  sensor.setTargetOrder(VL53L5_TARGET_ORDER_STRONGEST);
  tryRange("7. All config at 8x8");

  Serial.println(F("\nDone."));
}

void loop() {
  delay(1000);
}
