/*!
 * @file hw_test_11_lite_range_bisect.ino
 *
 * Bisect which config setter breaks ranging
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

bool tryRange(const char *label) {
  Serial.print(label);
  sensor.startRanging();
  unsigned long t = millis();
  while (millis() - t < 2000) {
    if (sensor.isDataReady()) {
      int16_t d[16];
      sensor.getRangingData(d);
      Serial.print(F(" OK: "));
      Serial.println(d[0]);
      sensor.stopRanging();
      return true;
    }
    delay(10);
  }
  // Debug: print raw bytes
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

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 11: Range Bisect ===\n"));

  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000);

  if (!sensor.begin(0x29, &Wire1)) {
    Serial.println(F("Init failed"));
    while (1) delay(10);
  }
  Serial.println(F("Init OK\n"));

  // Baseline: range right after init
  tryRange("1. After init");

  // Test each setter individually
  sensor.setResolution(64);
  tryRange("2. After setResolution(64)");

  sensor.setResolution(16);
  tryRange("3. After setResolution(16)");

  sensor.setRangingFrequency(15);
  tryRange("4. After setRangingFrequency(15)");

  sensor.setIntegrationTime(20);
  tryRange("5. After setIntegrationTime(20)");

  sensor.setSharpenerPercent(50);
  tryRange("6. After setSharpenerPercent(50)");

  sensor.setTargetOrder(VL53L5_TARGET_ORDER_STRONGEST);
  tryRange("7. After setTargetOrder(strongest)");

  sensor.setRangingMode(VL53L5_RANGING_MODE_AUTONOMOUS);
  tryRange("8. After setRangingMode(autonomous)");

  sensor.setRangingMode(VL53L5_RANGING_MODE_CONTINUOUS);
  tryRange("9. After setRangingMode(continuous)");

  Serial.println(F("\nDone."));
}

void loop() {
  delay(1000);
}
