/*!
 * @file hw_test_09_lite_init.ino
 *
 * Debug test: Lightweight VL53L5 driver — find where init fails
 */

#include "Adafruit_VL53L5CX_NoST.h"
#include "../hw_test_helper.h"

Adafruit_VL53L5CX_NoST sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 09: Lite Driver Debug ==="));
  Serial.println();

  HW_TEST_I2C_INIT();  // STEMMA QT on ESP32 QT Py
  

  // Manual step-by-step init to find the failure point
  // Step 1: I2C scan
  HW_TEST_WIRE.beginTransmission(0x29);
  bool i2cOk = (HW_TEST_WIRE.endTransmission() == 0);
  Serial.print(F("I2C scan 0x29: "));
  Serial.println(i2cOk ? F("found") : F("NOT FOUND"));

  // Step 2: Try begin
  Serial.println(F("Calling begin()..."));
  unsigned long start = millis();
  bool ok = sensor.begin(0x29, &HW_TEST_WIRE);
  unsigned long elapsed = millis() - start;
  Serial.print(F("begin() returned "));
  Serial.print(ok ? F("true") : F("false"));
  Serial.print(F(" in "));
  Serial.print(elapsed);
  Serial.println(F(" ms"));

  if (!ok) {
    Serial.println(F("FAILED — trying to identify where..."));
    
    // Check if sensor is still responding on I2C
    HW_TEST_WIRE.beginTransmission(0x29);
    bool stillThere = (HW_TEST_WIRE.endTransmission() == 0);
    Serial.print(F("I2C still responding: "));
    Serial.println(stillThere ? F("yes") : F("no"));
  } else {
    Serial.println(F("SUCCESS!"));
    
    // Quick ranging test
    uint8_t res = sensor.getResolution();
    Serial.print(F("Resolution: "));
    Serial.println(res);
    
    sensor.setRangingFrequency(15);
    sensor.startRanging();
    
    unsigned long t = millis();
    while (millis() - t < 5000) {
      if (sensor.isDataReady()) {
        int16_t distances[16];
        sensor.getRangingData(distances);
        Serial.print(F("Distances: "));
        for (int i = 0; i < 16; i++) {
          Serial.print(distances[i]);
          Serial.print(F(" "));
        }
        Serial.println();
        break;
      }
      delay(5);
    }
    sensor.stopRanging();
  }

  Serial.println(F("Done."));
}

void loop() {
  delay(1000);
}
