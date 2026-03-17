/*!
 * @file hw_test_09_lite_init.ino
 *
 * Debug test: Lightweight VL53L5 driver — find where init fails
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 09: Lite Driver Debug ==="));
  Serial.println();

  Wire1.begin(SDA1, SCL1);  // STEMMA QT on ESP32 QT Py
  Wire1.setClock(400000);

  // Manual step-by-step init to find the failure point
  // Step 1: I2C scan
  Wire1.beginTransmission(0x29);
  bool i2cOk = (Wire1.endTransmission() == 0);
  Serial.print(F("I2C scan 0x29: "));
  Serial.println(i2cOk ? F("found") : F("NOT FOUND"));

  // Step 2: Try begin
  Serial.println(F("Calling begin()..."));
  unsigned long start = millis();
  bool ok = sensor.begin(0x29, &Wire1);
  unsigned long elapsed = millis() - start;
  Serial.print(F("begin() returned "));
  Serial.print(ok ? F("true") : F("false"));
  Serial.print(F(" in "));
  Serial.print(elapsed);
  Serial.println(F(" ms"));

  if (!ok) {
    Serial.println(F("FAILED — trying to identify where..."));
    
    // Check if sensor is still responding on I2C
    Wire1.beginTransmission(0x29);
    bool stillThere = (Wire1.endTransmission() == 0);
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
