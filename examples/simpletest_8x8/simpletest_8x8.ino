/*!
 * @file simpletest_8x8.ino
 *
 * 8x8 ranging demo for VL53L5CX multizone ToF sensor.
 * Prints an 8x8 distance grid to Serial.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("VL53L5CX NoST 8x8 Demo"));

  Wire.begin();
  Wire.setClock(400000);

  if (!sensor.begin(0x29, &Wire)) {
    Serial.println(F("Sensor not found!"));
    while (1) delay(10);
  }
  Serial.println(F("Sensor initialized"));

  sensor.setResolution(64);
  sensor.setRangingFrequency(15);
  sensor.startRanging();
}

void loop() {
  if (sensor.isDataReady()) {
    int16_t distances[64];
    uint8_t statuses[64];

    if (sensor.getRangingData(distances, statuses)) {
      for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
          int i = row * 8 + col;
          if (distances[i] < 10000) {
            Serial.print(distances[i]);
          } else {
            Serial.print(F("----"));
          }
          Serial.print(F("\t"));
        }
        Serial.println();
      }
      Serial.println();
    }
  }
}
