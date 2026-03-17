/*!
 * @file simpletest.ino
 *
 * Simple ranging demo for VL53L5CX 8x8 multizone ToF sensor.
 * Prints a 4x4 distance grid to Serial every second.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("VL53L5CX NoST Simpletest"));

  Wire.begin();
  Wire.setClock(400000);

  if (!sensor.begin(0x29, &Wire)) {
    Serial.println(F("Sensor not found!"));
    while (1) delay(10);
  }
  Serial.println(F("Sensor initialized"));

  sensor.setRangingFrequency(15);
  sensor.startRanging();
}

void loop() {
  if (sensor.isDataReady()) {
    int16_t distances[16];
    uint8_t statuses[16];

    if (sensor.getRangingData(distances, statuses)) {
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
          int i = row * 4 + col;
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
