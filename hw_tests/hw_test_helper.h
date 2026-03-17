/*!
 * @file hw_test_helper.h
 *
 * Board-specific I2C setup for hardware tests.
 * ESP32 QT Py uses Wire1 (SDA1/SCL1) for STEMMA QT connector.
 * All other boards default to Wire.
 */

#ifndef HW_TEST_HELPER_H
#define HW_TEST_HELPER_H

#include <Wire.h>

// Boards where STEMMA QT is on Wire1
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) || \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||    \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
  #define HW_TEST_WIRE Wire1
  #define HW_TEST_I2C_INIT() do { Wire1.begin(SDA1, SCL1); Wire1.setClock(400000); } while(0)
#else
  #define HW_TEST_WIRE Wire
  #define HW_TEST_I2C_INIT() do { Wire.begin(); Wire.setClock(400000); } while(0)
#endif

#endif // HW_TEST_HELPER_H
