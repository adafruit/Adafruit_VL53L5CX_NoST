/*!
 * @file Adafruit_VL53L5CX_NoST.h
 *
 * Lightweight BusIO-native driver for VL53L5CX 8x8 ToF sensor.
 * Does not depend on the ST Ultra Lite Driver C code.
 * Still requires the firmware/config/xtalk blobs from vl53l5cx_buffers.h.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 * MIT license
 */

#ifndef ADAFRUIT_VL53L5CX_NOST_H
#define ADAFRUIT_VL53L5CX_NOST_H

#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

// Firmware and config blobs (still needed — sensor has no internal ROM)
#include "vl53l5cx_buffers.h"

#define VL53L5_DEFAULT_ADDRESS 0x29

// Resolution constants
#define VL53L5_RESOLUTION_4X4 16
#define VL53L5_RESOLUTION_8X8 64

// Target order
#define VL53L5_TARGET_ORDER_CLOSEST 1
#define VL53L5_TARGET_ORDER_STRONGEST 2

// Ranging mode
#define VL53L5_RANGING_MODE_CONTINUOUS 1
#define VL53L5_RANGING_MODE_AUTONOMOUS 3

// Power mode
#define VL53L5_POWER_MODE_SLEEP 0
#define VL53L5_POWER_MODE_WAKEUP 1

// Status codes
#define VL53L5_STATUS_OK 0
#define VL53L5_STATUS_ERROR 1
#define VL53L5_STATUS_TIMEOUT 2

// Internal constants
#define VL53L5_PAGE_REG 0x7FFF
#define VL53L5_UI_CMD_STATUS 0x2C00
#define VL53L5_UI_CMD_START 0x2C04
#define VL53L5_UI_CMD_END 0x2FFF

// DCI addresses
#define VL53L5_DCI_ZONE_CONFIG 0x5450
#define VL53L5_DCI_FREQ_HZ 0x5458
#define VL53L5_DCI_INT_TIME 0x545C
#define VL53L5_DCI_FW_NB_TARGET 0x5478
#define VL53L5_DCI_RANGING_MODE 0xAD30
#define VL53L5_DCI_DSS_CONFIG 0xAD38
#define VL53L5_DCI_TARGET_ORDER 0xAE64
#define VL53L5_DCI_SHARPENER 0xAED8
#define VL53L5_DCI_INTERNAL_CP 0xB39C
#define VL53L5_DCI_SINGLE_RANGE 0xD964
#define VL53L5_DCI_OUTPUT_CONFIG 0xD968
#define VL53L5_DCI_OUTPUT_ENABLES 0xD970
#define VL53L5_DCI_OUTPUT_LIST 0xD980
#define VL53L5_DCI_PIPE_CONTROL 0xDB80

// Glare filter DCI address
#define VL53L5_GLARE_FILTER 0xCD5C

// Number of targets per zone
#define VL53L5CX_NB_TARGET_PER_ZONE 1

// Output block headers (1 target per zone)
#define VL53L5CX_START_BH 0x0000000DU
#define VL53L5CX_METADATA_BH 0x54B400C0U
#define VL53L5CX_COMMONDATA_BH 0x54C00040U
#define VL53L5CX_AMBIENT_RATE_BH 0x54D00104U
#define VL53L5CX_SPAD_COUNT_BH 0x55D00404U
#define VL53L5CX_NB_TARGET_DETECTED_BH 0xDB840401U
#define VL53L5CX_SIGNAL_RATE_BH 0xDBC40404U
#define VL53L5CX_RANGE_SIGMA_MM_BH 0xDEC40402U
#define VL53L5CX_DISTANCE_BH 0xDF440402U
#define VL53L5CX_REFLECTANCE_BH 0xE0440401U
#define VL53L5CX_TARGET_STATUS_BH 0xE0840401U
#define VL53L5CX_MOTION_DETECT_BH 0xD85808C0U

// Block header union for parsing output block descriptors
union Block_header {
  uint32_t bytes;
  struct {
    uint32_t type : 4;
    uint32_t size : 12;
    uint32_t idx : 16;
  };
};

// NVM/calibration sizes
#define VL53L5_NVM_DATA_SIZE 492
#define VL53L5_OFFSET_BUFFER_SIZE 488
#define VL53L5_XTALK_BUFFER_SIZE 776

// Max I2C chunk
// VL53L5_I2C_CHUNK is now determined at runtime via _i2c->maxBufferSize()

// Temp buffer — shared for DCI ops. Minimum 1024, or max results size.
// For lite driver we use a fixed 1024 byte buffer.
#define VL53L5_TEMP_BUFFER_SIZE 2048

/*!
 * @brief Lightweight driver for VL53L5CX ToF sensor using Adafruit BusIO
 */
class Adafruit_VL53L5CX_NoST {
public:
  Adafruit_VL53L5CX_NoST();
  ~Adafruit_VL53L5CX_NoST();

  bool begin(uint8_t address = VL53L5_DEFAULT_ADDRESS,
             TwoWire *wire = &Wire);

  // Ranging control
  bool startRanging();
  bool stopRanging();
  bool isDataReady();

  /*!
   * @brief Read ranging data. Distances in mm, statuses per zone.
   * @param distances Array of int16_t, size = resolution (16 or 64)
   * @param statuses Optional array of uint8_t target status per zone
   * @param sigmas Optional array of uint16_t sigma (noise) per zone
   * @return true on success
   */
  bool getRangingData(int16_t *distances, uint8_t *statuses = nullptr,
                      uint16_t *sigmas = nullptr);

  // Configuration
  bool setResolution(uint8_t res);
  uint8_t getResolution();

  bool setRangingFrequency(uint8_t hz);
  uint8_t getRangingFrequency();

  bool setIntegrationTime(uint32_t ms);
  uint32_t getIntegrationTime();

  bool setSharpenerPercent(uint8_t pct);
  uint8_t getSharpenerPercent();

  bool setTargetOrder(uint8_t order);
  uint8_t getTargetOrder();

  bool setRangingMode(uint8_t mode);
  uint8_t getRangingMode();

  bool setPowerMode(uint8_t mode);
  uint8_t getPowerMode();

  bool setAddress(uint8_t new_address);

  //! Get last silicon temperature reading (from most recent getRangingData)
  int8_t getTemperature() { return _temperature; }

private:
  Adafruit_I2CDevice *_i2c = nullptr;
  uint8_t _streamcount = 255;
  uint32_t _data_read_size = 0;
  uint8_t _resolution = VL53L5_RESOLUTION_4X4;
  int8_t _temperature = 0;
  bool _initialized = false;

  // Shared temp buffer for DCI operations
  uint8_t _temp[VL53L5_TEMP_BUFFER_SIZE];

  // Offset and xtalk calibration data (read from NVM during init)
  uint8_t _offset_data[VL53L5_OFFSET_BUFFER_SIZE];
  uint8_t _xtalk_data[VL53L5_XTALK_BUFFER_SIZE];

  // Low-level I2C
  bool _writeByte(uint16_t reg, uint8_t val);
  bool _readByte(uint16_t reg, uint8_t *val);
  bool _writeMulti(uint16_t reg, const uint8_t *data, uint32_t len);
  bool _readMulti(uint16_t reg, uint8_t *data, uint32_t len);
  void _setPage(uint8_t page);

  // DCI mailbox protocol
  bool _dciRead(uint16_t index, uint8_t *data, uint16_t size);
  bool _dciWrite(const uint8_t *data, uint16_t index, uint16_t size);
  bool _dciReplace(uint16_t index, uint16_t total_size, const uint8_t *data,
                   uint16_t data_size, uint16_t offset);

  // Polling
  bool _pollForAnswer(uint8_t size, uint8_t pos, uint16_t reg, uint8_t mask,
                      uint8_t expected);
  bool _pollForBoot();

  // Init sub-steps
  bool _swReboot();
  bool _uploadFirmware();
  bool _loadNVMAndCalibration();
  bool _sendDefaultConfig();
  bool _sendOffsetData(uint8_t resolution);
  bool _sendXtalkData(uint8_t resolution);

  // Data parsing helpers
  void _swapBuffer(uint8_t *buf, uint16_t size);
};

#endif // ADAFRUIT_VL53L5CX_NOST_H
