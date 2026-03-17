/*!
 * @file Adafruit_VL53L5CX_NoST.cpp
 *
 * Lightweight BusIO-native driver for VL53L5CX 8x8 ToF sensor.
 * Reimplements the ST ULD protocol using only Adafruit_I2CDevice.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 * MIT license
 */

#include "Adafruit_VL53L5CX_NoST.h"

// Block header indices for output data parsing
// Block header IDX values for output data parsing (from ST ULD)
#define VL53L5CX_METADATA_IDX 0x54B4U
#define VL53L5CX_DISTANCE_IDX 0xDF44U
#define VL53L5CX_TARGET_STATUS_IDX 0xE084U
#define VL53L5CX_RANGE_SIGMA_MM_IDX 0xDEC4U

// Glare filter DCI address
#define VL53L5_GLARE_FILTER 0xB870

// ============================================================
// Constructor / Destructor
// ============================================================

Adafruit_VL53L5CX_NoST::Adafruit_VL53L5CX_NoST() {}

Adafruit_VL53L5CX_NoST::~Adafruit_VL53L5CX_NoST() {
  if (_i2c) {
    delete _i2c;
  }
}

// ============================================================
// Low-level I2C helpers
// ============================================================

bool Adafruit_VL53L5CX_NoST::_writeByte(uint16_t reg, uint8_t val) {
  uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val};
  return _i2c->write(buf, 3);
}

bool Adafruit_VL53L5CX_NoST::_readByte(uint16_t reg, uint8_t *val) {
  uint8_t r[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  return _i2c->write_then_read(r, 2, val, 1);
}

bool Adafruit_VL53L5CX_NoST::_writeMulti(uint16_t reg, const uint8_t *data,
                                        uint32_t len) {
  uint32_t maxPayload = _i2c->maxBufferSize() - 2; // reserve 2 for reg addr
  uint8_t buf[maxPayload + 2];
  uint32_t offset = 0;

  while (offset < len) {
    uint32_t chunk = min(maxPayload, len - offset);
    uint16_t addr = reg + offset;
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFF);
    memcpy(buf + 2, data + offset, chunk);
    if (!_i2c->write(buf, chunk + 2)) {
      return false;
    }
    offset += chunk;
  }
  return true;
}

bool Adafruit_VL53L5CX_NoST::_readMulti(uint16_t reg, uint8_t *data,
                                       uint32_t len) {
  uint32_t maxRead = _i2c->maxBufferSize();
  uint32_t offset = 0;

  while (offset < len) {
    uint32_t chunk = min(maxRead, len - offset);
    uint16_t addr = reg + offset;
    uint8_t r[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
    if (!_i2c->write_then_read(r, 2, data + offset, chunk)) {
      return false;
    }
    offset += chunk;
  }
  return true;
}

void Adafruit_VL53L5CX_NoST::_setPage(uint8_t page) {
  _writeByte(VL53L5_PAGE_REG, page);
}

// ============================================================
// Byte-swap for big-endian sensor data on little-endian host
// ============================================================

void Adafruit_VL53L5CX_NoST::_swapBuffer(uint8_t *buf, uint16_t size) {
  for (uint16_t i = 0; i < size; i += 4) {
    uint32_t tmp = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i + 1] << 16) |
                   ((uint32_t)buf[i + 2] << 8) | (uint32_t)buf[i + 3];
    memcpy(&buf[i], &tmp, 4);
  }
}

// ============================================================
// Polling helpers
// ============================================================

bool Adafruit_VL53L5CX_NoST::_pollForAnswer(uint8_t size, uint8_t pos,
                                           uint16_t reg, uint8_t mask,
                                           uint8_t expected) {
  uint16_t timeout = 0;
  uint8_t status;

  do {
    if (!_readMulti(reg, _temp, size)) {
      return false;
    }
    if (size >= 4 && _temp[2] >= 0x7F) {
      return false; // MCU error
    }
    if ((_temp[pos] & mask) == expected) {
      return true;
    }
    delay(1);
    timeout++;
  } while (timeout < 2000);

  return false; // Timeout
}

bool Adafruit_VL53L5CX_NoST::_pollForBoot() {
  uint8_t go2_status0, go2_status1;
  uint16_t timeout = 0;

  do {
    if (!_readByte(0x06, &go2_status0)) {
      return false;
    }
    if (go2_status0 & 0x80) {
      _readByte(0x07, &go2_status1);
      return (go2_status1 == 0);
    }
    if (go2_status0 & 0x01) {
      return true;
    }
    delay(1);
    timeout++;
  } while (timeout < 500);

  return false;
}

// ============================================================
// DCI (Device Configuration Interface) protocol
// ============================================================

bool Adafruit_VL53L5CX_NoST::_dciRead(uint16_t index, uint8_t *data,
                                     uint16_t size) {
  uint32_t rd_size = (uint32_t)size + 12;
  if (rd_size > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }

  uint8_t cmd[12] = {
      (uint8_t)(index >> 8),
      (uint8_t)(index & 0xFF),
      (uint8_t)((size & 0xFF0) >> 4),
      (uint8_t)((size & 0x0F) << 4),
      0x00, 0x00, 0x00, 0x0F, 0x00, 0x02, 0x00, 0x08};

  if (!_writeMulti(VL53L5_UI_CMD_END - 11, cmd, sizeof(cmd))) {
    return false;
  }
  if (!_pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03)) {
    return false;
  }
  if (!_readMulti(VL53L5_UI_CMD_START, _temp, rd_size)) {
    return false;
  }
  _swapBuffer(_temp, size + 12);
  memmove(data, _temp + 4, size);  // data may be _temp (overlapping)
  return true;
}

bool Adafruit_VL53L5CX_NoST::_dciWrite(const uint8_t *data, uint16_t index,
                                      uint16_t size) {
  // Mirrors vl53l5cx_dci_write_data exactly:
  // Build [header(4) | swapped_data(size) | footer(8)] in _temp,
  // then write as one contiguous block.

  if ((size + 12) > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }

  uint16_t address = VL53L5_UI_CMD_END - (size + 12) + 1;

  // Headers: raw (NOT swapped), matching ST exactly
  uint8_t headers[4] = {
      (uint8_t)(index >> 8),
      (uint8_t)(index & 0xFF),
      (uint8_t)((size & 0xFF0) >> 4),
      (uint8_t)((size & 0x0F) << 4)};

  uint8_t footer[8] = {0x00, 0x00, 0x00, 0x0F, 0x05, 0x01,
                        (uint8_t)((size + 8) >> 8),
                        (uint8_t)((size + 8) & 0xFF)};

  // Swap data in a local copy (ST swaps the input buffer in place and
  // restores it after; we copy to _temp to avoid mutating caller's data)
  // Use memmove because _dciReplace passes data=_temp (overlapping)
  memmove(&_temp[4], data, size);
  _swapBuffer(&_temp[4], size);

  // Assemble frame: header + swapped data + footer
  memcpy(&_temp[0], headers, 4);
  memcpy(&_temp[4 + size], footer, 8);

  // Single write of the entire frame
  if (!_writeMulti(address, _temp, size + 12)) {
    return false;
  }

  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5CX_NoST::_dciReplace(uint16_t index, uint16_t total_size,
                                        const uint8_t *data,
                                        uint16_t data_size, uint16_t offset) {
  // Read current value
  if (!_dciRead(index, _temp, total_size)) {
    return false;
  }
  // Replace bytes at offset
  memcpy(_temp + offset, data, data_size);
  // Write back
  return _dciWrite(_temp, index, total_size);
}

// ============================================================
// Init sequence
// ============================================================

bool Adafruit_VL53L5CX_NoST::_swReboot() {
  uint8_t tmp;

  _setPage(0x00);
  _writeByte(0x0009, 0x04);
  _writeByte(0x000F, 0x40);
  _writeByte(0x000A, 0x03);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x000C, 0x01);

  _writeByte(0x0101, 0x00);
  _writeByte(0x0102, 0x00);
  _writeByte(0x010A, 0x01);
  _writeByte(0x4002, 0x01);
  _writeByte(0x4002, 0x00);
  _writeByte(0x010A, 0x03);
  _writeByte(0x0103, 0x01);
  _writeByte(0x000C, 0x00);
  _writeByte(0x000F, 0x43);
  delay(1);

  _writeByte(0x000F, 0x40);
  _writeByte(0x000A, 0x01);
  delay(100);

  // Wait for sensor boot
  _setPage(0x00);
  if (!_pollForAnswer(1, 0, 0x06, 0xFF, 1)) {
    return false;
  }

  _writeByte(0x000E, 0x01);
  _setPage(0x02);

  // Enable FW access
  _writeByte(0x03, 0x0D);
  _setPage(0x01);
  if (!_pollForAnswer(1, 0, 0x21, 0x10, 0x10)) {
    return false;
  }
  _setPage(0x00);

  // Enable host access to GO1
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x01);

  // Power ON status
  _setPage(0x00);
  _writeByte(0x101, 0x00);
  _writeByte(0x102, 0x00);
  _writeByte(0x010A, 0x01);
  _writeByte(0x4002, 0x01);
  _writeByte(0x4002, 0x00);
  _writeByte(0x010A, 0x03);
  _writeByte(0x103, 0x01);
  _writeByte(0x400F, 0x00);
  _writeByte(0x21A, 0x43);
  _writeByte(0x21A, 0x03);
  _writeByte(0x21A, 0x01);
  _writeByte(0x21A, 0x00);
  _writeByte(0x219, 0x00);
  _writeByte(0x21B, 0x00);

  // Wake up MCU
  _setPage(0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x00);
  _setPage(0x01);
  _writeByte(0x20, 0x07);
  _writeByte(0x20, 0x06);

  return true;
}

bool Adafruit_VL53L5CX_NoST::_uploadFirmware() {
  // Upload firmware in 3 chunks across pages 0x09, 0x0A, 0x0B
  _setPage(0x09);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0], 0x8000)) {
    return false;
  }
  _setPage(0x0A);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0x8000], 0x8000)) {
    return false;
  }
  _setPage(0x0B);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0x10000], 0x5000)) {
    return false;
  }
  _setPage(0x01);

  // Verify FW download
  _setPage(0x02);
  _writeByte(0x03, 0x0D);
  _setPage(0x01);
  if (!_pollForAnswer(1, 0, 0x21, 0x10, 0x10)) {
    return false;
  }

  uint8_t tmp;
  _setPage(0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x01);

  // Reset MCU and wait boot
  _setPage(0x00);
  _writeByte(0x114, 0x00);
  _writeByte(0x115, 0x00);
  _writeByte(0x116, 0x42);
  _writeByte(0x117, 0x00);
  _writeByte(0x0B, 0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x00);
  _writeByte(0x0B, 0x01);

  if (!_pollForBoot()) {
    return false;
  }

  _setPage(0x02);
  return true;
}

bool Adafruit_VL53L5CX_NoST::_sendOffsetData(uint8_t resolution) {
  // Mirrors _vl53l5cx_send_offset_data in the ST ULD exactly
  uint32_t signal_grid[64];
  int16_t range_grid[64];
  uint8_t dss_4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
  uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};

  memcpy(_temp, _offset_data, VL53L5_OFFSET_BUFFER_SIZE);

  if (resolution == VL53L5_RESOLUTION_4X4) {
    memcpy(&_temp[0x10], dss_4x4, sizeof(dss_4x4));
    _swapBuffer(_temp, VL53L5_OFFSET_BUFFER_SIZE);
    memcpy(signal_grid, &_temp[0x3C], sizeof(signal_grid));
    memcpy(range_grid, &_temp[0x140], sizeof(range_grid));

    for (int8_t j = 0; j < 4; j++) {
      for (int8_t i = 0; i < 4; i++) {
        signal_grid[i + (4 * j)] =
            (signal_grid[(2 * i) + (16 * j) + 0] +
             signal_grid[(2 * i) + (16 * j) + 1] +
             signal_grid[(2 * i) + (16 * j) + 8] +
             signal_grid[(2 * i) + (16 * j) + 9]) /
            (uint32_t)4;
        range_grid[i + (4 * j)] =
            (range_grid[(2 * i) + (16 * j)] +
             range_grid[(2 * i) + (16 * j) + 1] +
             range_grid[(2 * i) + (16 * j) + 8] +
             range_grid[(2 * i) + (16 * j) + 9]) /
            (int16_t)4;
      }
    }
    memset(&range_grid[0x10], 0, 96);
    memset(&signal_grid[0x10], 0, 192);
    memcpy(&_temp[0x3C], signal_grid, sizeof(signal_grid));
    memcpy(&_temp[0x140], range_grid, sizeof(range_grid));
    _swapBuffer(_temp, VL53L5_OFFSET_BUFFER_SIZE);
  }

  // Shift buffer left by 8 bytes (strip DCI header)
  for (uint16_t k = 0; k < (VL53L5_OFFSET_BUFFER_SIZE - 4); k++) {
    _temp[k] = _temp[k + 8];
  }
  // Append footer
  memcpy(&_temp[0x1E0], footer, 8);

  if (!_writeMulti(0x2E18, _temp, VL53L5_OFFSET_BUFFER_SIZE)) {
    return false;
  }
  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5CX_NoST::_sendXtalkData(uint8_t resolution) {
  // Mirrors _vl53l5cx_send_xtalk_data in the ST ULD exactly
  uint8_t res4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
  uint8_t dss_4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
  uint8_t profile_4x4[] = {0xA0, 0xFC, 0x01, 0x00};
  uint32_t signal_grid[64];

  memcpy(_temp, _xtalk_data, VL53L5_XTALK_BUFFER_SIZE);

  if (resolution == VL53L5_RESOLUTION_4X4) {
    memcpy(&_temp[0x08], res4x4, sizeof(res4x4));
    memcpy(&_temp[0x20], dss_4x4, sizeof(dss_4x4));

    _swapBuffer(_temp, VL53L5_XTALK_BUFFER_SIZE);
    memcpy(signal_grid, &_temp[0x34], sizeof(signal_grid));

    for (int8_t j = 0; j < 4; j++) {
      for (int8_t i = 0; i < 4; i++) {
        signal_grid[i + (4 * j)] =
            (signal_grid[(2 * i) + (16 * j) + 0] +
             signal_grid[(2 * i) + (16 * j) + 1] +
             signal_grid[(2 * i) + (16 * j) + 8] +
             signal_grid[(2 * i) + (16 * j) + 9]) /
            (uint32_t)4;
      }
    }
    memset(&signal_grid[0x10], 0, 192);
    memcpy(&_temp[0x34], signal_grid, sizeof(signal_grid));
    _swapBuffer(_temp, VL53L5_XTALK_BUFFER_SIZE);
    memcpy(&_temp[0x134], profile_4x4, sizeof(profile_4x4));
    memset(&_temp[0x078], 0, 4);
  }

  // ST writes directly to 0x2CF8, no footer, no shift
  if (!_writeMulti(0x2CF8, _temp, VL53L5_XTALK_BUFFER_SIZE)) {
    return false;
  }
  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5CX_NoST::_loadNVMAndCalibration() {
  if (!_writeMulti(0x2FD8, (uint8_t *)VL53L5CX_GET_NVM_CMD,
                   sizeof(VL53L5CX_GET_NVM_CMD))) {
    return false;
  }
  if (!_pollForAnswer(4, 0, VL53L5_UI_CMD_STATUS, 0xFF, 2)) {
    return false;
  }
  if (!_readMulti(VL53L5_UI_CMD_START, _temp, VL53L5_NVM_DATA_SIZE)) {
    return false;
  }
  memcpy(_offset_data, _temp, VL53L5_OFFSET_BUFFER_SIZE);

  if (!_sendOffsetData(VL53L5_RESOLUTION_4X4)) {
    return false;
  }

  memcpy(_xtalk_data, (uint8_t *)VL53L5CX_DEFAULT_XTALK,
         VL53L5_XTALK_BUFFER_SIZE);
  if (!_sendXtalkData(VL53L5_RESOLUTION_4X4)) {
    return false;
  }
  return true;
}

bool Adafruit_VL53L5CX_NoST::_sendDefaultConfig() {
  if (!_writeMulti(0x2C34, (uint8_t *)VL53L5CX_DEFAULT_CONFIGURATION,
                   sizeof(VL53L5CX_DEFAULT_CONFIGURATION))) {
    return false;
  }
  if (!_pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03)) {
    return false;
  }

  uint8_t pipe_ctrl[] = {1, 0x00, 0x01, 0x00};
  if (!_dciWrite(pipe_ctrl, VL53L5_DCI_PIPE_CONTROL, sizeof(pipe_ctrl))) {
    return false;
  }

  uint32_t single_range = 0x01;
  if (!_dciWrite((uint8_t *)&single_range, VL53L5_DCI_SINGLE_RANGE, 4)) {
    return false;
  }

  uint8_t one = 1;
  if (!_dciReplace(VL53L5_GLARE_FILTER, 40, &one, 1, 0x26)) {
    return false;
  }
  if (!_dciReplace(VL53L5_GLARE_FILTER, 40, &one, 1, 0x25)) {
    return false;
  }
  return true;
}

// ============================================================
// Public API
// ============================================================

bool Adafruit_VL53L5CX_NoST::begin(uint8_t address, TwoWire *wire) {
  if (_i2c) {
    delete _i2c;
  }
  _i2c = new Adafruit_I2CDevice(address, wire);
  if (!_i2c->begin()) {
    return false;
  }

  // Check sensor is alive
  _setPage(0x00);
  uint8_t device_id = 0, revision_id = 0;
  _readByte(0x00, &device_id);
  _readByte(0x01, &revision_id);
  _setPage(0x02);


  if (device_id != 0xF0 || revision_id != 0x02) {
    return false;
  }

  if (!_swReboot()) {
    return false;
  }

  if (!_uploadFirmware()) {
    return false;
  }

  if (!_loadNVMAndCalibration()) {
    return false;
  }

  if (!_sendDefaultConfig()) {
    return false;
  }

  _initialized = true;
  _resolution = VL53L5_RESOLUTION_4X4;
  return true;
}

bool Adafruit_VL53L5CX_NoST::startRanging() {
  if (!_initialized) {
    return false;
  }

  // Mirrors vl53l5cx_start_ranging exactly
  uint8_t resolution = getResolution();
  _data_read_size = 0;
  _streamcount = 255;

  uint32_t header_config[2] = {0, 0};
  uint8_t cmd[] = {0x00, 0x03, 0x00, 0x00};

  // Output block headers — must match VL53L5CX_*_BH defines (1 target/zone)
  uint32_t output[] = {
      VL53L5CX_START_BH,
      VL53L5CX_METADATA_BH,
      VL53L5CX_COMMONDATA_BH,
      VL53L5CX_AMBIENT_RATE_BH,
      VL53L5CX_SPAD_COUNT_BH,
      VL53L5CX_NB_TARGET_DETECTED_BH,
      VL53L5CX_SIGNAL_RATE_BH,
      VL53L5CX_RANGE_SIGMA_MM_BH,
      VL53L5CX_DISTANCE_BH,
      VL53L5CX_REFLECTANCE_BH,
      VL53L5CX_TARGET_STATUS_BH,
      VL53L5CX_MOTION_DETECT_BH};

  // Enable bits: mandatory (meta+common+start) + all optional outputs
  uint32_t output_bh_enable[4] = {0x00000007, 0x00000000, 0x00000000,
                                   0xC0000000};
  // Enable ambient(8) + spad(16) + nb_target(32) + signal(64) +
  // sigma(128) + distance(256) + reflectance(512) + status(1024) +
  // motion(2048)
  output_bh_enable[0] += 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;

  // Compute data_read_size from block headers (ST algorithm)
  uint32_t i;
  for (i = 0; i < (sizeof(output) / sizeof(uint32_t)); i++) {
    if (output[i] == 0 ||
        (output_bh_enable[i / 32] & ((uint32_t)1 << (i % 32))) == 0) {
      continue;
    }
    union Block_header *bh = (union Block_header *)&output[i];
    if (bh->type >= 0x1 && bh->type < 0x0d) {
      if (bh->idx >= 0x54d0 && bh->idx < (0x54d0 + 960)) {
        bh->size = resolution;
      } else {
        bh->size = resolution * VL53L5CX_NB_TARGET_PER_ZONE;
      }
      _data_read_size += bh->type * bh->size;
    } else {
      _data_read_size += bh->size;
    }
    _data_read_size += 4;
  }
  _data_read_size += 24;

  // Write output list, config, and enables via DCI
  if (!_dciWrite((uint8_t *)output, VL53L5_DCI_OUTPUT_LIST,
                 sizeof(output))) {
    return false;
  }

  header_config[0] = _data_read_size;
  header_config[1] = i + 1;
  if (!_dciWrite((uint8_t *)header_config, VL53L5_DCI_OUTPUT_CONFIG,
                 sizeof(header_config))) {
    return false;
  }

  if (!_dciWrite((uint8_t *)output_bh_enable, VL53L5_DCI_OUTPUT_ENABLES,
                 sizeof(output_bh_enable))) {
    return false;
  }

  // Start xshut bypass (interrupt mode)
  _setPage(0x00);
  _writeByte(0x09, 0x05);
  _setPage(0x02);

  // Start ranging session
  if (!_writeMulti(VL53L5_UI_CMD_END - 3, cmd, sizeof(cmd))) {
    return false;
  }
  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5CX_NoST::stopRanging() {
  if (!_initialized) {
    return false;
  }

  // Mirrors vl53l5cx_stop_ranging exactly
  uint8_t tmp = 0;
  uint16_t timeout = 0;
  uint32_t auto_stop_flag = 0;

  _readMulti(0x2FFC, (uint8_t *)&auto_stop_flag, 4);

  if (auto_stop_flag != 0x4FF) {
    _setPage(0x00);

    // Provoke MCU stop
    _writeByte(0x15, 0x16);
    _writeByte(0x14, 0x01);

    // Poll for GO2 status bit 7 set (MCU stopped)
    while (((tmp & 0x80) >> 7) == 0x00) {
      _readByte(0x06, &tmp);
      delay(10);
      timeout++;
      if (timeout > 500) {
        break;
      }
    }
  }

  // Check GO2 status 1
  _readByte(0x06, &tmp);
  if ((tmp & 0x80) != 0) {
    _readByte(0x07, &tmp);
    // Status 0x84 and 0x85 are OK
  }

  // Undo MCU stop
  _setPage(0x00);
  _writeByte(0x14, 0x00);
  _writeByte(0x15, 0x00);

  // Stop xshut bypass
  _writeByte(0x09, 0x04);
  _setPage(0x02);

  return true;
}

bool Adafruit_VL53L5CX_NoST::isDataReady() {
  if (!_initialized) {
    return false;
  }

  uint8_t buf[4];
  if (!_readMulti(0x0000, buf, 4)) {
    return false;
  }

  if ((buf[0] != _streamcount) && (buf[0] != 255) && (buf[1] == 0x05) &&
      ((buf[2] & 0x05) == 0x05) && ((buf[3] & 0x10) == 0x10)) {
    _streamcount = buf[0];
    return true;
  }
  return false;
}

bool Adafruit_VL53L5CX_NoST::getRangingData(int16_t *distances,
                                            uint8_t *statuses,
                                            uint16_t *sigmas) {
  if (!_initialized || _data_read_size == 0 ||
      _data_read_size > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }

  // Read raw data — mirrors vl53l5cx_get_ranging_data exactly
  if (!_readMulti(0x0000, _temp, _data_read_size)) {
    return false;
  }
  _streamcount = _temp[0];
  _swapBuffer(_temp, (uint16_t)_data_read_size);

  uint8_t zones =
      (_resolution == VL53L5_RESOLUTION_8X8) ? 64 : 16;

  // Parse tagged blocks using Block_header union (same as ST)
  for (uint32_t i = 16; i < _data_read_size; i += 4) {
    union Block_header *bh = (union Block_header *)&_temp[i];
    uint32_t msize;

    if (bh->type > 0x1 && bh->type < 0xd) {
      msize = bh->type * bh->size;
    } else {
      msize = bh->size;
    }

    switch (bh->idx) {
    case VL53L5CX_METADATA_IDX:
      _temperature = (int8_t)_temp[i + 12];
      break;
    case VL53L5CX_DISTANCE_IDX:
      if (distances) {
        memcpy(distances, &_temp[i + 4], msize);
      }
      break;
    case VL53L5CX_TARGET_STATUS_IDX:
      if (statuses) {
        memcpy(statuses, &_temp[i + 4], msize);
      }
      break;
    case VL53L5CX_RANGE_SIGMA_MM_IDX:
      if (sigmas) {
        memcpy(sigmas, &_temp[i + 4], msize);
      }
      break;
    default:
      break;
    }
    i += msize;
  }
  return true;
}

// ============================================================
// Configuration get/set via DCI
// ============================================================

bool Adafruit_VL53L5CX_NoST::setResolution(uint8_t res) {
  if (!_initialized ||
      (res != VL53L5_RESOLUTION_4X4 && res != VL53L5_RESOLUTION_8X8)) {
    return false;
  }

  // Mirrors vl53l5cx_set_resolution: read-modify-write DSS and zone config
  uint8_t dss[16], zone[8];

  if (!_dciRead(VL53L5_DCI_DSS_CONFIG, dss, 16)) {
    return false;
  }
  if (!_dciRead(VL53L5_DCI_ZONE_CONFIG, zone, 8)) {
    return false;
  }

  if (res == VL53L5_RESOLUTION_4X4) {
    dss[0x04] = 64;
    dss[0x06] = 64;
    dss[0x09] = 4;
    zone[0x00] = 4;
    zone[0x01] = 4;
    zone[0x04] = 8;
    zone[0x05] = 8;
  } else {
    dss[0x04] = 16;
    dss[0x06] = 16;
    dss[0x09] = 1;
    zone[0x00] = 8;
    zone[0x01] = 8;
    zone[0x04] = 4;
    zone[0x05] = 4;
  }

  if (!_dciWrite(dss, VL53L5_DCI_DSS_CONFIG, 16)) {
    return false;
  }
  if (!_dciWrite(zone, VL53L5_DCI_ZONE_CONFIG, 8)) {
    return false;
  }

  _sendOffsetData(res);
  _sendXtalkData(res);
  _resolution = res;
  return true;
}

uint8_t Adafruit_VL53L5CX_NoST::getResolution() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_ZONE_CONFIG, buf, 8)) {
    return 0;
  }
  return buf[0] * buf[1]; // width * height
}

bool Adafruit_VL53L5CX_NoST::setRangingFrequency(uint8_t hz) {
  if (!_initialized) {
    return false;
  }
  return _dciReplace(VL53L5_DCI_FREQ_HZ, 4, &hz, 1, 0x01);
}

uint8_t Adafruit_VL53L5CX_NoST::getRangingFrequency() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[4] = {0};
  if (!_dciRead(VL53L5_DCI_FREQ_HZ, buf, 4)) {
    return 0;
  }
  return buf[0x01];
}

bool Adafruit_VL53L5CX_NoST::setIntegrationTime(uint32_t ms) {
  if (!_initialized) {
    return false;
  }
  // Integration time is at offset 0 in a 20-byte DCI block
  // Value is in microseconds (* 1000)
  uint32_t us = ms * 1000;
  return _dciReplace(VL53L5_DCI_INT_TIME, 20, (uint8_t *)&us, 4, 0);
}

uint32_t Adafruit_VL53L5CX_NoST::getIntegrationTime() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[20];
  if (!_dciRead(VL53L5_DCI_INT_TIME, buf, 20)) {
    return 0;
  }
  uint32_t us;
  memcpy(&us, buf, 4);
  return us / 1000; // Convert to ms
}

bool Adafruit_VL53L5CX_NoST::setSharpenerPercent(uint8_t pct) {
  if (!_initialized || pct >= 100) {
    return false;
  }
  // Mirrors vl53l5cx_set_sharpener_percent: 1 byte at offset 0x0D
  uint8_t sharpener = (uint8_t)(((uint16_t)pct * 255) / 100);
  return _dciReplace(VL53L5_DCI_SHARPENER, 16, &sharpener, 1, 0x0D);
}

uint8_t Adafruit_VL53L5CX_NoST::getSharpenerPercent() {
  if (!_initialized) {
    return 0xFF;
  }
  uint8_t buf[16];
  if (!_dciRead(VL53L5_DCI_SHARPENER, buf, 16)) {
    return 0xFF;
  }
  return (uint8_t)(((uint16_t)buf[0x0D] * 100) / 255);
}

bool Adafruit_VL53L5CX_NoST::setTargetOrder(uint8_t order) {
  if (!_initialized ||
      (order != VL53L5_TARGET_ORDER_CLOSEST &&
       order != VL53L5_TARGET_ORDER_STRONGEST)) {
    return false;
  }
  return _dciReplace(VL53L5_DCI_TARGET_ORDER, 4, &order, 1, 0x00);
}

uint8_t Adafruit_VL53L5CX_NoST::getTargetOrder() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[4] = {0};
  if (!_dciRead(VL53L5_DCI_TARGET_ORDER, buf, 4)) {
    return 0;
  }
  return buf[0x00];
}

bool Adafruit_VL53L5CX_NoST::setRangingMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  // Mirrors vl53l5cx_set_ranging_mode
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_RANGING_MODE, buf, 8)) {
    return false;
  }

  uint32_t single_range;
  switch (mode) {
  case VL53L5_RANGING_MODE_CONTINUOUS:
    buf[0x01] = 0x01;
    buf[0x03] = 0x03;
    single_range = 0x00;
    break;
  case VL53L5_RANGING_MODE_AUTONOMOUS:
    buf[0x01] = 0x03;
    buf[0x03] = 0x02;
    single_range = 0x01;
    break;
  default:
    return false;
  }

  if (!_dciWrite(buf, VL53L5_DCI_RANGING_MODE, 8)) {
    return false;
  }
  return _dciWrite((uint8_t *)&single_range, VL53L5_DCI_SINGLE_RANGE, 4);
}

uint8_t Adafruit_VL53L5CX_NoST::getRangingMode() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_RANGING_MODE, buf, 8)) {
    return 0;
  }
  // ST stores mode encoding in buf[0x01]: 0x01=continuous, 0x03=autonomous
  return (buf[0x01] == 0x03) ? VL53L5_RANGING_MODE_AUTONOMOUS
                              : VL53L5_RANGING_MODE_CONTINUOUS;
}

bool Adafruit_VL53L5CX_NoST::setPowerMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  uint8_t current = getPowerMode();
  if (current == mode) {
    return true;
  }

  if (mode == VL53L5_POWER_MODE_WAKEUP) {
    _setPage(0x00);
    _writeByte(0x09, 0x04);
    if (!_pollForAnswer(1, 0, 0x06, 0x01, 1)) {
      return false;
    }
  } else if (mode == VL53L5_POWER_MODE_SLEEP) {
    _setPage(0x00);
    _writeByte(0x09, 0x02);
    if (!_pollForAnswer(1, 0, 0x06, 0x01, 0)) {
      return false;
    }
  } else {
    return false;
  }

  _setPage(0x02);
  return true;
}

uint8_t Adafruit_VL53L5CX_NoST::getPowerMode() {
  if (!_initialized) {
    return 0xFF;
  }

  uint8_t tmp;
  _setPage(0x00);
  _readByte(0x009, &tmp);
  _setPage(0x02);

  if (tmp == 0x04) {
    return VL53L5_POWER_MODE_WAKEUP;
  } else if (tmp == 0x02) {
    return VL53L5_POWER_MODE_SLEEP;
  }
  return 0xFF;
}

bool Adafruit_VL53L5CX_NoST::setAddress(uint8_t new_address) {
  if (!_initialized) {
    return false;
  }

  _setPage(0x00);
  _writeByte(0x04, new_address);
  _setPage(0x02);

  delete _i2c;
  _i2c = new Adafruit_I2CDevice(new_address, &Wire);
  return _i2c->begin();
}
