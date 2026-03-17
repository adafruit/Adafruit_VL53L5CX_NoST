# Adafruit VL53L5CX NoST

Arduino driver for the ST VL53L5CX 8x8 multizone time-of-flight sensor — **without the ST Ultra-Lite Driver (ULD) C code**.

This library uses only the firmware and configuration blobs from ST, with all I2C communication handled through [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO). The DCI (Device Configuration Interface) protocol is implemented directly in C++.

## Features

- **4x4 and 8x8 resolution** ranging
- Configurable ranging frequency (1–60 Hz)
- Configurable integration time (2–1000 ms)
- Sharpener percentage control
- Target order (closest / strongest)
- Ranging mode (continuous / autonomous)
- Power mode control (sleep / wakeup)
- I2C address change

## Why NoST?

The official ST VL53L5CX ULD is a C library with its own I2C abstraction layer (`platform.c`), custom buffer management, and ~1200 lines of init/config code. This library replaces all of that with a clean C++ class using BusIO's `Adafruit_I2CDevice`, making it:

- Easier to read and maintain
- Compatible with BusIO's `maxBufferSize()` for automatic I2C chunking
- Smaller code footprint (single .cpp/.h vs 5+ ST source files)

## Requirements

- 32-bit microcontroller (ESP32, SAMD, nRF52, RP2040, etc.)
- NOT compatible with AVR (firmware blob is ~86KB)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)

## Quick Start

```cpp
#include "Adafruit_VL53L5CX_NoST.h"

Adafruit_VL53L5CX_NoST sensor;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  sensor.begin(0x29, &Wire);
  sensor.setRangingFrequency(15);
  sensor.startRanging();
}

void loop() {
  if (sensor.isDataReady()) {
    int16_t distances[16];
    sensor.getRangingData(distances);
    // distances[] now contains mm values for each zone
  }
}
```

## License

MIT — see [LICENSE](LICENSE)

Firmware blobs (`vl53l5cx_buffers.h`) are copyright STMicroelectronics, provided AS-IS.
