# DGSx Gas Sensor Component

This component provides an interface for SPEC Sensors DGS and DGS2 gas sensor modules using the AirgradientSerial interface.

## Features

- Support for both DGS and DGS2 sensor types
- Serial communication at 9600 baud
- Continuous and single measurement modes
- EEPROM and header query capabilities
- Temperature and humidity reading (when available)
- Error handling and debugging support

## Hardware Connections

The DGS/DGS2 sensors are 3.3V modules and are not compatible with 5V power supply or 5V TX/RX connections.

**DGS & DGS2 Pin Connections:**
- Pin 1: N/C
- Pin 2: Serial Receive (RXD) - 3.3V
- Pin 3: Serial Transmit (TXD) - 3.3V  
- Pin 4: N/C
- Pin 5: N/C
- Pin 6: GND
- Pin 7: N/C
- Pin 8: 3.3V

## Usage

### Basic Initialization

```cpp
#include "DGSx.h"
#include "AirgradientSerial.h"

// Create serial interface and sensor
AirgradientSerial* serial = new AirgradientSerial();
DGSx* sensor = new DGSx(serial);

// Initialize the sensor
if (sensor->begin()) {
    ESP_LOGI("APP", "DGSx sensor initialized successfully");
} else {
    ESP_LOGE("APP", "Failed to initialize: %s", sensor->getLastError());
}
```

### Single Measurement

```cpp
DGSx::Data data;

// Request a measurement
sensor->requestMeasurement();

// Wait for response with timeout
if (sensor->readUntil(data, 3000)) {
    if (data.isValid) {
        ESP_LOGI("APP", "Gas Concentration: %.2f", data.gasConcentration);
        ESP_LOGI("APP", "Temperature: %.1f°C", data.temperature);
        ESP_LOGI("APP", "Humidity: %.1f%%", data.humidity);
    }
}
```

### Continuous Measurement Mode

```cpp
// Enable continuous mode with 5-second intervals
sensor->setContinuousMode(true, 5);

DGSx::Data data;
while (true) {
    if (sensor->read(data) && data.isValid) {
        ESP_LOGI("APP", "Gas: %.2f, Temp: %.1f°C", 
                data.gasConcentration, data.temperature);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

### Query Sensor Information

```cpp
// Query EEPROM settings
if (sensor->queryEEPROM()) {
    ESP_LOGI("APP", "EEPROM query sent");
}

// Query header information (DGS2 only)
if (sensor->queryHeader()) {
    ESP_LOGI("APP", "Header query sent");
}

// Read responses
DGSx::Data data;
if (sensor->readUntil(data, 2000)) {
    // Process sensor information
}
```

## Data Structure

The `DGSx::Data` structure contains:

- `gasConcentration`: Gas concentration value (units depend on sensor type)
- `temperature`: Temperature in Celsius
- `humidity`: Humidity in %RH (if available)
- `rawValue`: Raw sensor data as uint16_t
- `isValid`: Boolean indicating if the data is valid
- `timestamp`: Timestamp of measurement in milliseconds
- `sensorType`: String identifying sensor type (DGS or DGS2)
- `gasType`: String identifying the gas being measured

## Error Handling

The component provides error information through:

```cpp
const char* error = sensor->getLastError();
ESP_LOGE("APP", "Sensor error: %s", error);
```

## Commands Supported

Based on the original Arduino implementation, the sensor supports:

- `'e'`: Query EEPROM settings
- `'h'`: Query header string (DGS2 only)
- `'\r'`: Request measurement reading
- Various configuration commands (refer to sensor documentation)

## Notes

- The sensor requires time to stabilize after power-up
- Some commands may interfere with continuous output mode
- DGS2 sensors support additional commands not available on DGS sensors
- Serial communication is at 9600 baud, 8N1 format
- Response parsing handles various data formats returned by different sensor types

## Dependencies

- AirgradientSerial component
- ESP-IDF framework
- FreeRTOS (for task delays)

## Example

See `DGSx_example.cpp` for a complete usage example including initialization, single measurements, and continuous monitoring.
