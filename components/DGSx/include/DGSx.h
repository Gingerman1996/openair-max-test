#ifndef DGSX_H
#define DGSX_H

#include "AirgradientSerial.h"
#include <string>

class DGSx {
public:
  struct Data {
    // Gas concentration value (depends on sensor type)
    float gasConcentration;
    // Temperature in Celsius
    float temperature;
    // Humidity in %RH (if available)
    float humidity;
    // Raw ADC values
    uint16_t rawG;
    uint16_t rawT;
    uint16_t rawH;
    // Sensor status
    bool isValid;
    // Timestamp of measurement
    uint32_t timestamp;
    // Sensor type identifier (DGS or DGS2)
    std::string sensorType;
    // Gas type being measured
    std::string gasType;
  };

  DGSx(AirgradientSerial *agSerial);
  ~DGSx() {}

  // Initialize the sensor
  bool begin();
  
  // Check if sensor is connected and responding
  bool isConnected();
  
  // Query EEPROM settings from DGS/DGS2
  bool queryEEPROM();
  
  // Query header string from DGS2 (not recognized by DGS)
  bool queryHeader();
  
  // Request a measurement reading
  void requestMeasurement();
  
  // Clear the serial buffer
  void clearBuffer();
  
  // Non-blocking function to parse response
  bool read(Data &data);
  
  // Blocking function to parse response with timeout
  bool readUntil(Data &data, uint16_t timeoutMs = 2000);
  
  // Enable/disable continuous output mode
  void setContinuousMode(bool enable, uint16_t intervalSeconds = 5);

  bool calibrateZero();
  
  // Get last error message
  const char* getLastError();

private:
  const char *const TAG = "DGSx";
  AirgradientSerial *agSerial_ = nullptr;
  
  // Buffer for incoming data
  static const size_t BUFFER_SIZE = 256;
  char _buffer[BUFFER_SIZE];
  size_t _bufferIndex = 0;
  
  // Timing
  uint32_t _lastRequestTime = 0;
  uint32_t _requestInterval = 5000; // 5 seconds default
  bool _continuousMode = false;
  
  // Error handling
  char _lastError[64];
  
  // Gas type detected from barcode
  std::string _detectedGasType;
  
  // Internal helper functions
  bool _processResponse(Data &data);
  bool _parseDataLine(const char* line, Data &data);
  bool _waitForResponse(uint16_t timeoutMs);
  void _setError(const char* error);
  uint32_t _getMillis();
};

#endif // DGSX_H
