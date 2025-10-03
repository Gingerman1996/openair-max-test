#include "DGSx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdlib>
#include <ctime>
#include <string.h>
#include <string>

DGSx::DGSx(AirgradientSerial *agSerial) {
  agSerial_ = agSerial;
  _bufferIndex = 0;
  _lastRequestTime = 0;
  _continuousMode = false;
  _detectedGasType = "Unknown"; // Initialize gas type
  memset(_buffer, 0, BUFFER_SIZE);
  memset(_lastError, 0, sizeof(_lastError));
}

bool DGSx::begin() {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not initialized");
    return false;
  }

  if (!agSerial_->begin(9600)) {
    _setError("Failed to initialize serial communication");
    return false;
  }

  // Wait for sensor to stabilize
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Clear any existing data
  clearBuffer();

  ESP_LOGI(TAG, "DGSx sensor initialized");
  return true;
}

bool DGSx::isConnected() {
  // Simple connection test - check if serial interface is available
  if (agSerial_ == nullptr) {
    return false;
  }

  // Check if serial interface is working without sending additional commands
  return agSerial_->available() >= 0;
}

bool DGSx::queryEEPROM() {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return false;
  }

  clearBuffer();

  ESP_LOGI(TAG, "EEPROM command 'e' sent, waiting for response...");

  // Send EEPROM query command
  uint8_t cmd = 'e';
  if (agSerial_->write(&cmd, 1) != 1) {
    _setError("Failed to send EEPROM query command");
    return false;
  }

  // Wait for response and collect all EEPROM lines
  vTaskDelay(pdMS_TO_TICKS(500)); // Wait for sensor to start responding

  uint32_t startTime = _getMillis();
  uint32_t timeout = 2000;
  bool gotResponse = false;
  int lineCount = 0;

  ESP_LOGI(TAG, "EEPROM response detected, reading data...");

  while ((_getMillis() - startTime) < timeout) {
    // Read character by character until we get a complete line
    while (agSerial_->available() > 0 && _bufferIndex < (BUFFER_SIZE - 1)) {
      int c = agSerial_->read();
      if (c >= 0) {
        _buffer[_bufferIndex++] = (char)c;
        _buffer[_bufferIndex] = '\0';

        // Check if we have a complete line
        if (c == '\r' || c == '\n') {
          if (_bufferIndex > 1) {
            // Skip empty lines and lines with just asterisks
            if (strlen(_buffer) > 1 && strstr(_buffer, "***") == nullptr) {
              lineCount++;
              ESP_LOGI(TAG, "line%d: %s", lineCount, _buffer);
              gotResponse = true;

              // Check for gas type in barcode line
              if (strstr(_buffer, "Sensor Barcode") != nullptr) {
                ESP_LOGI(TAG, "📋 Sensor barcode detected in EEPROM");
                if (strstr(_buffer, " NO2 ") != nullptr) {
                  _detectedGasType = "NO2";
                  ESP_LOGI(TAG, "🌬️  Gas type detected from EEPROM: NO2");
                } else if (strstr(_buffer, " CO ") != nullptr) {
                  _detectedGasType = "CO";
                  ESP_LOGI(TAG, "⚠️  Gas type detected from EEPROM: CO");
                } else if (strstr(_buffer, " O3 ") != nullptr) {
                  _detectedGasType = "O3";
                  ESP_LOGI(TAG, "🫧 Gas type detected from EEPROM: O3");
                } else if (strstr(_buffer, " SO2 ") != nullptr) {
                  _detectedGasType = "SO2";
                  ESP_LOGI(TAG, "🌫️  Gas type detected from EEPROM: SO2");
                } else if (strstr(_buffer, " NH3 ") != nullptr) {
                  _detectedGasType = "NH3";
                  ESP_LOGI(TAG, "💨 Gas type detected from EEPROM: NH3");
                }
              }
            }
          }

          // Reset buffer for next line
          _bufferIndex = 0;
          memset(_buffer, 0, BUFFER_SIZE);
        }
      }
    }

    // If no data available, wait a bit
    if (agSerial_->available() == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }

  if (gotResponse) {
    ESP_LOGI(TAG, "EEPROM query completed with %d lines", lineCount);
  } else {
    ESP_LOGW(TAG, "No EEPROM response received");
  }

  return gotResponse;
}

bool DGSx::queryHeader() {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return false;
  }

  clearBuffer();

  // Send header query command (DGS2 only)
  uint8_t cmd = 'h';
  if (agSerial_->write(&cmd, 1) != 1) {
    _setError("Failed to send header query command");
    return false;
  }

  // Wait for response
  return _waitForResponse(1000);
}

void DGSx::requestMeasurement() {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return;
  }

  // Send carriage return to request measurement
  uint8_t cmd = '\r';
  agSerial_->write(&cmd, 1);
  _lastRequestTime = _getMillis();
}

void DGSx::clearBuffer() {
  if (agSerial_ == nullptr) {
    return;
  }

  // Clear internal buffer
  _bufferIndex = 0;
  memset(_buffer, 0, BUFFER_SIZE);

  // Clear serial buffer
  while (agSerial_->available() > 0) {
    agSerial_->read();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool DGSx::read(Data &data) {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return false;
  }

  // Read available data into buffer
  while (agSerial_->available() > 0 && _bufferIndex < (BUFFER_SIZE - 1)) {
    int c = agSerial_->read();
    if (c >= 0) {
      _buffer[_bufferIndex++] = (char)c;
      _buffer[_bufferIndex] = '\0';

      // Check if we have a complete line (ending with \r or \n)
      if (c == '\r' || c == '\n') {
        if (_bufferIndex > 1) { // Ignore empty lines
          bool result = _processResponse(data);
          _bufferIndex = 0; // Reset buffer
          memset(_buffer, 0, BUFFER_SIZE);
          return result;
        }
        _bufferIndex = 0; // Reset buffer for empty lines
      }
    }
  }

  return false; // No complete response yet
}

bool DGSx::readUntil(Data &data, uint16_t timeoutMs) {
  uint32_t startTime = _getMillis();

  while ((_getMillis() - startTime) < timeoutMs) {
    if (read(data)) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy waiting
  }

  _setError("Read timeout");
  return false;
}

void DGSx::setContinuousMode(bool enable) {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return;
  }

  // First, check current sensor state by attempting to read data
  bool sensorInContinuousMode = false;
  ESP_LOGI(TAG, "Checking current sensor state...");

  // Clear buffer first
  clearBuffer();

  // Try to read data for 1 second to see if sensor is already sending
  // continuous data
  Data testData;
  if (readUntil(testData, 1000)) {
    ESP_LOGI(TAG,
             "Sensor is already in continuous mode (received PPB data: %.1f)",
             testData.gasConcentration);
    sensorInContinuousMode = true;
  } else {
    ESP_LOGI(TAG, "No continuous data detected, sensor appears to be in "
                  "single-shot mode");
  }

  if (enable) {
    if (sensorInContinuousMode) {
      ESP_LOGI(
          TAG,
          "Sensor already in continuous mode, no need to send 'C' command");
      _continuousMode = true;
      return;
    }

    ESP_LOGI(TAG, "Enabling continuous mode...");

    // Clear buffer before starting
    clearBuffer();

    // Send continuous mode command
    uint8_t cmd = 'C';
    if (agSerial_->write(&cmd, 1) != 1) {
      _setError("Failed to send continuous mode command");
      return;
    }

    _continuousMode = true;
    ESP_LOGI(TAG, "Continuous mode command 'C' sent to sensor");

  } else {
    if (!sensorInContinuousMode) {
      ESP_LOGI(TAG, "Sensor already in single-shot mode, no need to toggle");
      _continuousMode = false;
      return;
    }

    ESP_LOGI(TAG, "Disabling continuous mode...");

    // Send 'C' command to toggle off continuous mode (since it's a toggle)
    uint8_t toggleCmd = 'C';
    if (agSerial_->write(&toggleCmd, 1) != 1) {
      _setError("Failed to send toggle command to disable continuous mode");
      return;
    }

    _continuousMode = false;
    ESP_LOGI(TAG, "Continuous mode toggled off with 'C' command");
  }
}

bool DGSx::calibrateZero() {
  if (agSerial_ == nullptr) {
    _setError("Serial interface not available");
    return false;
  }

  if (_detectedGasType != "NO2") {
    _setError("NO2 zero calibration can only be performed on NO2 sensors");
    return false;
  }

  const int MAX_CALIBRATION_ATTEMPTS = 10;
  int attemptNumber = 0;
  bool calibrationSuccess = false;

  ESP_LOGI(TAG, "Starting NO2 zero calibration process (max %d attempts)...", 
           MAX_CALIBRATION_ATTEMPTS);

  while (attemptNumber < MAX_CALIBRATION_ATTEMPTS && !calibrationSuccess) {
    attemptNumber++;
    ESP_LOGI(TAG, "========== Calibration Attempt %d/%d ==========", 
             attemptNumber, MAX_CALIBRATION_ATTEMPTS);

    clearBuffer();

    // Step 1: Enable continuous mode for stability monitoring
    ESP_LOGI(TAG, "Enabling continuous mode for stability check...");
    this->setContinuousMode(true); // Enable continuous mode

    // Wait a moment for continuous data to start
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Check stability
    Data stabilityData;
    int stableReadings = 0;
    float lastReading = -999.0;            // Initialize to impossible value
    const float STABILITY_THRESHOLD = 5.0; // PPB tolerance for stability
    const int MIN_STABLE_READINGS =
        3; // Minimum consecutive stable readings required
    uint32_t stabilityStartTime = _getMillis();
    uint32_t stabilityTimeout = 20000; // 20 seconds timeout
    int totalReadings = 0;

    ESP_LOGI(TAG,
             "Checking stability for up to 20 seconds (threshold: ±%.1f PPB)...",
             STABILITY_THRESHOLD);

    while ((_getMillis() - stabilityStartTime) < stabilityTimeout) {
      if (this->readUntil(stabilityData, 3000)) {
        if (stabilityData.isValid) {
          totalReadings++;

          if (lastReading != -999.0) {
            float difference = abs(stabilityData.gasConcentration - lastReading);

            if (difference <= STABILITY_THRESHOLD) {
              stableReadings++;
              ESP_LOGI(TAG, "Stable reading %d: %.1f PPB (diff: %.1f)",
                       stableReadings, stabilityData.gasConcentration,
                       difference);

              if (stableReadings >= MIN_STABLE_READINGS) {
                ESP_LOGI(
                    TAG,
                    "✅ Sensor readings are stable after %d readings in %lu ms",
                    totalReadings, _getMillis() - stabilityStartTime);
                break;
              }
            } else {
              // Reset counter if reading is not stable
              ESP_LOGI(
                  TAG,
                  "Unstable reading %d: %.1f PPB (diff: %.1f) - resetting count",
                  totalReadings, stabilityData.gasConcentration, difference);
              stableReadings = 0;
            }
          } else {
            ESP_LOGI(TAG, "First reading: %.1f PPB",
                     stabilityData.gasConcentration);
          }

          lastReading = stabilityData.gasConcentration;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (stableReadings < MIN_STABLE_READINGS) {
      ESP_LOGW(TAG,
               "Insufficient stable readings for calibration (got %d, need %d)",
               stableReadings, MIN_STABLE_READINGS);
      this->setContinuousMode(false); // Stop continuous mode
      
      if (attemptNumber < MAX_CALIBRATION_ATTEMPTS) {
        ESP_LOGI(TAG, "⏳ Waiting 5 seconds before retry...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      } else {
        _setError("Insufficient stable readings after all attempts");
        return false;
      }
    }

    // Perform actual calibration
    clearBuffer();
    ESP_LOGI(TAG, "Sending NO2 zero calibration command 'Z'...");

    uint8_t calCmd = 'Z';
    if (agSerial_->write(&calCmd, 1) != 1) {
      _setError("Failed to send NO2 zero calibration command");
      this->setContinuousMode(false);
      
      if (attemptNumber < MAX_CALIBRATION_ATTEMPTS) {
        ESP_LOGI(TAG, "⏳ Waiting 5 seconds before retry...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      } else {
        return false;
      }
    }

    // Wait for calibration acknowledgment and verify readings
    Data calData;
    ESP_LOGI(TAG, "Monitoring calibration results for 20 seconds...");

    uint32_t calStartTime = _getMillis();
    uint32_t calTimeout = 20000; // 20 seconds timeout
    int validPostCalReadings = 0;
    int negativeReadings = 0;

    while ((_getMillis() - calStartTime) < calTimeout) {
      if (readUntil(calData, 3000)) { // 3 second timeout per reading
        if (calData.isValid) {
          validPostCalReadings++;

          if (calData.gasConcentration < 0) {
            negativeReadings++;
            ESP_LOGW(TAG,
                     "Post-cal reading %d: %.1f PPB (NEGATIVE - calibration may "
                     "have failed)",
                     validPostCalReadings, calData.gasConcentration);
          } else {
            ESP_LOGI(TAG, "Post-cal reading %d: %.1f PPB (positive - good)",
                     validPostCalReadings, calData.gasConcentration);
          }

          // Check if we have enough readings to evaluate calibration
          if (validPostCalReadings >= 5) {
            float negativePercentage =
                (float)negativeReadings / validPostCalReadings * 100.0;
            ESP_LOGI(TAG,
                     "Calibration assessment: %d/%d readings negative (%.1f%%)",
                     negativeReadings, validPostCalReadings, negativePercentage);

            // Consider calibration successful if less than 20% of readings are
            // negative
            if (negativePercentage < 20.0) {
              calibrationSuccess = true;
              ESP_LOGI(TAG, "✅ Zero calibration appears successful - low "
                            "negative readings");
              break;
            }
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Final assessment for this attempt
    if (validPostCalReadings == 0) {
      ESP_LOGE(TAG, "❌ Attempt %d: No valid readings received after calibration command", 
               attemptNumber);
      
      if (attemptNumber < MAX_CALIBRATION_ATTEMPTS) {
        ESP_LOGI(TAG, "⏳ Waiting 5 seconds before retry...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      } else {
        _setError("No valid readings received after calibration command");
        return false;
      }
    }

    float finalNegativePercentage =
        (float)negativeReadings / validPostCalReadings * 100.0;
    ESP_LOGI(TAG, "Attempt %d results: %d/%d readings negative (%.1f%%)",
             attemptNumber, negativeReadings, validPostCalReadings, 
             finalNegativePercentage);

    if (finalNegativePercentage >= 50.0) {
      ESP_LOGE(TAG, "❌ Attempt %d: Calibration failed - too many negative readings", 
               attemptNumber);
      
      if (attemptNumber < MAX_CALIBRATION_ATTEMPTS) {
        ESP_LOGI(TAG, "⏳ Waiting 5 seconds before retry...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      } else {
        _setError("High percentage of negative readings after all calibration attempts");
        return false;
      }
    } else if (finalNegativePercentage >= 20.0) {
      ESP_LOGW(TAG, "⚠️  Attempt %d: Calibration partially successful - some negative readings remain", 
               attemptNumber);
      
      if (attemptNumber < MAX_CALIBRATION_ATTEMPTS) {
        ESP_LOGI(TAG, "⏳ Waiting 5 seconds before retry to improve results...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      } else {
        ESP_LOGW(TAG, "⚠️  Accepting partial calibration after %d attempts", 
                 MAX_CALIBRATION_ATTEMPTS);
        calibrationSuccess = true;
      }
    } else {
      // Success!
      calibrationSuccess = true;
    }
  }

  if (calibrationSuccess) {
    ESP_LOGI(TAG, "🎉 NO2 zero calibration completed successfully after %d attempt(s)!", 
             attemptNumber);
    return true;
  } else {
    ESP_LOGE(TAG, "💥 NO2 zero calibration failed after %d attempts", 
             MAX_CALIBRATION_ATTEMPTS);
    _setError("Calibration failed after maximum retry attempts");
    return false;
  }
}

const char *DGSx::getLastError() { return _lastError; }

bool DGSx::isContinuousMode() { return _continuousMode; }

bool DGSx::_processResponse(Data &data) {
  // Initialize data structure
  data.gasConcentration = 0.0;
  data.temperature = 0.0;
  data.humidity = 0.0;
  data.rawG = 0;
  data.rawT = 0;
  data.rawH = 0;
  data.isValid = false;
  data.timestamp = _getMillis();
  data.sensorType = "Unknown";
  data.gasType = "Unknown";

  // Skip empty responses
  if (_bufferIndex <= 1) {
    return false;
  }

  // Log the raw response for debugging
  ESP_LOGD(TAG, "Raw response: '%s'", _buffer);

  // Try to parse the response
  return _parseDataLine(_buffer, data);
}

bool DGSx::_parseDataLine(const char *line, Data &data) {
  if (line == nullptr || strlen(line) == 0) {
    return false;
  }

  // Log the raw response for debugging
  ESP_LOGI(TAG, "Raw sensor response: '%s'", line);

  // Check for firmware information
  if (strstr(line, "FW Date") != nullptr) {
    ESP_LOGI(TAG, "Firmware info: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for sensor barcode and gas type
  if (strstr(line, "Sensor Barcode") != nullptr) {
    ESP_LOGI(TAG, "📋 Sensor barcode: %s", line);
    // Extract gas type (e.g., "NO2", "CO", "O3")
    if (strstr(line, " NO2 ") != nullptr) {
      data.gasType = "NO2";
      _detectedGasType = "NO2"; // Store for future use
      ESP_LOGI(TAG, "🌬️  Gas type detected: NO2");
    } else if (strstr(line, " CO ") != nullptr) {
      data.gasType = "CO";
      _detectedGasType = "CO"; // Store for future use
      ESP_LOGI(TAG, "⚠️  Gas type detected: CO");
    } else if (strstr(line, " O3 ") != nullptr) {
      data.gasType = "O3";
      _detectedGasType = "O3"; // Store for future use
      ESP_LOGI(TAG, "🫧 Gas type detected: O3");
    } else if (strstr(line, " SO2 ") != nullptr) {
      data.gasType = "SO2";
      _detectedGasType = "SO2"; // Store for future use
      ESP_LOGI(TAG, "🌫️  Gas type detected: SO2");
    } else if (strstr(line, " NH3 ") != nullptr) {
      data.gasType = "NH3";
      _detectedGasType = "NH3"; // Store for future use
      ESP_LOGI(TAG, "💨 Gas type detected: NH3");
    } else {
      ESP_LOGW(TAG, "⚠️  Unknown gas type in barcode, please check format");
    }
    data.sensorType = "DGS";
    return true;
  }

  // Check for sensitivity and calibration data
  if (strstr(line, "pA/PPM") != nullptr) {
    ESP_LOGI(TAG, "⚙️  Calibration data: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for temperature/humidity compensation
  if (strstr(line, "Nx10") != nullptr && strstr(line, "Toff") != nullptr) {
    ESP_LOGI(TAG, "🌡️  Compensation settings: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for potentiostat settings
  if (strstr(line, "LPDACDAT") != nullptr &&
      strstr(line, "LPTIACON") != nullptr) {
    ESP_LOGI(TAG, "🔧 Potentiostat settings: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for Open Circuit, Zero, Span calibration data
  if (strstr(line, "OC") != nullptr && strstr(line, "ADC G") != nullptr) {
    ESP_LOGI(TAG, "📊 Open Circuit calibration: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  if (strstr(line, "Zero") != nullptr && strstr(line, "ADC G") != nullptr) {
    ESP_LOGI(TAG, "0️⃣  Zero calibration: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  if (strstr(line, "Span") != nullptr && strstr(line, "ADC G") != nullptr) {
    ESP_LOGI(TAG, "📏 Span calibration: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for module barcode
  if (strstr(line, "Module Barcode") != nullptr) {
    ESP_LOGI(TAG, "🏷️  Module info: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for EEPROM unlock message
  if (strstr(line, "EEPROM Unlocked") != nullptr) {
    ESP_LOGI(TAG, "EEPROM unlocked");
    data.sensorType = "DGS";
    return true;
  }

  // Check for header format info
  if (strstr(line, "Serial, PPB, Tx100") != nullptr ||
      strstr(line, "PPB") != nullptr) {
    ESP_LOGI(TAG, "Data format header: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Check for configuration data
  if (strstr(line, "Nx10") != nullptr || strstr(line, "LPDACDAT") != nullptr) {
    ESP_LOGI(TAG, "Configuration data: %s", line);
    data.sensorType = "DGS";
    return true;
  }

  // Try to parse measurement data (comma-separated values)
  // Format: Serial, PPB, Temp, Humi, ADC_G, ADC_T, ADC_H
  char *comma1 = strchr(line, ',');

  // Declare ADC variables at function scope
  uint16_t adc_g_raw = 0, adc_t_raw = 0, adc_h_raw = 0;

  if (comma1 != nullptr) {
    char *comma2 = strchr(comma1 + 1, ',');
    if (comma2 != nullptr) {
      // Extract PPB value (gas concentration)
      char ppb_str[16];
      size_t ppb_len = comma2 - comma1 - 1;
      if (ppb_len < sizeof(ppb_str)) {
        strncpy(ppb_str, comma1 + 1, ppb_len);
        ppb_str[ppb_len] = '\0';

        // Remove leading/trailing spaces
        char *ppb_start = ppb_str;
        while (*ppb_start == ' ')
          ppb_start++;

        float ppb_value = strtof(ppb_start, nullptr);
        if (ppb_value != 0.0 || *ppb_start == '0') {
          data.gasConcentration = ppb_value;
          data.isValid = true;
          data.sensorType = "DGS";

          // Use detected gas type from barcode, or default to NO2 if not
          // detected
          if (!_detectedGasType.empty() && _detectedGasType != "Unknown") {
            data.gasType = _detectedGasType;
          } else {
            data.gasType = "NO2"; // Default for current sensor
          }

          // Try to extract temperature, humidity, and raw ADC values
          // Format: Serial, PPB, Temp, Humi, ADC_G, ADC_T, ADC_H
          char *comma3 = strchr(comma2 + 1, ',');
          char *comma4 = comma3 ? strchr(comma3 + 1, ',') : nullptr;
          char *comma5 =
              comma4 ? strchr(comma4 + 1, ',') : nullptr; // ADC_G position

          if (comma3 && comma4 && comma5) {
            // Extract Temperature (field 3: divide by 100 to get actual
            // temperature)
            char temp_str[16];
            size_t temp_len = comma3 - comma2 - 1;
            if (temp_len < sizeof(temp_str)) {
              strncpy(temp_str, comma2 + 1, temp_len);
              temp_str[temp_len] = '\0';

              char *temp_start = temp_str;
              while (*temp_start == ' ')
                temp_start++;

              data.temperature = strtof(temp_start, nullptr) / 100.0;
            }

            // Extract Humidity (field 4: divide by 100 to get actual humidity)
            char hum_str[16];
            size_t hum_len = comma4 - comma3 - 1;
            if (hum_len < sizeof(hum_str)) {
              strncpy(hum_str, comma3 + 1, hum_len);
              hum_str[hum_len] = '\0';

              char *hum_start = hum_str;
              while (*hum_start == ' ')
                hum_start++;

              data.humidity = strtof(hum_start, nullptr) / 100.0;
            }

            // Extract ADC_G (raw gas ADC value) - field 5
            char *comma6 = strchr(comma5 + 1, ',');
            if (comma6) {
              char adc_g_str[16];
              size_t adc_g_len = comma5 - comma4 - 1;
              if (adc_g_len < sizeof(adc_g_str)) {
                strncpy(adc_g_str, comma4 + 1, adc_g_len);
                adc_g_str[adc_g_len] = '\0';

                char *adc_g_start = adc_g_str;
                while (*adc_g_start == ' ')
                  adc_g_start++;

                adc_g_raw = (uint16_t)strtoul(adc_g_start, nullptr, 10);
                data.rawG = adc_g_raw;

                // Extract ADC_T (raw temperature ADC value) - field 6
                char *comma7 = strchr(comma6 + 1, ',');
                if (comma7) {
                  // If comma7 exists, there are more fields after ADC_T
                  char adc_t_str[16];
                  size_t adc_t_len = comma7 - comma6 - 1;
                  if (adc_t_len < sizeof(adc_t_str)) {
                    strncpy(adc_t_str, comma6 + 1, adc_t_len);
                    adc_t_str[adc_t_len] = '\0';

                    char *adc_t_start = adc_t_str;
                    while (*adc_t_start == ' ')
                      adc_t_start++;

                    adc_t_raw = (uint16_t)strtoul(adc_t_start, nullptr, 10);
                    data.rawT = adc_t_raw;

                    // Extract ADC_H (raw humidity ADC value) - field 7 (last
                    // field)
                    char adc_h_str[16];
                    const char *adc_h_ptr = comma7 + 1;
                    size_t adc_h_len = strlen(adc_h_ptr);
                    // Remove trailing whitespace and newlines
                    while (adc_h_len > 0 && (adc_h_ptr[adc_h_len - 1] == '\r' ||
                                             adc_h_ptr[adc_h_len - 1] == '\n' ||
                                             adc_h_ptr[adc_h_len - 1] == ' ')) {
                      adc_h_len--;
                    }
                    if (adc_h_len < sizeof(adc_h_str)) {
                      strncpy(adc_h_str, adc_h_ptr, adc_h_len);
                      adc_h_str[adc_h_len] = '\0';

                      char *adc_h_start = adc_h_str;
                      while (*adc_h_start == ' ')
                        adc_h_start++;

                      adc_h_raw = (uint16_t)strtoul(adc_h_start, nullptr, 10);
                      data.rawH = adc_h_raw;
                    }
                  }
                } else {
                  // No comma7, so we have only 7 fields: Serial, PPB, Temp,
                  // Humi, ADC_G, ADC_T, ADC_H Extract ADC_T from comma5+1 to
                  // comma6
                  char adc_t_str[16];
                  size_t adc_t_len = comma6 - comma5 - 1;
                  if (adc_t_len < sizeof(adc_t_str)) {
                    strncpy(adc_t_str, comma5 + 1, adc_t_len);
                    adc_t_str[adc_t_len] = '\0';

                    char *adc_t_start = adc_t_str;
                    while (*adc_t_start == ' ')
                      adc_t_start++;

                    adc_t_raw = (uint16_t)strtoul(adc_t_start, nullptr, 10);
                    data.rawT = adc_t_raw;
                  }

                  // Extract ADC_H from comma6+1 to end of line
                  char adc_h_str[16];
                  const char *adc_h_ptr = comma6 + 1;
                  size_t adc_h_len = strlen(adc_h_ptr);
                  // Remove trailing whitespace and newlines
                  while (adc_h_len > 0 && (adc_h_ptr[adc_h_len - 1] == '\r' ||
                                           adc_h_ptr[adc_h_len - 1] == '\n' ||
                                           adc_h_ptr[adc_h_len - 1] == ' ')) {
                    adc_h_len--;
                  }
                  if (adc_h_len < sizeof(adc_h_str)) {
                    strncpy(adc_h_str, adc_h_ptr, adc_h_len);
                    adc_h_str[adc_h_len] = '\0';

                    char *adc_h_start = adc_h_str;
                    while (*adc_h_start == ' ')
                      adc_h_start++;

                    adc_h_raw = (uint16_t)strtoul(adc_h_start, nullptr, 10);
                    data.rawH = adc_h_raw;
                  }
                }
              }
            } else {
              // Fallback: if no comma6, no ADC values available
              data.rawG = 0;
              data.rawT = 0;
              data.rawH = 0;
            }

            // Display data in the requested format
            ESP_LOGI(TAG, "Gas concentration: %.1f PPB (%s)", ppb_value,
                     data.gasType.c_str());
            ESP_LOGI(TAG, "Temp: %.1f°C", data.temperature);
            ESP_LOGI(TAG, "Humidity: %.1f%%", data.humidity);
            ESP_LOGI(TAG, "ADC_G: %d", adc_g_raw);
            ESP_LOGI(TAG, "ADC_T: %d", adc_t_raw);
            ESP_LOGI(TAG, "ADC_H: %d", adc_h_raw);
          } else {
            // Fallback: if parsing fails, no ADC values available
            data.rawG = 0;
            data.rawT = 0;
            data.rawH = 0;
          }

          return true;
        }
      }
    }
  }

  // If we can't parse the line specifically, log it for debugging
  ESP_LOGD(TAG, "Unparsed response: %s", line);
  return false;
}

bool DGSx::_waitForResponse(uint16_t timeoutMs) {
  uint32_t startTime = _getMillis();

  while ((_getMillis() - startTime) < timeoutMs) {
    if (agSerial_->available() > 0) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return false;
}

void DGSx::_setError(const char *error) {
  strncpy(_lastError, error, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = '\0';
  ESP_LOGE(TAG, "%s", error);
}

uint32_t DGSx::_getMillis() { return pdTICKS_TO_MS(xTaskGetTickCount()); }
