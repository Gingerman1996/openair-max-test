#include "DGSx.h"
#include "AirgradientSerial.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Example usage of DGSx sensor component
class DGSxExample {
private:
    static const char* TAG;
    AirgradientSerial* serial;
    DGSx* sensor;

public:
    DGSxExample() {
        serial = new AirgradientSerial();
        sensor = new DGSx(serial);
    }
    
    ~DGSxExample() {
        delete sensor;
        delete serial;
    }
    
    bool initialize() {
        ESP_LOGI(TAG, "Initializing DGSx sensor...");
        
        // Initialize the sensor
        if (!sensor->begin()) {
            ESP_LOGE(TAG, "Failed to initialize DGSx sensor: %s", sensor->getLastError());
            return false;
        }
        
        // Check connection
        if (!sensor->isConnected()) {
            ESP_LOGW(TAG, "Sensor may not be properly connected");
        }
        
        ESP_LOGI(TAG, "DGSx sensor initialized successfully");
        return true;
    }
    
    void runContinuousMode() {
        ESP_LOGI(TAG, "Starting continuous measurement mode...");
        
        // Enable continuous mode with 5-second intervals
        sensor->setContinuousMode(true);
        
        DGSx::Data data;
        
        while (true) {
            if (sensor->read(data)) {
                if (data.isValid) {
                    ESP_LOGI(TAG, "Gas Concentration: %.2f, Temperature: %.1f°C, Humidity: %.1f%%", 
                            data.gasConcentration, data.temperature, data.humidity);
                    
                    if (!data.gasType.empty()) {
                        ESP_LOGI(TAG, "Gas Type: %s", data.gasType.c_str());
                    }
                    
                    if (!data.sensorType.empty()) {
                        ESP_LOGI(TAG, "Sensor Type: %s", data.sensorType.c_str());
                    }
                } else {
                    ESP_LOGD(TAG, "Received invalid or configuration data");
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(100)); // Check for data every 100ms
        }
    }
    
    void runSingleMeasurement() {
        ESP_LOGI(TAG, "Taking single measurement...");
        
        DGSx::Data data;
        
        // Request a measurement
        sensor->requestMeasurement();
        
        // Wait for response with 3-second timeout
        if (sensor->readUntil(data, 3000)) {
            if (data.isValid) {
                ESP_LOGI(TAG, "Measurement successful:");
                ESP_LOGI(TAG, "  Gas Concentration: %.2f", data.gasConcentration);
                ESP_LOGI(TAG, "  Temperature: %.1f°C", data.temperature);
                ESP_LOGI(TAG, "  Humidity: %.1f%%", data.humidity);
                ESP_LOGI(TAG, "  Timestamp: %lu", data.timestamp);
            } else {
                ESP_LOGW(TAG, "Measurement data invalid");
            }
        } else {
            ESP_LOGE(TAG, "Failed to get measurement: %s", sensor->getLastError());
        }
    }
    
    void querySensorInfo() {
        ESP_LOGI(TAG, "Querying sensor information...");
        
        // Query EEPROM settings
        if (sensor->queryEEPROM()) {
            ESP_LOGI(TAG, "EEPROM query sent successfully");
        } else {
            ESP_LOGW(TAG, "EEPROM query failed: %s", sensor->getLastError());
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Query header (DGS2 specific)
        if (sensor->queryHeader()) {
            ESP_LOGI(TAG, "Header query sent successfully");
        } else {
            ESP_LOGW(TAG, "Header query failed: %s", sensor->getLastError());
        }
        
        // Read any responses
        DGSx::Data data;
        uint32_t startTime = pdTICKS_TO_MS(xTaskGetTickCount());
        
        while ((pdTICKS_TO_MS(xTaskGetTickCount()) - startTime) < 2000) {
            if (sensor->read(data)) {
                ESP_LOGI(TAG, "Received sensor info");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
};

const char* DGSxExample::TAG = "DGSxExample";

// Task function for FreeRTOS
extern "C" void dgsx_example_task(void* parameter) {
    DGSxExample example;
    
    if (example.initialize()) {
        // Query sensor information first
        example.querySensorInfo();
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Take a single measurement
        example.runSingleMeasurement();
        
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Run continuous measurement mode
        example.runContinuousMode();
    }
    
    vTaskDelete(NULL);
}

// Function to start the example
extern "C" void start_dgsx_example() {
    xTaskCreate(dgsx_example_task, "dgsx_example", 4096, NULL, 5, NULL);
}
