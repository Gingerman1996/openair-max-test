/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include <cstdint>
#include <ratio>
#include <stdio.h>
#include <inttypes.h>
#include <string>

#include <fcntl.h>
#include "airgradientOtaWifi.h"
#include "airgradientWifiClient.h"
#include "esp_attr.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "MaxConfig.h"
#include "Configuration.h"
#include "PayloadCache.h"
#include "StatusLed.h"
#include "Sensor.h"
#include "AirgradientSerial.h"
#include "AirgradientUART.h"
#include "airgradientClient.h"
#include "cellularModule.h"
#include "airgradientCellularClient.h"
#include "cellularModuleA7672xx.h"
#include "airgradientOtaCellular.h"
#include "WiFiManager.h"

#define CONSOLE_MAX_CMDLINE_ARGS 8
#define CONSOLE_MAX_CMDLINE_LENGTH 256
#define CONSOLE_PROMPT_MAX_LEN (32)

// Wake up counter that saved on Low Power memory
RTC_DATA_ATTR unsigned long xWakeUpCounter = 0;

// Hold lead time of the next measurement schedule
// Help in the process of fragmented sleep
RTC_DATA_ATTR int xMeasurementLeadTimeSeconds = 0;

// Hold the index of measures queue on which http post should start send measures
// eg. xHttpCacheQueueIndex = 3, then for [q1, q2, q3, q4, q5, q6] should only send q4, q5, q6
RTC_DATA_ATTR unsigned long xHttpCacheQueueIndex = 0;

// Hold the previous wake up cycle measure interval
// Help to check if there's a measure interval change that make previous payload cache invalid
RTC_DATA_ATTR int xMeasureInterval = 180;

// Global Vars
static const char *const TAG = "APP";
static std::string g_serialNumber;
static bool g_networkReady = false;
static std::string g_fimwareVersion;
static Configuration g_configuration;
static StatusLed g_statusLed(IO_LED_INDICATOR);
static AirgradientSerial *g_ceAgSerial = nullptr;
static CellularModule *g_cellularCard = nullptr;
static AirgradientClient *g_agClient = nullptr;
static WiFiManager g_wifiManager;

static QueueHandle_t bootButtonQueue = NULL;
static TaskHandle_t watchdogTaskHandle = NULL;
static volatile bool watchdogTaskActive = false;

static void IRAM_ATTR bootButtonISRHandler(void *arg) {
  (void)arg;
  int level = gpio_get_level(IO_BOOT_BUTTON);
  xQueueSendFromISR(bootButtonQueue, &level, NULL);
}

/**
 * Reset monitor external watchdog timer
 */
static void resetExtWatchdog() {
  // Toggle the watchdog pin to reset external watchdog
  gpio_set_level(IO_WDT, 0);
  vTaskDelay(pdMS_TO_TICKS(10)); // Hold low for 10ms
  gpio_set_level(IO_WDT, 1);
}

/**
 * Watchdog reset task that runs in background
 */
static void watchdogResetTask(void *pvParameters) {
  const TickType_t watchdog_interval = pdMS_TO_TICKS(5 * 60 * 1000); // 5 minutes
  int reset_count = 0;
  
  ESP_LOGI(TAG, "Watchdog reset task started - will reset every 5 minutes");
  
  while (watchdogTaskActive) {
    vTaskDelay(watchdog_interval);
    if (watchdogTaskActive) {
      resetExtWatchdog();
      reset_count++;
      ESP_LOGI(TAG, "Background watchdog reset #%d (%.1f minutes)", reset_count, reset_count * 5.0f);
    }
  }
  
  ESP_LOGI(TAG, "Watchdog reset task stopping (performed %d resets)", reset_count);
  vTaskDelete(NULL);
}

/**
 * Re-initialize console for logging
 */
static void initConsole();

static void initGPIO();

static void bootButtonTask(void *arg);

static void initBootButton();

/**
 * Start background watchdog reset task
 */
static void startWatchdogTask();

/**
 * Stop background watchdog reset task
 */
static void stopWatchdogTask();

/**
 * Define measure interval based on battery level
 */
static int getMeasureInterval(float batteryVoltage);

/**
 * Calculate the next measurement schedule
 */
static int calculateMeasurementSchedule(uint32_t startTimeMs, int measureInterval);

/**
 * Calculate how long the monitor should sleep
 * Because there's a hardware watchdog if measurement more than 15, it needs wakeup for a moment to reset watchdog
 */
static int setWakeUpTimer(int nextMeasurementScheduleSec);

/**
 * Helper to print system reset reason
 */
static void printResetReason();

/**
 * Helper to print out the reason system wake up from deepsleep
 */
static void printWakeupReason(esp_sleep_wakeup_cause_t reason);

/**
 * Build monitor serial number from WiFi mac address
 */
static std::string buildSerialNumber();

/**
 * Retrieve currently running firmware version
 */
static std::string getFirmwareVersion();

static AirgradientClient::PayloadType getPayloadType();

static void ensureConnectionReady();
static int getNetworkSignalStrength();
static bool initializeNetwork(unsigned long wakeUpCounter);
static bool initializeWiFiNetwork(unsigned long wakeUpCounter);
static bool initializeCellularNetwork(unsigned long wakeUpCounter);
static bool initializeCellularForGnss(unsigned long wakeUpCounter);

static bool checkRemoteConfiguration(unsigned long wakeUpCounter);
static bool checkForFirmwareUpdate(unsigned long wakeUpCounter);
static bool sendMeasuresByCellular(unsigned long wakeUpCounter, PayloadCache &payloadCache,
                                   int measureInterval);
static bool sendMeasuresByWiFi(unsigned long wakeUpCounter,
                               AirgradientClient::MaxSensorPayload sensorPayload);
static bool sendMeasuresUsingMqtt(unsigned long wakeUpCounter, PayloadCache &payloadCache);

// GNSS test commands
static int gnss_init_cmd(int argc, char **argv);
static int gnss_start_cmd(int argc, char **argv);
static int gnss_stop_cmd(int argc, char **argv);
static int gnss_location_cmd(int argc, char **argv);
static int gnss_test_cmd(int argc, char **argv);
static int gnss_hotstart_cmd(int argc, char **argv);
static int gnss_coldstart_cmd(int argc, char **argv);
static int gnss_agps_cmd(int argc, char **argv);
static int gnss_monitor_cmd(int argc, char **argv);
static int gnss_info_cmd(int argc, char **argv);
static int gnss_gpsinfo_cmd(int argc, char **argv);
static int gnss_antenna_cmd(int argc, char **argv);
static int gnss_satellite_cmd(int argc, char **argv);
static int gnss_signal_cmd(int argc, char **argv);
static void register_gnss_commands();
static void runGnssOnlyMode(int wakeUpCounter);

extern "C" void app_main(void) {
  // Re-initialize console for logging only if it just wake up after deepsleep
  esp_sleep_wakeup_cause_t wakeUpReason = esp_sleep_get_wakeup_cause();
  if (wakeUpReason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    initConsole();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Check if its not the time to do measurements
    if (xMeasurementLeadTimeSeconds > 0) {
      int nextMeasurementScheduleSeconds = xMeasurementLeadTimeSeconds;
      ESP_LOGI(TAG, "Next measurements is in %d seconds", nextMeasurementScheduleSeconds);

      // Set the wakeup timer
      int sleepDurationSeconds = setWakeUpTimer(nextMeasurementScheduleSeconds);
      xMeasurementLeadTimeSeconds = nextMeasurementScheduleSeconds - sleepDurationSeconds;
      ESP_LOGI(TAG, "Sleeping for %d seconds", sleepDurationSeconds);

      // Reset external watchdog timer before sleep to make sure its not trigger while in sleep
      //   before system wakeup
      initGPIO();
      resetExtWatchdog();

      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_deep_sleep_start();
      // Will not continue here
      // This process is not considered wake up, hence xWakeUpCounter not incremented
    }

    ++xWakeUpCounter;
  }
  ESP_LOGI(TAG, "Wakeup count: %lu", xWakeUpCounter);
  printResetReason();
  printWakeupReason(wakeUpReason);

  ESP_LOGI(TAG, "MAX!");

  // Indicate wake up time
  uint32_t wakeUpTimeMs = 0;

  // Initialize every peripheral GPIOs to OFF state
  initGPIO();

  // Reset external WDT
  resetExtWatchdog();

  if (xWakeUpCounter == 0) {
    // Initialize boot button event handler only on the first boot
    // So on next boot onwards, boot button will not function
    initBootButton();
    
    // Register console commands for GNSS testing on first boot only
    register_gnss_commands();
  }

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_LOGI(TAG, "MAX!");
  g_statusLed.start();
  if (xWakeUpCounter == 0) {
    // Only turn on led indicator when it just powered on, not wakeup from sleep
    g_statusLed.on();
  }

  g_fimwareVersion = getFirmwareVersion();
  ESP_LOGI(TAG, "Firmware version: %s", g_fimwareVersion.c_str());

  g_serialNumber = buildSerialNumber();
  ESP_LOGI(TAG, "Serial number: %s", g_serialNumber.c_str());

  // Load configuration that saved on NVS
  g_configuration.load();

  /** Run system settings if
   *    system setting expected to run OR
   *    (network option is wifi AND wifi haven't configured)
   */
  if (g_configuration.runSystemSettings() ||
      (g_configuration.getNetworkOption() == NetworkOption::WiFi &&
       g_configuration.isWifiConfigured() == false)) {
    // Run indicator that portal is currently running
    g_statusLed.blinkAsync(0, 100);

    SettingsForm settings;
    if (g_configuration.getNetworkOption() == NetworkOption::Cellular) {
      settings.networkMode = NETWORK_MODE_CELLULAR_STR;
    } else {
      settings.networkMode = NETWORK_MODE_WIFI_STR;
    }
    settings.apn = g_configuration.getAPN();
    settings.httpDomain = g_configuration.getHttpDomain();
    g_wifiManager.setSettings(settings);

    // Run portal
    std::string ssid = std::string("airgradient-") + g_serialNumber;
    g_wifiManager.setConfigPortalBlocking(true);
    bool success = g_wifiManager.startConfigPortal(ssid.c_str(), "cleanair");
    if (!success) {
      if (g_wifiManager.getState() == WM_STATE_PORTAL_ABORT) {
        // If portal is aborted, then disable system settings portal
        ESP_LOGI(TAG, "System settings portal is aborted, disabling it.");
        g_configuration.setRunSystemSettings(false);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      esp_restart();
    }

    // Notify that it success and restart
    g_statusLed.on();

    // Keep the changes and save to persistant configuration
    auto config = g_configuration.get();
    config.runSystemSettings = false;
    settings = g_wifiManager.getSettings();
    config.httpDomain = settings.httpDomain;
    if (settings.networkMode == NETWORK_MODE_CELLULAR_STR) {
      config.networkOption = NetworkOption::Cellular;
      config.apn = settings.apn;
      g_configuration.set(config);

      // Notify that it success and restart
      g_statusLed.on();
      vTaskDelay(pdMS_TO_TICKS(2000));
      esp_restart();
    }

    // No need to restart for wifi mode, just continue because its already successfuly connect
    config.networkOption = NetworkOption::WiFi;
    config.isWifiConfigured = true;
    g_configuration.set(config);

    // Wifi is ready from the start
    g_networkReady = true;
    // Starting time for every cycle should start now because time taken by system settings portal
    wakeUpTimeMs = MILLIS();
  }

  // Reset external WDT
  resetExtWatchdog();

  ESP_LOGI(TAG, "Starting GNSS-only mode (skip sensors, networking, and deep sleep)");

  int wakeUpCounter = xWakeUpCounter;
  runGnssOnlyMode(wakeUpCounter);
  return;
}

static void runGnssOnlyMode(int wakeUpCounter) {
  ESP_LOGI(TAG, "Entering GNSS-only mode loop");

  if (g_configuration.getNetworkOption() != NetworkOption::Cellular) {
    ESP_LOGW(TAG, "Device configured for non-cellular operation, continuing with cellular GNSS mode");
  }

  if (!initializeCellularForGnss(wakeUpCounter)) {
    ESP_LOGE(TAG, "Failed to initialize cellular network; unable to proceed with GNSS mode");
    while (true) {
      resetExtWatchdog();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  auto *a7672xx = static_cast<CellularModuleA7672XX *>(g_cellularCard);
  if (a7672xx == nullptr) {
    ESP_LOGE(TAG, "Cellular module instance missing; aborting GNSS mode");
    while (true) {
      resetExtWatchdog();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  while (true) {
    auto status = a7672xx->gnssInit();
    if (status == CellReturnStatus::Ok) {
      ESP_LOGI(TAG, "GNSS initialization successful");
      break;
    }
    ESP_LOGE(TAG, "GNSS initialization failed (status=%d), retrying...", static_cast<int>(status));
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  // Follow required sequence: power on, set mode/NMEA (in gnssInit), cold start, then continue
  auto coldStatus = a7672xx->gnssColdStart();
  if (coldStatus == CellReturnStatus::Ok) {
    ESP_LOGI(TAG, "GNSS cold start issued");
  } else {
    ESP_LOGW(TAG, "GNSS cold start failed (status=%d)", static_cast<int>(coldStatus));
  }

  ESP_LOGI(TAG, "GNSS module ready; continuously reading GNSS data (no deep sleep)");

  // Reset external watchdog every 5 minutes while sampling GNSS once per second
  constexpr int kWatchdogResetIntervalSec = 5 * 60;
  int secondsSinceWatchdogReset = 0;

  while (true) {
    auto info = a7672xx->gnssGetInfo();
    if (info.status == CellReturnStatus::Ok) {
      ESP_LOGI(TAG, "GNSS Fix: %s", info.data.c_str());
    } else if (info.status == CellReturnStatus::Failed) {
      ESP_LOGW(TAG, "GNSS fix not yet available (last response: %s)",
               info.data.empty() ? "<empty>" : info.data.c_str());
    } else if (info.status == CellReturnStatus::Timeout) {
      ESP_LOGW(TAG, "Timed out waiting for GNSS info");
    } else {
      ESP_LOGE(TAG, "Error reading GNSS info (status=%d, last response: %s)",
               static_cast<int>(info.status),
               info.data.empty() ? "<empty>" : info.data.c_str());
    }

    secondsSinceWatchdogReset++;
    if (secondsSinceWatchdogReset >= kWatchdogResetIntervalSec) {
      resetExtWatchdog();
      secondsSinceWatchdogReset = 0;
    }
    // Sample GNSS once per second
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void initConsole() {
  fflush(stdout);
  fsync(fileno(stdout));
  esp_console_deinit();

  /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  /* Move the caret to the beginning of the next line on '\n' */
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  /* Enable blocking mode on stdin and stdout */
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t jtag_config = {
      .tx_buffer_size = 256,
      .rx_buffer_size = 256,
  };

  /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_config));

  /* Tell vfs to use usb-serial-jtag driver */
  usb_serial_jtag_vfs_use_driver();

  /* Initialize the console */
  esp_console_config_t console_config = {.max_cmdline_length = CONSOLE_MAX_CMDLINE_LENGTH,
                                         .max_cmdline_args = CONSOLE_MAX_CMDLINE_ARGS,
#if CONFIG_LOG_COLORS
                                         .hint_color = atoi(LOG_COLOR_CYAN)
#endif
  };
  ESP_ERROR_CHECK(esp_console_init(&console_config));
}

void startWatchdogTask() {
  if (watchdogTaskHandle != NULL) {
    ESP_LOGW(TAG, "Watchdog task already running");
    return;
  }
  
  watchdogTaskActive = true;
  BaseType_t result = xTaskCreate(
    watchdogResetTask,
    "watchdog_reset",
    2048,
    NULL,
    5,  // Priority
    &watchdogTaskHandle
  );
  
  if (result == pdPASS) {
    ESP_LOGI(TAG, "Watchdog reset task created successfully");
  } else {
    ESP_LOGE(TAG, "Failed to create watchdog reset task");
    watchdogTaskActive = false;
  }
}

void stopWatchdogTask() {
  if (watchdogTaskHandle == NULL) {
    ESP_LOGW(TAG, "Watchdog task not running");
    return;
  }
  
  ESP_LOGI(TAG, "Stopping watchdog reset task...");
  watchdogTaskActive = false;
  
  // Wait for task to finish
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Reset handle
  watchdogTaskHandle = NULL;
  
  ESP_LOGI(TAG, "Watchdog reset task stopped");
}

void initGPIO() {
  // Initialize IOs configurations
  gpio_config_t io_conf;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  if (xWakeUpCounter == 0) {
    io_conf.pin_bit_mask = (1ULL << IO_WDT) | (1ULL << EN_PMS1) | (1ULL << EN_PMS2) |
                           (1ULL << EN_CO2) | (1ULL << EN_CE_CARD) | (1ULL << EN_ALPHASENSE);
  } else {
    // Ignore CO2 and Alphasense load switch IO since the state already retained
    io_conf.pin_bit_mask =
        (1ULL << IO_WDT) | (1ULL << EN_PMS1) | (1ULL << EN_PMS2) | (1ULL << EN_CE_CARD);
  }
  gpio_config(&io_conf);

  // Load switch needs more IO current drive
  gpio_set_drive_capability(IO_WDT, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_PMS1, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_PMS2, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_CO2, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_ALPHASENSE, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_CE_CARD, GPIO_DRIVE_CAP_3);

  // Set default state to off
  gpio_set_level(IO_WDT, 0);
  gpio_set_level(EN_PMS1, 0);
  gpio_set_level(EN_PMS2, 0);
  gpio_set_level(EN_CE_CARD, 0);

  if (xWakeUpCounter == 0) {
    // Directly enable CO2 and AlphaSense load switch and hold the state only when first boot
    // gpio_hold_en will retain the GPIO state on every sleep cycle even when system reset
    gpio_set_level(EN_CO2, 1);
    gpio_set_level(EN_ALPHASENSE, 1);
    gpio_hold_en(EN_CO2);
    gpio_hold_en(EN_ALPHASENSE);
  }
}

void bootButtonTask(void *arg) {
  uint32_t startTimeButtonPressed = 0;
  int level;
  while (1) {
    if (xQueueReceive(bootButtonQueue, &level, portMAX_DELAY)) {
      if (level == 0) {
        // Button pressed
        startTimeButtonPressed = MILLIS();
        continue;
      }
      // Button released

      if ((MILLIS() - startTimeButtonPressed) > 3000 && startTimeButtonPressed != 0) {
        bool currentState = g_configuration.runSystemSettings();
        g_configuration.setRunSystemSettings(!currentState);
        ESP_LOGI(TAG, "Restarting in 2s...");
        g_statusLed.blink(2000, 200);
        esp_restart();
      }
      startTimeButtonPressed = 0;
    }
  }
}

void initBootButton() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1ULL << IO_BOOT_BUTTON);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&io_conf);

  //create a queue to handle gpio event from isr
  bootButtonQueue = xQueueCreate(3, sizeof(uint32_t));
  //start gpio task
  xTaskCreate(bootButtonTask, "boot_button", 2048, NULL, 10, NULL);

  //install gpio isr service
  gpio_install_isr_service(0);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(IO_BOOT_BUTTON, bootButtonISRHandler, nullptr);
}

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
  case ESP_RST_UNKNOWN:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_UNKNOWN");
    break;
  case ESP_RST_POWERON:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_POWERON");
    break;
  case ESP_RST_EXT:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_EXT");
    break;
  case ESP_RST_SW:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_SW");
    break;
  case ESP_RST_PANIC:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_PANIC");
    break;
  case ESP_RST_INT_WDT:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_INT_WDT");
    break;
  case ESP_RST_TASK_WDT:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_TASK_WDT");
    break;
  case ESP_RST_WDT:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_WDT");
    break;
  case ESP_RST_BROWNOUT:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_BROWNOUT");
    break;
  case ESP_RST_SDIO:
    ESP_LOGI(TAG, "Reset reason: ESP_RST_SDIO");
    break;
  default:
    ESP_LOGI(TAG, "Reset reason: unknown");
    break;
  }
}

void printWakeupReason(esp_sleep_wakeup_cause_t reason) {
  switch (reason) {
  case ESP_SLEEP_WAKEUP_EXT0:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    ESP_LOGI(TAG, "Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    ESP_LOGI(TAG, "Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    ESP_LOGI(TAG, "Wakeup caused by ULP program");
    break;
  default:
    ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d", reason);
    ESP_LOGI(TAG, "Wakeup counter: %lu", xWakeUpCounter);
  }
}

std::string buildSerialNumber() {
  uint8_t mac_address[6];
  esp_err_t err = esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get MAC address (%s)", esp_err_to_name(err));
    return {};
  }

  char result[13] = {0};
  snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  std::string sn = std::string(result);

  return sn;
}

std::string getFirmwareVersion() {
  const esp_app_desc_t *app_desc = esp_app_get_description();
  return app_desc->version;
}

int getMeasureInterval(float batteryVoltage) {
  // Define measurement schedule based on battery voltage to keep the monitor running longer
  int measureInterval = g_configuration.getConfigSchedule().pm02;
  if (batteryVoltage >= 0) {
    if (batteryVoltage >= 10.0) {
      // Do nothing, use default from configuration
    } else if (batteryVoltage >= 9.5) {
      measureInterval = 1800; // 30 minutes
    } else {
      measureInterval = 3600; // 60 minutes
    }
  }

  return measureInterval;
}

int calculateMeasurementSchedule(uint32_t startTimeMs, int measureInterval) {
  int aliveTimeSpendMs = MILLIS() - startTimeMs;

  // Calculate time left for the next schedule to make consistent schedule
  int nextScheduleSec = ((measureInterval * 1000) - aliveTimeSpendMs) / 1000;
  if (nextScheduleSec < 0) {
    nextScheduleSec = 0;
  }

  return nextScheduleSec;
}

int setWakeUpTimer(int nextMeasurementScheduleSec) {
  int toSleepSeconds = nextMeasurementScheduleSec;
  if (nextMeasurementScheduleSec > MAX_SLEEP_TIME) {
    // If next measurement schedule is more than 10 minutes, then
    /// sleep for 10 minutes only and wake up to only reset the external watchdog timer. Then sleep again
    ESP_LOGI(TAG, "Next measurement is scheduled in more than 5 minutes. Will do fragmented sleep "
                  "to reset the external watchdog timer before returning to sleep again");
    toSleepSeconds = MAX_SLEEP_TIME;
  }

  esp_sleep_enable_timer_wakeup(toSleepSeconds * 1000000); // Convert to uS
  return toSleepSeconds;
}

AirgradientClient::PayloadType getPayloadType() {
  if (g_configuration.getModel() == Configuration::O_M_1PPSTON_CE) {
    return AirgradientClient::MAX_WITH_O3_NO2;
  } else {
    return AirgradientClient::MAX_WITHOUT_O3_NO2;
  }
}

void ensureConnectionReady() {
  bool reset = false;
  uint32_t lastResetTime = MILLIS();

  while (g_agClient->isClientReady() == false &&
         MILLIS() < TIMEOUT_ENSURING_CONNECTION_ON_BOOT_MS) {
    // Make sure watchdog not reset
    resetExtWatchdog();
    ESP_LOGI(TAG, "Ensuring client connection...");
    if (g_agClient->ensureClientConnection(reset)) {
      // Now its connected, set led notification and stop reconnection
      ESP_LOGI(TAG, "Client connection is ready");
      g_statusLed.blink(600, 100);
      reset = false;
      break;
    }

    // Still not connected
    ESP_LOGE(TAG, "Airgradient client still not connected, retry in 10s");
    g_statusLed.blink(10000, 500);

    if ((MILLIS() - lastResetTime) > RESET_CE_CARD_CYCLE_ON_RECONNECTION_MS) {
      // If last reset CE card is more than 15 mins, then attempt restart
      ESP_LOGI(TAG, "Will reset CE card in the next cycle");
      reset = true;
      lastResetTime = MILLIS();
    }
  }
}

int getNetworkSignalStrength() {
  int signalStrength = 99;
  if (g_configuration.getNetworkOption() == NetworkOption::Cellular) {
    auto result = g_cellularCard->retrieveSignal();
    if (result.status == CellReturnStatus::Ok && result.data != 99) {
      int dbm = g_cellularCard->csqToDbm(result.data);
      if (dbm != 0) {
        signalStrength = dbm;
      }
    }
  } else {
    signalStrength = g_wifiManager.getSignal();
  }

  return signalStrength;
}

bool initializeNetwork(unsigned long wakeUpCounter) {
  if (g_networkReady) {
    ESP_LOGI(TAG, "Network is already ready to use");
    return true;
  }

  if (g_configuration.getNetworkOption() == NetworkOption::Cellular) {
    g_networkReady = initializeCellularNetwork(wakeUpCounter);
  } else {
    g_networkReady = initializeWiFiNetwork(wakeUpCounter);
  }

  // Make sure watchdog not triggered for the rest of the code before sleep
  resetExtWatchdog();

  return g_networkReady;
}

bool initializeCellularNetwork(unsigned long wakeUpCounter) {
  if (g_ceAgSerial != nullptr || g_cellularCard != nullptr || g_agClient != nullptr) {
    ESP_LOGW(
        TAG,
        "Give up cellular initialization on this wakeup cycle since previous attempt is failed");
    return false;
  }

  // Start watchdog reset task before initializing cellular
  startWatchdogTask();

  // Enable CE card power
  gpio_set_level(EN_CE_CARD, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  uint32_t registrationTimeout = TIMEOUT_REGISTER_NETWORK_WAKEUP_CYCLE_MS;
  if (wakeUpCounter == 0) {
    // When currently initializing network, indicate using blink animation
    g_statusLed.blinkAsync(400, 100);
    registrationTimeout = TIMEOUT_REGISTER_NETWORK_ON_FIRST_BOOT_MS;
  }

  g_ceAgSerial = new AirgradientUART();
  if (!g_ceAgSerial->begin(UART_BAUD_PORT_CE_CARD, UART_BAUD_CE_CARD, UART_RX_CE_CARD,
                           UART_TX_CE_CARD)) {
    ESP_LOGI(TAG, "Failed initialize serial communication for cellular card");
    g_statusLed.blink(5000, 400);
    // TODO: maybe restart here?
    return false;
  }

  // Enable debugging when CE card initializing
  g_ceAgSerial->setDebug(true);
  // Initialize cellular card and client
  g_cellularCard = new CellularModuleA7672XX(g_ceAgSerial, IO_CE_POWER);
  g_agClient = new AirgradientCellularClient(g_cellularCard);
  g_agClient->setHttpDomain(g_configuration.getHttpDomain());

  resetExtWatchdog();

  g_agClient->setNetworkRegistrationTimeoutMs(registrationTimeout);
  g_agClient->setAPN(g_configuration.getAPN());
  if (g_agClient->begin(g_serialNumber, getPayloadType())) {
    // Connected
    if (wakeUpCounter == 0) {
      g_statusLed.blink(600, 100);
    }
  } else {
    // Not connected
    ESP_LOGE(TAG, "Failed start airgradient client");
    if (wakeUpCounter == 0) {
      // When its a first boot, ensure connection is ready
      ensureConnectionReady();
    }
  }

  // Disable again
  g_ceAgSerial->setDebug(false);

  // Wait for a moment for CE card is ready for transmission
  vTaskDelay(pdMS_TO_TICKS(2000));

  return true;
}

bool initializeCellularForGnss(unsigned long wakeUpCounter) {
  if (g_cellularCard != nullptr) {
    return true;
  }

  startWatchdogTask();

  gpio_set_level(EN_CE_CARD, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  if (g_ceAgSerial == nullptr) {
    g_ceAgSerial = new AirgradientUART();
    if (!g_ceAgSerial->begin(UART_BAUD_PORT_CE_CARD, UART_BAUD_CE_CARD, UART_RX_CE_CARD,
                             UART_TX_CE_CARD)) {
      ESP_LOGE(TAG, "Failed to open serial port for cellular module (GNSS mode)");
      delete g_ceAgSerial;
      g_ceAgSerial = nullptr;
      return false;
    }
  }

  g_ceAgSerial->setDebug(true);

  auto *module = new CellularModuleA7672XX(g_ceAgSerial, IO_CE_POWER);
  if (!module->init()) {
    ESP_LOGE(TAG, "Failed to initialize cellular module for GNSS mode");
    g_ceAgSerial->setDebug(false);
    delete module;
    return false;
  }

  g_cellularCard = module;
  g_ceAgSerial->setDebug(false);

  // Allow module to settle
  vTaskDelay(pdMS_TO_TICKS(500));

  return true;
}

bool initializeWiFiNetwork(unsigned long wakeUpCounter) {
  if (wakeUpCounter == 0) {
    // When currently initializing network, indicate using blink animation
    g_statusLed.blinkAsync(400, 100);
  }

  if (g_wifiManager.autoConnect(true) == false) {
    ESP_LOGE(TAG, "Failed connect to WiFi");
    return false;
  }

  return true;
}

bool sendMeasuresByCellular(unsigned long wakeUpCounter, PayloadCache &payloadCache,
                            int measureInterval) {
  // Check if pass criteria to post measures
  if (wakeUpCounter != 0 && (wakeUpCounter % TRANSMIT_MEASUREMENTS_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to post measures by cellular network, skip");
    return false;
  }

  if (!initializeNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip send measures");
    return false;
  }

  // Retrieve measurements from the cache
  std::vector<AirgradientClient::MaxSensorPayload> sensorPayload;
  PayloadCacheType tmp;
  ESP_LOGI(TAG, "cache size: %d", payloadCache.getSize());
  /// Define start index of which needs to be included as HTTP post payload
  int idx = 0;
  if (xHttpCacheQueueIndex > 0) {
    idx = xHttpCacheQueueIndex;
    ESP_LOGI(TAG, "HTTP cache queue index start from %d", idx);
  }
  for (int i = idx; i < payloadCache.getSize(); i++) {
    payloadCache.peekAtIndex(i, &tmp);
    sensorPayload.push_back(tmp);
  }

  // Structure payload
  AirgradientClient::AirgradientPayload payload;
  payload.sensor = &sensorPayload;
  payload.measureInterval = measureInterval;
  payload.signal = getNetworkSignalStrength();
  ESP_LOGI(TAG, "Signal strength: %d", payload.signal);

  // Attempt to send
  bool postSuccess = false;
  int attemptCounter = 0;

  do {
    attemptCounter = attemptCounter + 1;
    postSuccess =
        g_agClient->httpPostMeasures(payload, g_configuration.isExtendedPmMeasuresEnabled());
    if (postSuccess) {
      if (wakeUpCounter == 0) {
        // Notify post success only on first boot
        g_statusLed.blinkAsync(800, 100);
      }
      break;
    }

    if (wakeUpCounter != 0) {
      // Check if this is first boot, if not retry in next schedule
      ESP_LOGE(TAG, "Send measures failed, retry in next schedule");
      // Run failed post led indicator
      g_statusLed.blink(3000, 500);
      break;
    }

    if (g_agClient->isClientReady() == false) {
      // Client is not ready,
      g_statusLed.blinkAsync(3000, 500); // Run failed post led indicator
      ESP_LOGE(TAG, "Send measures failed because of client is not ready, ensuring connection...");
      g_ceAgSerial->setDebug(true);
      if (g_agClient->ensureClientConnection(true) == false) {
        ESP_LOGE(TAG, "Failed ensuring client connection, system restart in 5s");
        g_statusLed.blink(5000, 500);
        esp_restart();
      }
      g_ceAgSerial->setDebug(false);

      ESP_LOGI(TAG, "Client is ready now, retry post measures in 5s");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    if (g_agClient->isRegisteredOnAgServer() == false) {
      // TODO: What to do here? maybe just add new led indicator and never restart until user restart it manually
    }

    ESP_LOGW(TAG, "Send measures failed because of server issue, retry in next schedule");
    // Run failed post led indicator because of server issue
    g_statusLed.blink(4000, 500);
    break;

  } while (attemptCounter < 3 && !postSuccess);

  // Check if still first boot, client is not ready and after 3 attempts is still failed post measures
  if (wakeUpCounter == 0 && !postSuccess && g_agClient->isClientReady() == false) {
    ESP_LOGE(TAG, "Give up after 3 attempts of failed post measures because of network reasons, "
                  "restart systems in 6s");
    g_statusLed.blink(6000, 500);
    esp_restart();
  }

  return postSuccess;
}

bool sendMeasuresByWiFi(unsigned long wakeUpCounter,
                        AirgradientClient::MaxSensorPayload sensorPayload) {
  if (!initializeNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip send measures");
    return false;
  }

  // Initialize airgrdaient client
  g_agClient = new AirgradientWifiClient;
  g_agClient->begin(g_serialNumber, getPayloadType());
  g_agClient->setHttpDomain(g_configuration.getHttpDomain());

  AirgradientClient::AirgradientPayload payload;
  payload.sensor = &sensorPayload;
  payload.signal = getNetworkSignalStrength();
  ESP_LOGI(TAG, "Signal strength: %d", payload.signal);

  bool postSuccess = g_agClient->httpPostMeasures(payload, false);
  if (!postSuccess) {
    ESP_LOGE(TAG, "Send measures failed, retry in next schedule");
    // Run failed post led indicator
    g_statusLed.blink(3000, 500);
    return false;
  }

  if (wakeUpCounter == 0) {
    // Notify post success only on first boot
    g_statusLed.blinkAsync(800, 100);
  }

  return true;
}

bool sendMeasuresUsingMqtt(unsigned long wakeUpCounter, PayloadCache &payloadCache) {
  // Sanity check if MQTT is enabled
  std::string uri = g_configuration.getMqttBrokerUrl();
  if (uri.empty()) {
    return true;
  }

  // Sanity check if its time to send measures
  if (wakeUpCounter != 0 && (wakeUpCounter % TRANSMIT_MEASUREMENTS_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to post measures by cellular network, skip");
    return false;
  }

  // Make sure network is indeed connected
  if (!initializeNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip send measures");
    return false;
  }

  if (g_agClient->mqttConnect(uri.c_str()) == false) {
    ESP_LOGE(
        TAG,
        "Skip send measures through MQTT, failed to connect. Will retry publish on next schedule");
    // Please see comment on mqttPublishMeasures below on why set the cache
    xHttpCacheQueueIndex = payloadCache.getSize();
    return false;
  }

  // Retrieve measurements from the cache
  std::vector<AirgradientClient::MaxSensorPayload> sensorPayload;
  PayloadCacheType tmp;
  ESP_LOGI(TAG, "cache size: %d", payloadCache.getSize());
  for (int i = 0; i < payloadCache.getSize(); i++) {
    payloadCache.peekAtIndex(i, &tmp);
    sensorPayload.push_back(tmp);
  }

  // Structure payload
  AirgradientClient::AirgradientPayload payload;
  payload.sensor = &sensorPayload;
  payload.measureInterval = g_configuration.getConfigSchedule().pm02;
  payload.signal = getNetworkSignalStrength();
  ESP_LOGI(TAG, "Signal strength: %d", payload.signal);

  // When success, clean the cache (because already send to both HTTP and MQTT)
  //  and also reset xHttpCacheQueueIndex to make sure post through both HTTP and MQTT start fresh
  // But if failed, since HTTP already send all the measures on cache,
  //  set xHttpCacheQueueIndex to the cache current size as the start index of which HTTP post measures from queue
  bool success = g_agClient->mqttPublishMeasures(payload);
  if (success) {
    payloadCache.clean();
    xHttpCacheQueueIndex = 0;
  } else {
    ESP_LOGI(TAG, "Retry publish on next schedule");
    xHttpCacheQueueIndex = payloadCache.getSize();
  }

  // Ignore disconnect result
  g_agClient->mqttDisconnect();

  return success;
}

bool checkForFirmwareUpdate(unsigned long wakeUpCounter) {
  if (wakeUpCounter != 0 && (wakeUpCounter % FIRMWARE_UPDATE_CHECK_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to check firmware update, skip");
    return true;
  }

  if (!initializeNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip check firmware update");
    return false;
  }

  AirgradientOTA *agOta = nullptr;
  if (g_configuration.getNetworkOption() == NetworkOption::Cellular) {
    agOta = new AirgradientOTACellular(g_cellularCard, g_agClient->getICCID());
  } else {
    agOta = new AirgradientOTAWifi;
  }
  auto result = agOta->updateIfAvailable(g_serialNumber, g_fimwareVersion);

  switch (result) {
  case AirgradientOTA::Failed:
    ESP_LOGI(TAG, "Firmware update failed");
    break;
  case AirgradientOTA::Skipped:
    ESP_LOGI(TAG, "Firmware update is skipped");
    break;
  case AirgradientOTA::AlreadyUpToDate:
    ESP_LOGI(TAG, "Firmware version already up to date");
    break;
  case AirgradientOTA::Success:
    ESP_LOGI(TAG, "Firmware update success, will restart in 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    break;
  default:
    break;
  }

  return true;
}

bool checkRemoteConfiguration(unsigned long wakeUpCounter) {
  if (wakeUpCounter != 0 && (wakeUpCounter % FIRMWARE_UPDATE_CHECK_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to check remote configuration, skip");
    return true;
  }

  if (!initializeNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to network, skip check remote configuration");
    return false;
  }

  // Attempt retrieve configuration
  std::string result = g_agClient->httpFetchConfig();
  if (g_agClient->isRegisteredOnAgServer() == false) {
    ESP_LOGW(TAG, "Monitor hasn't registered on AirGradient dashboard yet");
    return false;
  }

  if (g_agClient->isLastFetchConfigSucceed() == false) {
    return false;
  }

  if (g_configuration.parseRemoteConfig(result) == false) {
    return false;
  }

  if (g_configuration.isConfigChanged()) {
    ESP_LOGI(TAG, "Changed configuration will be applied on next wakeup cycle onwards");
  }

  return true;
}

// GNSS test command implementations
static int gnss_init_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized. Please connect to cellular network first.\n");
    return 1;
  }

  // Cast to A7672XX to access GNSS functions
  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Initializing GNSS...\n");
  auto result = a7672xx->gnssInit();
  if (result == CellReturnStatus::Ok) {
    printf("GNSS initialized successfully\n");
    return 0;
  } else {
    printf("Failed to initialize GNSS (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_start_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized. Please run gnss_init first.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Starting GNSS positioning...\n");
  auto result = a7672xx->gnssStart();
  if (result == CellReturnStatus::Ok) {
    printf("GNSS positioning started successfully\n");
    printf("NMEA data should now be streaming to UART port\n");
    return 0;
  } else {
    printf("Failed to start GNSS positioning (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_stop_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Stopping GNSS...\n");
  auto result = a7672xx->gnssStop();
  if (result == CellReturnStatus::Ok) {
    printf("GNSS stopped successfully\n");
    return 0;
  } else {
    printf("Failed to stop GNSS (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_location_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Reading GNSS location...\n");
  auto result = a7672xx->gnssGetLocation();
  if (result.status == CellReturnStatus::Ok) {
    printf("Location: %s\n", result.data.c_str());
    printf("Format: [mode],[GPS-SVs],[GLONASS-SVs],[BEIDOU-SVs],[lat],[N/S],[lon],[E/W],[date],[UTC-time],[alt],[speed],[course],[PDOP],[HDOP],[VDOP]\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("No GNSS fix available. Please wait for satellites to be acquired.\n");
    return 1;
  } else {
    printf("Failed to read GNSS location (error: %d)\n", (int)result.status);
    return 1;
  }
}

static int gnss_test_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  struct {
    struct arg_lit *enable;
    struct arg_lit *disable;
    struct arg_end *end;
  } args;

  args.enable = arg_lit0("e", "enable", "Enable GNSS test mode");
  args.disable = arg_lit0("d", "disable", "Disable GNSS test mode");
  args.end = arg_end(2);

  void *argtable[] = {args.enable, args.disable, args.end};
  
  if (arg_parse(argc, argv, argtable) != 0) {
    arg_print_errors(stderr, args.end, "gnss_test");
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return 1;
  }

  bool enable = false;
  if (args.enable->count > 0) {
    enable = true;
  } else if (args.disable->count > 0) {
    enable = false;
  } else {
    printf("Usage: gnss_test [-e|--enable] [-d|--disable]\n");
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return 1;
  }

  printf("%s GNSS test mode...\n", enable ? "Enabling" : "Disabling");
  auto result = a7672xx->gnssTestMode(enable);
  if (result == CellReturnStatus::Ok) {
    printf("GNSS test mode %s successfully\n", enable ? "enabled" : "disabled");
    if (enable) {
      printf("Test mode will stream continuous GxGSV sentences to show satellite visibility\n");
    }
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return 0;
  } else {
    printf("Failed to %s GNSS test mode (error: %d)\n", enable ? "enable" : "disable", (int)result);
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return 1;
  }
}

static int gnss_monitor_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Starting GNSS 1-hour monitoring...\n");
  printf("Background watchdog reset task should already be running.\n");
  
  // Start the 1-hour GNSS monitoring
  auto result = a7672xx->gnssReadOneHour();
  
  if (result == CellReturnStatus::Ok) {
    printf("GNSS 1-hour monitoring completed successfully\n");
    return 0;
  } else {
    printf("GNSS 1-hour monitoring failed (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_hotstart_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Performing GNSS hot start...\n");
  auto result = a7672xx->gnssHotStart();
  if (result == CellReturnStatus::Ok) {
    printf("GNSS hot start completed successfully\n");
    printf("Hot start uses latest available data for fastest acquisition\n");
    return 0;
  } else {
    printf("Failed to perform GNSS hot start (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_coldstart_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Performing GNSS cold start...\n");
  auto result = a7672xx->gnssColdStart();
  if (result == CellReturnStatus::Ok) {
    printf("GNSS cold start completed successfully\n");
    printf("Cold start clears all assistance data and starts fresh acquisition\n");
    return 0;
  } else {
    printf("Failed to perform GNSS cold start (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_agps_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Enabling AGPS assistance...\n");
  auto result = a7672xx->gnssEnableAGPS();
  if (result == CellReturnStatus::Ok) {
    printf("AGPS enabled successfully\n");
    printf("AGPS provides assistance data to speed up satellite acquisition\n");
    return 0;
  } else if (result == CellReturnStatus::Failed) {
    printf("AGPS request failed - check network connection\n");
    return 1;
  } else {
    printf("Failed to enable AGPS (error: %d)\n", (int)result);
    return 1;
  }
}

static int gnss_info_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Getting GNSS info via AT+CGNSSINFO...\n");
  auto result = a7672xx->gnssGetInfo();
  if (result.status == CellReturnStatus::Ok) {
    printf("CGNSSINFO result: %s\n", result.data.c_str());
    printf("This command returns complete GNSS navigation info\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("CGNSSINFO request failed\n");
    return 1;
  } else {
    printf("CGNSSINFO request timed out\n");
    return 1;
  }
}

static int gnss_gpsinfo_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Getting GPS info via AT+CGPSINFO...\n");
  auto result = a7672xx->gnssGetGPSInfo();
  if (result.status == CellReturnStatus::Ok) {
    printf("CGPSINFO result: %s\n", result.data.c_str());
    printf("This command returns GPS-specific position info\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("CGPSINFO request failed\n");
    return 1;
  } else {
    printf("CGPSINFO request timed out\n");
    return 1;
  }
}

static int gnss_antenna_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Getting GNSS antenna info via AT+CGNSSANT...\n");
  auto result = a7672xx->gnssGetAntennaInfo();
  if (result.status == CellReturnStatus::Ok) {
    printf("CGNSSANT result: %s\n", result.data.c_str());
    printf("This shows antenna detection status\n");
    printf("0=antenna open, 1=antenna short, 2=antenna normal\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("CGNSSANT request failed\n");
    return 1;
  } else {
    printf("CGNSSANT request timed out\n");
    return 1;
  }
}

static int gnss_satellite_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Getting satellite info via AT+CGNSSSAT...\n");
  auto result = a7672xx->gnssGetSatelliteInfo();
  if (result.status == CellReturnStatus::Ok) {
    printf("CGNSSSAT result: %s\n", result.data.c_str());
    printf("This shows visible satellites and their signal strength\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("CGNSSSAT request failed\n");
    return 1;
  } else {
    printf("CGNSSSAT request timed out\n");
    return 1;
  }
}

static int gnss_signal_cmd(int argc, char **argv) {
  if (g_cellularCard == nullptr) {
    printf("Error: Cellular module not initialized.\n");
    return 1;
  }

  CellularModuleA7672XX* a7672xx = static_cast<CellularModuleA7672XX*>(g_cellularCard);

  printf("Getting GNSS signal strength via AT+CGNSSISR...\n");
  auto result = a7672xx->gnssGetSignalStrength();
  if (result.status == CellReturnStatus::Ok) {
    printf("CGNSSISR result: %s\n", result.data.c_str());
    printf("This shows GNSS signal strength and quality\n");
    return 0;
  } else if (result.status == CellReturnStatus::Failed) {
    printf("CGNSSISR request failed\n");
    return 1;
  } else {
    printf("CGNSSISR request timed out\n");
    return 1;
  }
}

static void register_gnss_commands() {
  const esp_console_cmd_t gnss_init = {
    .command = "gnss_init",
    .help = "Initialize GNSS module",
    .hint = NULL,
    .func = &gnss_init_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_init));

  const esp_console_cmd_t gnss_start = {
    .command = "gnss_start",
    .help = "Start GNSS positioning",
    .hint = NULL,
    .func = &gnss_start_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_start));

  const esp_console_cmd_t gnss_stop = {
    .command = "gnss_stop",
    .help = "Stop GNSS positioning",
    .hint = NULL,
    .func = &gnss_stop_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_stop));

  const esp_console_cmd_t gnss_location = {
    .command = "gnss_location",
    .help = "Get current GNSS location",
    .hint = NULL,
    .func = &gnss_location_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_location));

  const esp_console_cmd_t gnss_test = {
    .command = "gnss_test",
    .help = "Enable/disable GNSS test mode",
    .hint = "[-e|--enable] [-d|--disable]",
    .func = &gnss_test_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_test));

  const esp_console_cmd_t gnss_hotstart = {
    .command = "gnss_hotstart",
    .help = "Perform GNSS hot start",
    .hint = NULL,
    .func = &gnss_hotstart_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_hotstart));

  const esp_console_cmd_t gnss_coldstart = {
    .command = "gnss_coldstart",
    .help = "Perform GNSS cold start",
    .hint = NULL,
    .func = &gnss_coldstart_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_coldstart));

  const esp_console_cmd_t gnss_agps = {
    .command = "gnss_agps",
    .help = "Enable AGPS assistance",
    .hint = NULL,
    .func = &gnss_agps_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_agps));

  const esp_console_cmd_t gnss_info = {
    .command = "gnss_info",
    .help = "Get GNSS info via AT+CGNSSINFO",
    .hint = NULL,
    .func = &gnss_info_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_info));

  const esp_console_cmd_t gnss_gpsinfo = {
    .command = "gnss_gpsinfo",
    .help = "Get GPS info via AT+CGPSINFO",
    .hint = NULL,
    .func = &gnss_gpsinfo_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_gpsinfo));

  const esp_console_cmd_t gnss_antenna = {
    .command = "gnss_antenna",
    .help = "Check GNSS antenna status via AT+CGNSSANT",
    .hint = NULL,
    .func = &gnss_antenna_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_antenna));

  const esp_console_cmd_t gnss_satellite = {
    .command = "gnss_satellite",
    .help = "Get satellite info via AT+CGNSSSAT",
    .hint = NULL,
    .func = &gnss_satellite_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_satellite));

  const esp_console_cmd_t gnss_signal = {
    .command = "gnss_signal",
    .help = "Get GNSS signal strength via AT+CGNSSISR",
    .hint = NULL,
    .func = &gnss_signal_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_signal));

  const esp_console_cmd_t gnss_monitor = {
    .command = "gnss_monitor",
    .help = "Monitor GNSS for 1 hour with watchdog reset every 5 minutes",
    .hint = NULL,
    .func = &gnss_monitor_cmd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&gnss_monitor));

  printf("GNSS test commands registered:\n");
  printf("  gnss_init      - Initialize GNSS module\n");
  printf("  gnss_start     - Start GNSS positioning\n");
  printf("  gnss_stop      - Stop GNSS positioning\n");
  printf("  gnss_location  - Get current location\n");
  printf("  gnss_test      - Enable/disable test mode\n");
  printf("  gnss_hotstart  - Perform hot start\n");
  printf("  gnss_coldstart - Perform cold start\n");
  printf("  gnss_agps      - Enable AGPS assistance\n");
  printf("  gnss_info      - Get GNSS info via AT+CGNSSINFO\n");
  printf("  gnss_gpsinfo   - Get GPS info via AT+CGPSINFO\n");
  printf("  gnss_antenna   - Check GNSS antenna status\n");
  printf("  gnss_satellite - Get satellite information\n");
  printf("  gnss_signal    - Get GNSS signal strength\n");
  printf("  gnss_monitor   - Monitor GNSS for 1 hour with WD reset\n");
}
