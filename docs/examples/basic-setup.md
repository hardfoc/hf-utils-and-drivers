# Basic Setup Example

<div align="center">

![Basic Setup](https://img.shields.io/badge/Basic-Setup-blue?style=for-the-badge)
![Getting Started](https://img.shields.io/badge/Getting-Started-green?style=for-the-badge)
![Hello World](https://img.shields.io/badge/Hello-World-orange?style=for-the-badge)

**Getting Started with HardFOC Utils and Drivers**

</div>

---

## 🎯 Overview

This example demonstrates the basic setup and initialization of the HardFOC library. It shows how to configure the logger, initialize basic handlers, and perform simple operations.

---

## 📋 Prerequisites

- HardFOC Utils and Drivers library installed
- Platform-specific tools (ESP-IDF, STM32 HAL, Arduino)
- Basic C++ knowledge

---

## 🚀 Basic Setup

### 1. Include Headers

```cpp
#include "driver-handlers/Logger.h"
#include "driver-handlers/As5047uHandler.h"
#include "driver-handlers/Bno08xHandler.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"
```

### 2. Configure Logger

```cpp
void SetupLogger() {
    // Configure logger for development
    LogConfig config;
    config.level = LogLevel::INFO;
    config.enable_colors = true;
    config.enable_effects = true;
    config.max_width = 80;
    config.center_text = true;
    config.add_border = true;
    config.enable_ascii_art = true;
    
    Logger::SetConfig(config);
    
    // Set component-specific log levels
    Logger::SetLogLevel("SYSTEM", LogLevel::INFO);
    Logger::SetLogLevel("SENSOR", LogLevel::DEBUG);
    Logger::SetLogLevel("MOTOR", LogLevel::WARN);
}
```

### 3. Initialize Hardware Interfaces

```cpp
// Global interface objects
EspSpi spi_interface;
EspI2c i2c_interface;

void InitializeHardware() {
    // Initialize SPI for AS5047U encoder
    if (spi_interface.Initialize(SPI2_HOST, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21)) {
        Logger::Info("SYSTEM", "SPI interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize SPI interface");
    }
    
    // Initialize I2C for BNO08X IMU
    if (i2c_interface.Initialize(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_23)) {
        Logger::Info("SYSTEM", "I2C interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize I2C interface");
    }
}
```

### 4. Create and Initialize Handlers

```cpp
// Global handler objects
As5047uHandler encoder;
Bno08xHandler imu;

void InitializeHandlers() {
    // Create encoder handler
    encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
    
    // Create IMU handler
    imu = Bno08xHandler(i2c_interface, "MAIN_IMU");
    
    // Initialize encoder
    if (encoder.Initialize() == As5047uError::SUCCESS) {
        Logger::Info("ENCODER", "AS5047U encoder initialized successfully");
    } else {
        Logger::Error("ENCODER", "Failed to initialize AS5047U encoder");
    }
    
    // Initialize IMU
    if (imu.Initialize() == Bno08xError::SUCCESS) {
        Logger::Info("IMU", "BNO08X IMU initialized successfully");
    } else {
        Logger::Error("IMU", "Failed to initialize BNO08X IMU");
    }
}
```

---

## 💡 Complete Example

### ESP32 Example

```cpp
#include "driver-handlers/Logger.h"
#include "driver-handlers/As5047uHandler.h"
#include "driver-handlers/Bno08xHandler.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Global objects
EspSpi spi_interface;
EspI2c i2c_interface;
As5047uHandler encoder;
Bno08xHandler imu;

void SetupLogger() {
    LogConfig config;
    config.level = LogLevel::INFO;
    config.enable_colors = true;
    config.enable_effects = true;
    config.max_width = 80;
    config.center_text = true;
    config.add_border = true;
    config.enable_ascii_art = true;
    
    Logger::SetConfig(config);
    
    Logger::SetLogLevel("SYSTEM", LogLevel::INFO);
    Logger::SetLogLevel("SENSOR", LogLevel::DEBUG);
    Logger::SetLogLevel("MOTOR", LogLevel::WARN);
}

void InitializeHardware() {
    // Initialize SPI
    if (spi_interface.Initialize(SPI2_HOST, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21)) {
        Logger::Info("SYSTEM", "SPI interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize SPI interface");
    }
    
    // Initialize I2C
    if (i2c_interface.Initialize(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_23)) {
        Logger::Info("SYSTEM", "I2C interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize I2C interface");
    }
}

void InitializeHandlers() {
    // Create handlers
    encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
    imu = Bno08xHandler(i2c_interface, "MAIN_IMU");
    
    // Initialize encoder
    if (encoder.Initialize() == As5047uError::SUCCESS) {
        Logger::Info("ENCODER", "AS5047U encoder initialized successfully");
    } else {
        Logger::Error("ENCODER", "Failed to initialize AS5047U encoder");
    }
    
    // Initialize IMU
    if (imu.Initialize() == Bno08xError::SUCCESS) {
        Logger::Info("IMU", "BNO08X IMU initialized successfully");
    } else {
        Logger::Error("IMU", "Failed to initialize BNO08X IMU");
    }
}

void ReadSensors() {
    // Read encoder
    uint16_t angle;
    if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
        double degrees = As5047uHandler::LSBToDegrees(angle);
        Logger::Info("ENCODER", "Angle: %.2f degrees (%d LSB)", degrees, angle);
    } else {
        Logger::Warn("ENCODER", "Failed to read encoder angle");
    }
    
    // Read IMU
    Bno08xQuaternion quat;
    if (imu.ReadQuaternion(quat) == Bno08xError::SUCCESS) {
        Logger::Info("IMU", "Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                     quat.w, quat.x, quat.y, quat.z);
    } else {
        Logger::Warn("IMU", "Failed to read IMU quaternion");
    }
}

extern "C" void app_main(void) {
    // Setup logger
    SetupLogger();
    
    // Display startup banner
    const char* banner = R"(
╔══════════════════════════════════════════════════════════════╗
║                    HardFOC Basic Setup                       ║
║                     Version 1.0.0                           ║
╚══════════════════════════════════════════════════════════════╝
)";
    
    Logger::LogBanner("SYSTEM", banner);
    
    // Initialize hardware
    InitializeHardware();
    
    // Initialize handlers
    InitializeHandlers();
    
    Logger::Info("SYSTEM", "Basic setup complete. Starting sensor reading loop...");
    
    // Main loop
    while (true) {
        ReadSensors();
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    }
}
```

### STM32 Example

```cpp
#include "driver-handlers/Logger.h"
#include "driver-handlers/As5047uHandler.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/stm32/Stm32Spi.h"
#include "main.h"
#include "stm32f4xx_hal.h"

// Global variables
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

// HardFOC interfaces
Stm32Spi spi_interface;
As5047uHandler encoder;

void SetupLogger() {
    LogConfig config;
    config.level = LogLevel::INFO;
    config.enable_colors = false; // No colors for UART
    config.enable_effects = false;
    config.max_width = 80;
    config.center_text = false;
    config.add_border = false;
    config.enable_ascii_art = true;
    
    Logger::SetConfig(config);
    
    Logger::SetLogLevel("SYSTEM", LogLevel::INFO);
    Logger::SetLogLevel("SENSOR", LogLevel::DEBUG);
}

void InitializeHardware() {
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize peripherals
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    
    // Initialize SPI interface
    if (spi_interface.Initialize(&hspi1)) {
        Logger::Info("SYSTEM", "SPI interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize SPI interface");
    }
}

void InitializeHandlers() {
    // Create encoder handler
    encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
    
    // Initialize encoder
    if (encoder.Initialize() == As5047uError::SUCCESS) {
        Logger::Info("ENCODER", "AS5047U encoder initialized successfully");
    } else {
        Logger::Error("ENCODER", "Failed to initialize AS5047U encoder");
    }
}

void ReadSensors() {
    // Read encoder
    uint16_t angle;
    if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
        double degrees = As5047uHandler::LSBToDegrees(angle);
        Logger::Info("ENCODER", "Angle: %.2f degrees (%d LSB)", degrees, angle);
    } else {
        Logger::Warn("ENCODER", "Failed to read encoder angle");
    }
}

int main(void) {
    // Setup logger
    SetupLogger();
    
    // Display startup banner
    const char* banner = R"(
HardFOC Basic Setup - STM32
Version 1.0.0
)";
    
    Logger::LogBanner("SYSTEM", banner);
    
    // Initialize hardware
    InitializeHardware();
    
    // Initialize handlers
    InitializeHandlers();
    
    Logger::Info("SYSTEM", "Basic setup complete. Starting sensor reading loop...");
    
    // Main loop
    while (1) {
        ReadSensors();
        HAL_Delay(100); // 100ms delay
    }
}

// HAL callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Handle SPI completion
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Handle UART completion
}
```

### Arduino Example

```cpp
#include "driver-handlers/Logger.h"
#include "driver-handlers/As5047uHandler.h"
#include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/arduino/ArduinoSpi.h"

// Hardware configuration
const int CS_PIN = 10;
const int MOSI_PIN = 11;
const int MISO_PIN = 12;
const int SCK_PIN = 13;

// HardFOC interfaces
ArduinoSpi spi_interface;
As5047uHandler encoder;

void SetupLogger() {
    LogConfig config;
    config.level = LogLevel::INFO;
    config.enable_colors = false; // No colors for Serial
    config.enable_effects = false;
    config.max_width = 80;
    config.center_text = false;
    config.add_border = false;
    config.enable_ascii_art = true;
    
    Logger::SetConfig(config);
    
    Logger::SetLogLevel("SYSTEM", LogLevel::INFO);
    Logger::SetLogLevel("SENSOR", LogLevel::DEBUG);
}

void InitializeHardware() {
    // Initialize SPI
    if (spi_interface.Initialize(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN)) {
        Logger::Info("SYSTEM", "SPI interface initialized successfully");
    } else {
        Logger::Error("SYSTEM", "Failed to initialize SPI interface");
    }
}

void InitializeHandlers() {
    // Create encoder handler
    encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
    
    // Initialize encoder
    if (encoder.Initialize() == As5047uError::SUCCESS) {
        Logger::Info("ENCODER", "AS5047U encoder initialized successfully");
    } else {
        Logger::Error("ENCODER", "Failed to initialize AS5047U encoder");
    }
}

void ReadSensors() {
    // Read encoder
    uint16_t angle;
    if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
        double degrees = As5047uHandler::LSBToDegrees(angle);
        Logger::Info("ENCODER", "Angle: %.2f degrees (%d LSB)", degrees, angle);
    } else {
        Logger::Warn("ENCODER", "Failed to read encoder angle");
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    // Setup logger
    SetupLogger();
    
    // Display startup banner
    const char* banner = R"(
HardFOC Basic Setup - Arduino
Version 1.0.0
)";
    
    Logger::LogBanner("SYSTEM", banner);
    
    // Initialize hardware
    InitializeHardware();
    
    // Initialize handlers
    InitializeHandlers();
    
    Logger::Info("SYSTEM", "Basic setup complete. Starting sensor reading loop...");
}

void loop() {
    ReadSensors();
    delay(100); // 100ms delay
}
```

---

## 🔧 Configuration Options

### Logger Configuration

```cpp
// Production configuration
LogConfig production_config;
production_config.level = LogLevel::WARN;
production_config.enable_colors = false;
production_config.enable_effects = false;
production_config.max_width = 120;
production_config.center_text = false;
production_config.add_border = false;
production_config.enable_ascii_art = false;
Logger::SetConfig(production_config);

// Development configuration
LogConfig dev_config;
dev_config.level = LogLevel::DEBUG;
dev_config.enable_colors = true;
dev_config.enable_effects = true;
dev_config.max_width = 80;
dev_config.center_text = true;
dev_config.add_border = true;
dev_config.enable_ascii_art = true;
Logger::SetConfig(dev_config);
```

### Handler Configuration

```cpp
// AS5047U configuration
As5047uConfig encoder_config;
encoder_config.enable_daec = true;
encoder_config.enable_adaptive_filter = true;
encoder_config.zero_position = 8192; // Set zero at 180 degrees
encoder_config.enable_abi_output = true;
encoder_config.abi_resolution_bits = 12;
encoder_config.high_temperature_mode = false;

As5047uHandler encoder(spi_interface, "MOTOR_ENCODER", encoder_config);

// BNO08X configuration
Bno08xConfig imu_config;
imu_config.interface_type = Bno08xInterfaceType::I2C;
imu_config.i2c_address = 0x4A;
imu_config.accel_range = Bno08xAccelRange::RANGE_4G;
imu_config.gyro_range = Bno08xGyroRange::RANGE_500DPS;
imu_config.mag_range = Bno08xMagRange::RANGE_1600UT;
imu_config.enable_wake_on_motion = true;
imu_config.default_report_interval_ms = 10;

Bno08xHandler imu(i2c_interface, "MAIN_IMU", imu_config);
```

---

## 🐛 Troubleshooting

### Common Issues

1. **Initialization Failures:**
   - Check hardware connections
   - Verify pin assignments
   - Ensure proper power supply

2. **Communication Errors:**
   - Check SPI/I2C configuration
   - Verify clock frequencies
   - Check for noise on communication lines

3. **Logging Issues:**
   - Verify serial/UART configuration
   - Check baud rate settings
   - Ensure proper output redirection

### Debug Tips

```cpp
// Enable verbose logging for debugging
Logger::SetLogLevel("SYSTEM", LogLevel::VERBOSE);
Logger::SetLogLevel("SENSOR", LogLevel::VERBOSE);
Logger::SetLogLevel("MOTOR", LogLevel::VERBOSE);

// Check error codes
auto result = encoder.Initialize();
if (result != As5047uError::SUCCESS) {
    Logger::Error("SYSTEM", "Encoder init failed: %s", 
                 As5047uErrorToString(result));
}

// Monitor system resources
size_t free_heap = esp_get_free_heap_size(); // ESP32
Logger::Info("SYSTEM", "Free heap: %d bytes", free_heap);
```

---

## 📚 Next Steps

After completing the basic setup:

1. **Read the [Advanced Examples](advanced-examples.md)** for more complex usage
2. **Explore [Component Documentation](../handlers/)** for detailed API reference
3. **Check [Integration Guide](../integration.md)** for platform-specific details
4. **Review [Architecture Overview](../architecture.md)** for system design understanding 