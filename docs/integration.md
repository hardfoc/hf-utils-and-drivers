# Integration Guide

<div align="center">

![Integration](https://img.shields.io/badge/Integration-Guide-blue?style=for-the-badge)
![Platforms](https://img.shields.io/badge/Platforms-ESP32%20%7C%20STM32%20%7C%20Arduino-green?style=for-the-badge)
![CMake](https://img.shields.io/badge/Build-CMake-orange?style=for-the-badge)

**Complete Integration Guide for HardFOC Utils and Drivers**

</div>

---

## 🎯 Overview

This guide provides comprehensive instructions for integrating the HardFOC Utils and Drivers library into your embedded projects. The library is designed to work seamlessly across multiple platforms including ESP32, STM32, and Arduino.

---

## 📋 Prerequisites

### 🔧 Required Tools

- **C++17** compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- **CMake** 3.16 or higher
- **Git** for cloning and submodule management
- **Platform-specific tools** (ESP-IDF, STM32CubeIDE, Arduino IDE)

### 📦 Required Dependencies

- **HardFOC Core Drivers** (included as submodule)
- **HardFOC Core Utils** (included as submodule)
- **Platform-specific HAL** (ESP-IDF, STM32 HAL, Arduino)

---

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/hardfoc/hf-utils-and-drivers.git
cd hf-utils-and-drivers
```

### 2. Initialize Submodules

```bash
git submodule update --init --recursive
```

### 3. Include in Your Project

```cmake
# Add the library to your CMake project
add_subdirectory(hf-utils-and-drivers)

# Link against the library
target_link_libraries(your_target hf-utils-and-drivers)
```

### 4. Basic Usage Example

```cpp
#include "driver-handlers/As5047uHandler.h"
#include "driver-handlers/Logger.h"

// Get platform-specific SPI interface
BaseSpi& spi = GetPlatformSpi();

// Create and initialize handler
As5047uHandler encoder(spi, "MOTOR_ENCODER");
if (encoder.Initialize() == As5047uError::SUCCESS) {
    Logger::Info("SYSTEM", "Encoder initialized successfully");
}
```

---

## 🔧 Platform-Specific Integration

### 🟦 ESP32 Integration

#### Prerequisites

- **ESP-IDF** v4.4 or higher
- **CMake** 3.16+
- **Python** 3.7+

#### Setup Instructions

1. **Install ESP-IDF:**
   ```bash
   # Clone ESP-IDF
   git clone --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   
   # Install ESP-IDF
   ./install.sh
   
   # Set up environment
   source export.sh
   ```

2. **Create ESP32 Project:**
   ```bash
   # Create new project
   idf.py create-project my_hardfoc_project
   cd my_hardfoc_project
   ```

3. **Add HardFOC Library:**
   ```bash
   # Clone HardFOC library
   git clone https://github.com/hardfoc/hf-utils-and-drivers.git
   cd hf-utils-and-drivers
   git submodule update --init --recursive
   cd ..
   ```

4. **Configure CMakeLists.txt:**
   ```cmake
   cmake_minimum_required(VERSION 3.16)
   
   # Include ESP-IDF
   include($ENV{IDF_PATH}/tools/cmake/project.cmake)
   
   # Add HardFOC library
   add_subdirectory(hf-utils-and-drivers)
   
   # Create project
   project(my_hardfoc_project)
   
   # Add source files
   file(GLOB_RECURSE SOURCES "main/*.cpp" "main/*.c")
   
   # Create executable
   idf_component_register(SRCS ${SOURCES})
   
   # Link HardFOC library
   target_link_libraries(${COMPONENT_LIB} hf-utils-and-drivers)
   ```

5. **Example ESP32 Application:**
   ```cpp
   #include "driver-handlers/As5047uHandler.h"
   #include "driver-handlers/Bno08xHandler.h"
   #include "driver-handlers/Logger.h"
   #include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
   #include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"
   
   extern "C" void app_main(void) {
       // Configure logger
       LogConfig config;
       config.level = LogLevel::INFO;
       config.enable_colors = true;
       Logger::SetConfig(config);
       
       Logger::Info("SYSTEM", "HardFOC ESP32 Application Starting...");
       
       // Initialize SPI for AS5047U
       EspSpi spi;
       spi.Initialize(SPI2_HOST, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21);
       
       // Initialize I2C for BNO08X
       EspI2c i2c;
       i2c.Initialize(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_23);
       
       // Create handlers
       As5047uHandler encoder(spi, "MOTOR_ENCODER");
       Bno08xHandler imu(i2c, "MAIN_IMU");
       
       // Initialize sensors
       if (encoder.Initialize() == As5047uError::SUCCESS) {
           Logger::Info("ENCODER", "AS5047U initialized successfully");
       }
       
       if (imu.Initialize() == Bno08xError::SUCCESS) {
           Logger::Info("IMU", "BNO08X initialized successfully");
       }
       
       // Main loop
       while (true) {
           // Read encoder
           uint16_t angle;
           if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
               Logger::Info("ENCODER", "Angle: %d LSB", angle);
           }
           
           // Read IMU
           Bno08xQuaternion quat;
           if (imu.ReadQuaternion(quat) == Bno08xError::SUCCESS) {
               Logger::Info("IMU", "Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                           quat.w, quat.x, quat.y, quat.z);
           }
           
           vTaskDelay(pdMS_TO_TICKS(100));
       }
   }
   ```

6. **Build and Flash:**
   ```bash
   # Build project
   idf.py build
   
   # Flash to device
   idf.py flash
   
   # Monitor output
   idf.py monitor
   ```

#### ESP32 Configuration

**sdkconfig Settings:**
```ini
# Enable SPI
CONFIG_SPI_MASTER=y
CONFIG_SPI_MASTER_IN_IRAM=y

# Enable I2C
CONFIG_I2C_MASTER=y

# Enable GPIO
CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL=y

# Enable FreeRTOS
CONFIG_FREERTOS=y
CONFIG_FREERTOS_HZ=1000

# Enable logging
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_COLORS=y
```

### 🟨 STM32 Integration

#### Prerequisites

- **STM32CubeIDE** or **STM32CubeMX**
- **STM32 HAL** libraries
- **CMake** 3.16+
- **ARM GCC** toolchain

#### Setup Instructions

1. **Install STM32CubeIDE:**
   - Download from [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
   - Install with STM32CubeMX

2. **Create STM32 Project:**
   ```bash
   # Create project directory
   mkdir my_stm32_hardfoc_project
   cd my_stm32_hardfoc_project
   ```

3. **Add HardFOC Library:**
   ```bash
   # Clone HardFOC library
   git clone https://github.com/hardfoc/hf-utils-and-drivers.git
   cd hf-utils-and-drivers
   git submodule update --init --recursive
   cd ..
   ```

4. **Configure CMakeLists.txt:**
   ```cmake
   cmake_minimum_required(VERSION 3.16)
   
   # Set project name
   project(my_stm32_hardfoc_project)
   
   # Set C++ standard
   set(CMAKE_CXX_STANDARD 17)
   set(CMAKE_CXX_STANDARD_REQUIRED ON)
   
   # Set MCU
   set(MCU STM32F407VGT6)
   set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
   
   # Add HardFOC library
   add_subdirectory(hf-utils-and-drivers)
   
   # Add source files
   file(GLOB_RECURSE SOURCES 
        "src/*.cpp" 
        "src/*.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
        "Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c"
        "Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s"
   )
   
   # Create executable
   add_executable(${PROJECT_NAME}.elf ${SOURCES})
   
   # Set MCU flags
   target_compile_options(${PROJECT_NAME}.elf PRIVATE ${CPU_PARAMETERS})
   target_link_options(${PROJECT_NAME}.elf PRIVATE ${CPU_PARAMETERS})
   
   # Include directories
   target_include_directories(${PROJECT_NAME}.elf PRIVATE
       src
       Drivers/STM32F4xx_HAL_Driver/Inc
       Drivers/CMSIS/Device/ST/STM32F4xx/Include
       Drivers/CMSIS/Include
   )
   
   # Link HardFOC library
   target_link_libraries(${PROJECT_NAME}.elf hf-utils-and-drivers)
   
   # Link script
   target_link_script(${PROJECT_NAME}.elf ${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld)
   ```

5. **Example STM32 Application:**
   ```cpp
   #include "driver-handlers/As5047uHandler.h"
   #include "driver-handlers/Logger.h"
   #include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/stm32/Stm32Spi.h"
   #include "main.h"
   #include "stm32f4xx_hal.h"
   
   // Global variables
   SPI_HandleTypeDef hspi1;
   UART_HandleTypeDef huart2;
   
   // HardFOC interfaces
   Stm32Spi spi_interface;
   As5047uHandler encoder;
   
   int main(void) {
       // HAL initialization
       HAL_Init();
       SystemClock_Config();
       
       // Initialize peripherals
       MX_GPIO_Init();
       MX_SPI1_Init();
       MX_USART2_UART_Init();
       
       // Configure logger
       LogConfig config;
       config.level = LogLevel::INFO;
       config.enable_colors = false; // No colors for UART
       Logger::SetConfig(config);
       
       Logger::Info("SYSTEM", "HardFOC STM32 Application Starting...");
       
       // Initialize SPI interface
       spi_interface.Initialize(&hspi1);
       
       // Create encoder handler
       encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
       
       // Initialize encoder
       if (encoder.Initialize() == As5047uError::SUCCESS) {
           Logger::Info("ENCODER", "AS5047U initialized successfully");
       } else {
           Logger::Error("ENCODER", "AS5047U initialization failed");
       }
       
       // Main loop
       while (1) {
           // Read encoder
           uint16_t angle;
           if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
               Logger::Info("ENCODER", "Angle: %d LSB", angle);
           }
           
           HAL_Delay(100);
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

6. **Build and Flash:**
   ```bash
   # Create build directory
   mkdir build
   cd build
   
   # Configure with CMake
   cmake ..
   
   # Build
   make -j4
   
   # Flash using ST-Link
   st-flash write ${PROJECT_NAME}.bin 0x08000000
   ```

### 🟥 Arduino Integration

#### Prerequisites

- **Arduino IDE** 1.8.19+ or **Arduino CLI**
- **Arduino Core** for your board
- **CMake** 3.16+ (for advanced builds)

#### Setup Instructions

1. **Install Arduino IDE:**
   - Download from [Arduino.cc](https://www.arduino.cc/en/software)
   - Install Arduino core for your board

2. **Add HardFOC Library:**
   ```bash
   # Navigate to Arduino libraries directory
   cd ~/Arduino/libraries
   
   # Clone HardFOC library
   git clone https://github.com/hardfoc/hf-utils-and-drivers.git
   cd hf-utils-and-drivers
   git submodule update --init --recursive
   ```

3. **Example Arduino Sketch:**
   ```cpp
   #include "driver-handlers/As5047uHandler.h"
   #include "driver-handlers/Logger.h"
   #include "hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/arduino/ArduinoSpi.h"
   
   // Hardware configuration
   const int CS_PIN = 10;
   const int MOSI_PIN = 11;
   const int MISO_PIN = 12;
   const int SCK_PIN = 13;
   
   // HardFOC interfaces
   ArduinoSpi spi_interface;
   As5047uHandler encoder;
   
   void setup() {
       // Initialize serial communication
       Serial.begin(115200);
       while (!Serial) {
           delay(10);
       }
       
       // Configure logger
       LogConfig config;
       config.level = LogLevel::INFO;
       config.enable_colors = false; // No colors for Serial
       Logger::SetConfig(config);
       
       Logger::Info("SYSTEM", "HardFOC Arduino Application Starting...");
       
       // Initialize SPI
       spi_interface.Initialize(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
       
       // Create encoder handler
       encoder = As5047uHandler(spi_interface, "MOTOR_ENCODER");
       
       // Initialize encoder
       if (encoder.Initialize() == As5047uError::SUCCESS) {
           Logger::Info("ENCODER", "AS5047U initialized successfully");
       } else {
           Logger::Error("ENCODER", "AS5047U initialization failed");
       }
   }
   
   void loop() {
       // Read encoder
       uint16_t angle;
       if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
           Logger::Info("ENCODER", "Angle: %d LSB", angle);
       }
       
       delay(100);
   }
   ```

4. **Build and Upload:**
   - Open Arduino IDE
   - Select your board
   - Select the correct port
   - Click "Upload"

---

## 🔧 Advanced Integration

### 🏗️ Custom Platform Integration

To integrate with a custom platform, you need to implement the base interfaces:

#### 1. Implement Base Interfaces

```cpp
// Example: Custom SPI implementation
class CustomSpi : public BaseSpi {
private:
    // Your platform-specific SPI handle
    CustomSpiHandle_t spi_handle_;
    
public:
    bool Initialize() noexcept override {
        // Initialize your SPI hardware
        return CustomSpiInit(&spi_handle_) == CUSTOM_SPI_OK;
    }
    
    bool Transfer(const uint8_t* tx, uint8_t* rx, size_t len) noexcept override {
        // Perform SPI transfer
        return CustomSpiTransfer(&spi_handle_, tx, rx, len) == CUSTOM_SPI_OK;
    }
    
    bool Deinitialize() noexcept override {
        // Cleanup SPI hardware
        return CustomSpiDeinit(&spi_handle_) == CUSTOM_SPI_OK;
    }
};

// Example: Custom I2C implementation
class CustomI2c : public BaseI2c {
private:
    CustomI2cHandle_t i2c_handle_;
    uint8_t device_address_;
    
public:
    bool Initialize() noexcept override {
        return CustomI2cInit(&i2c_handle_) == CUSTOM_I2C_OK;
    }
    
    bool Write(const uint8_t* data, size_t len) noexcept override {
        return CustomI2cWrite(&i2c_handle_, device_address_, data, len) == CUSTOM_I2C_OK;
    }
    
    bool Read(uint8_t* data, size_t len) noexcept override {
        return CustomI2cRead(&i2c_handle_, device_address_, data, len) == CUSTOM_I2C_OK;
    }
    
    bool Deinitialize() noexcept override {
        return CustomI2cDeinit(&i2c_handle_) == CUSTOM_I2C_OK;
    }
};
```

#### 2. Use Custom Interfaces

```cpp
// Create custom interfaces
CustomSpi custom_spi;
CustomI2c custom_i2c;

// Initialize interfaces
custom_spi.Initialize();
custom_i2c.Initialize();

// Create handlers with custom interfaces
As5047uHandler encoder(custom_spi, "CUSTOM_ENCODER");
Bno08xHandler imu(custom_i2c, "CUSTOM_IMU");

// Use handlers normally
encoder.Initialize();
imu.Initialize();
```

### 🔧 CMake Integration

#### Advanced CMake Configuration

```cmake
# Advanced CMake configuration
cmake_minimum_required(VERSION 3.16)

# Set project
project(MyHardFocProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific settings
if(CMAKE_SYSTEM_NAME STREQUAL "ESP32")
    # ESP32 specific settings
    set(CMAKE_C_COMPILER xtensa-esp32-elf-gcc)
    set(CMAKE_CXX_COMPILER xtensa-esp32-elf-g++)
    set(CMAKE_SYSTEM_PROCESSOR xtensa)
    
    # ESP-IDF integration
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    
elseif(CMAKE_SYSTEM_NAME STREQUAL "STM32")
    # STM32 specific settings
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_SYSTEM_PROCESSOR ARM)
    
    # STM32 flags
    set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_PARAMETERS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_PARAMETERS}")
    
elseif(CMAKE_SYSTEM_NAME STREQUAL "Arduino")
    # Arduino specific settings
    set(ARDUINO_SDK_PATH $ENV{ARDUINO_SDK_PATH})
    include(${ARDUINO_SDK_PATH}/Arduino.cmake)
    
endif()

# Add HardFOC library
add_subdirectory(hf-utils-and-drivers)

# Create main executable
add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/platform_specific.cpp
)

# Link HardFOC library
target_link_libraries(${PROJECT_NAME} hf-utils-and-drivers)

# Platform-specific linking
if(CMAKE_SYSTEM_NAME STREQUAL "ESP32")
    target_link_libraries(${PROJECT_NAME} idf::main)
elseif(CMAKE_SYSTEM_NAME STREQUAL "STM32")
    target_link_script(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Arduino")
    target_link_libraries(${PROJECT_NAME} arduino)
endif()
```

### 🔧 Cross-Platform Build

#### Multi-Platform CMakeLists.txt

```cmake
# Multi-platform CMake configuration
cmake_minimum_required(VERSION 3.16)

project(HardFocMultiPlatform)

# Platform detection
if(DEFINED ENV{IDF_PATH})
    set(PLATFORM "ESP32")
elseif(DEFINED ENV{STM32_TOOLCHAIN_PATH})
    set(PLATFORM "STM32")
elseif(DEFINED ENV{ARDUINO_SDK_PATH})
    set(PLATFORM "Arduino")
else()
    set(PLATFORM "Generic")
endif()

message(STATUS "Detected platform: ${PLATFORM}")

# Platform-specific configuration
if(PLATFORM STREQUAL "ESP32")
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    set(CMAKE_CXX_STANDARD 17)
    
elseif(PLATFORM STREQUAL "STM32")
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_SYSTEM_PROCESSOR ARM)
    set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_PARAMETERS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_PARAMETERS}")
    
elseif(PLATFORM STREQUAL "Arduino")
    set(ARDUINO_SDK_PATH $ENV{ARDUINO_SDK_PATH})
    include(${ARDUINO_SDK_PATH}/Arduino.cmake)
    
else()
    # Generic platform
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add HardFOC library
add_subdirectory(hf-utils-and-drivers)

# Create executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Platform-specific source files
if(PLATFORM STREQUAL "ESP32")
    target_sources(${PROJECT_NAME} PRIVATE src/esp32_platform.cpp)
elseif(PLATFORM STREQUAL "STM32")
    target_sources(${PROJECT_NAME} PRIVATE src/stm32_platform.cpp)
elseif(PLATFORM STREQUAL "Arduino")
    target_sources(${PROJECT_NAME} PRIVATE src/arduino_platform.cpp)
else()
    target_sources(${PROJECT_NAME} PRIVATE src/generic_platform.cpp)
endif()

# Link HardFOC library
target_link_libraries(${PROJECT_NAME} hf-utils-and-drivers)

# Platform-specific linking
if(PLATFORM STREQUAL "ESP32")
    target_link_libraries(${PROJECT_NAME} idf::main)
elseif(PLATFORM STREQUAL "STM32")
    target_link_script(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld)
elseif(PLATFORM STREQUAL "Arduino")
    target_link_libraries(${PROJECT_NAME} arduino)
endif()
```

---

## 🔧 Configuration and Customization

### 🎨 Logger Configuration

```cpp
// Production configuration
LogConfig production_config;
production_config.level = LogLevel::WARN;
production_config.enable_colors = false;
production_config.enable_effects = false;
production_config.max_width = 120;
Logger::SetConfig(production_config);

// Development configuration
LogConfig dev_config;
dev_config.level = LogLevel::DEBUG;
dev_config.enable_colors = true;
dev_config.enable_effects = true;
dev_config.max_width = 80;
Logger::SetConfig(dev_config);
```

### 🔧 Handler Configuration

```cpp
// AS5047U configuration
As5047uConfig encoder_config;
encoder_config.enable_daec = true;
encoder_config.enable_adaptive_filter = true;
encoder_config.zero_position = 8192;
As5047uHandler encoder(spi, "MOTOR_ENCODER", encoder_config);

// BNO08X configuration
Bno08xConfig imu_config;
imu_config.accel_range = Bno08xAccelRange::RANGE_4G;
imu_config.gyro_range = Bno08xGyroRange::RANGE_500DPS;
imu_config.default_report_interval_ms = 10;
Bno08xHandler imu(i2c, "MAIN_IMU", imu_config);
```

### 🔧 Thread Safety Configuration

```cpp
// Configure thread safety
RtosMutex::SetTimeout(1000); // 1 second timeout

// Use in multi-threaded applications
void SensorThread(void* parameter) {
    while (true) {
        uint16_t angle;
        if (encoder.ReadAngle(angle) == As5047uError::SUCCESS) {
            // Process angle data
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ControlThread(void* parameter) {
    while (true) {
        // Motor control logic
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

---

## 🐛 Troubleshooting

### 🔍 Common Integration Issues

#### 1. Compilation Errors

**Symptoms:** CMake or compiler errors
**Causes:**
- Missing dependencies
- Incorrect C++ standard
- Platform-specific issues

**Solutions:**
- Verify C++17 support
- Check all submodules are initialized
- Ensure platform-specific tools are installed

#### 2. Linker Errors

**Symptoms:** Undefined reference errors
**Causes:**
- Missing library linkage
- Incorrect library paths
- Platform mismatch

**Solutions:**
- Verify `target_link_libraries()` call
- Check library paths in CMake
- Ensure platform compatibility

#### 3. Runtime Errors

**Symptoms:** Initialization failures or crashes
**Causes:**
- Incorrect hardware configuration
- Missing platform initialization
- Memory issues

**Solutions:**
- Verify hardware connections
- Check platform initialization order
- Monitor memory usage

### 🔧 Debugging Tips

1. **Enable Verbose Logging:**
   ```cpp
   Logger::SetLogLevel("SYSTEM", LogLevel::VERBOSE);
   Logger::SetLogLevel("SENSOR", LogLevel::VERBOSE);
   ```

2. **Check Error Codes:**
   ```cpp
   auto result = encoder.Initialize();
   if (result != As5047uError::SUCCESS) {
       Logger::Error("SYSTEM", "Encoder init failed: %s", 
                    As5047uErrorToString(result));
   }
   ```

3. **Monitor System Resources:**
   ```cpp
   // Check available memory
   size_t free_heap = esp_get_free_heap_size(); // ESP32
   Logger::Info("SYSTEM", "Free heap: %d bytes", free_heap);
   ```

---

## 📚 Related Documentation

- **[Architecture Overview](architecture.md)** - System design and patterns
- **[API Reference](api/)** - Complete API documentation
- **[Examples](examples/)** - Practical usage examples
- **[Component Documentation](handlers/)** - Individual component guides 