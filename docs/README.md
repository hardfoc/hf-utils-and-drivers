# HardFOC Documentation

Welcome to the comprehensive documentation for the **HardFOC Utils and Drivers** library. This documentation provides detailed information about all components, handlers, and utilities in the library.

## 📚 Documentation Structure

### 🏗️ Architecture & Design

- **[Architecture Overview](architecture.md)** - Complete architectural overview, design patterns, and system design
- **[Integration Guide](integration.md)** - Platform-specific integration instructions and examples

### 🔧 Handler Documentation

#### Core Handlers
- **[AS5047U Handler](handlers/as5047u-handler.md)** - Magnetic rotary position sensor handler
- **[BNO08X Handler](handlers/bno08x-handler.md)** - 9-DOF IMU sensor handler with sensor fusion
- **[TMC9660 Handler](handlers/tmc9660-handler.md)** - BLDC motor controller with integrated GPIO/ADC
- **[PCAL95555 Handler](handlers/pcal95555-handler.md)** - 16-bit I2C GPIO expander handler
- **[NTC Temperature Handler](handlers/ntc-temperature-handler.md)** - NTC thermistor temperature sensor handler

#### Utility Handlers
- **[Logger](handlers/logger.md)** - Advanced logging system with colors, styles, and ASCII art

### 📖 Examples & Tutorials

- **[Basic Setup](examples/basic-setup.md)** - Getting started with the library
- **[Advanced Examples](examples/advanced-examples.md)** - Complex integration scenarios *(Coming Soon)*

### 🔌 API Reference

#### Base Interfaces
- **[Base SPI Interface](api/base-spi.md)** - SPI communication interface *(Coming Soon)*
- **[Base I2C Interface](api/base-i2c.md)** - I2C communication interface *(Coming Soon)*
- **[Base UART Interface](api/base-uart.md)** - UART communication interface *(Coming Soon)*
- **[Base GPIO Interface](api/base-gpio.md)** - GPIO control interface *(Coming Soon)*
- **[Base ADC Interface](api/base-adc.md)** - ADC measurement interface *(Coming Soon)*
- **[Base Temperature Interface](api/base-temperature.md)** - Temperature sensor interface *(Coming Soon)*

#### Utilities
- **[Error Handling Guide](api/error-handling.md)** - Error codes and handling patterns *(Coming Soon)*
- **[Thread Safety Guide](api/thread-safety.md)** - Thread safety patterns and best practices *(Coming Soon)*

### 🚀 Core Components

#### Drivers
- **[Core Drivers](drivers/README.md)** - Low-level hardware drivers *(Coming Soon)*
  - AS5047U Driver
  - BNO08X Driver
  - TMC9660 Driver
  - PCAL95555 Driver
  - NTC Thermistor Driver

#### Utilities
- **[Core Utils](utils/README.md)** - Utility libraries and helpers *(Coming Soon)*
  - General Utilities
  - CANopen Utilities
  - RTOS Wrappers
  - Thread-Safe Utilities

## 🎯 Quick Start

### 1. Choose Your Platform

The library supports multiple platforms:

- **ESP32** - Full support with ESP-IDF
- **STM32** - Full support with STM32 HAL
- **Arduino** - Full support with Arduino framework
- **Generic** - Platform-independent interfaces

### 2. Select Your Handlers

Based on your hardware requirements:

| Hardware | Handler | Key Features |
|----------|---------|--------------|
| Magnetic Encoder | AS5047U Handler | 14-bit angle, velocity, DAEC |
| IMU Sensor | BNO08X Handler | 9-DOF fusion, motion detection |
| Motor Controller | TMC9660 Handler | BLDC control, integrated GPIO/ADC |
| GPIO Expander | PCAL95555 Handler | 16-bit I2C GPIO expansion |
| Temperature Sensor | NTC Handler | Thermistor with multiple conversion methods |

### 3. Follow Integration Guide

See the **[Integration Guide](integration.md)** for platform-specific setup instructions.

## 🔍 Finding What You Need

### By Hardware Component

Looking for documentation on a specific hardware component?

- **Sensors**: Check the handler documentation in `handlers/`
- **Communication**: See the API reference in `api/`
- **Platform Setup**: Use the integration guide

### By Use Case

Common use cases and where to find help:

| Use Case | Documentation | Examples |
|----------|---------------|----------|
| Motor Control | TMC9660 Handler | Basic Setup, Advanced Examples |
| Position Sensing | AS5047U Handler | Basic Setup |
| Motion Detection | BNO08X Handler | Basic Setup |
| GPIO Expansion | PCAL95555 Handler | Basic Setup |
| Temperature Monitoring | NTC Handler | Basic Setup |
| Logging & Debugging | Logger | All Examples |

### By Platform

Platform-specific documentation:

- **ESP32**: See [Integration Guide](integration.md#esp32-integration)
- **STM32**: See [Integration Guide](integration.md#stm32-integration)
- **Arduino**: See [Integration Guide](integration.md#arduino-integration)

## 🛠️ Development Resources

### Contributing

- **[Contributing Guide](../CONTRIBUTING.md)** - How to contribute to the project
- **[Code Standards](../CONTRIBUTING.md#code-standards)** - C++ and documentation standards
- **[Testing Guidelines](../CONTRIBUTING.md#testing)** - Testing requirements and procedures

### Building & Testing

- **[Building Examples](examples/building-examples.md)** - How to build and test examples *(Coming Soon)*
- **[Platform Setup](examples/platform-setup.md)** - Development environment setup *(Coming Soon)*

## 📋 Documentation Status

### ✅ Completed

- [x] Architecture Overview
- [x] Integration Guide
- [x] All Handler Documentation
- [x] Basic Setup Examples
- [x] Contributing Guide

### 🚧 In Progress

- [ ] API Reference Documentation
- [ ] Advanced Examples
- [ ] Core Drivers Documentation
- [ ] Core Utils Documentation

### 📅 Planned

- [ ] Video Tutorials
- [ ] Interactive Examples
- [ ] Performance Benchmarks
- [ ] Migration Guides

## 🤝 Getting Help

### Documentation Issues

If you find issues with the documentation:

1. Check the [Contributing Guide](../CONTRIBUTING.md)
2. Submit an issue with the `documentation` label
3. Consider contributing a fix

### Code Issues

For code-related issues:

1. Check the troubleshooting sections in handler documentation
2. Review the [Error Handling Guide](api/error-handling.md) *(Coming Soon)*
3. Submit an issue with appropriate labels

### Community Support

- **GitHub Issues**: For bugs and feature requests
- **Discussions**: For questions and community help *(Coming Soon)*
- **Examples**: Check the examples directory for working code

## 📊 Documentation Metrics

- **Total Pages**: 8+ comprehensive documentation files
- **Code Examples**: 50+ working code examples
- **Platforms Covered**: 4 (ESP32, STM32, Arduino, Generic)
- **Handlers Documented**: 6 (All major handlers)
- **Architecture Diagrams**: 10+ Mermaid diagrams

## 🔄 Documentation Updates

This documentation is actively maintained and updated with each release. Check the [changelog](../CHANGELOG.md) for recent updates.

---

**Need help?** Start with the [Basic Setup](examples/basic-setup.md) or check the [Integration Guide](integration.md) for your platform. 