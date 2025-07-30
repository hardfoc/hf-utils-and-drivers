# Contributing to HardFOC Utils and Drivers

<div align="center">

![Contributing](https://img.shields.io/badge/Contributing-Guide-blue?style=for-the-badge)
![Welcome](https://img.shields.io/badge/Welcome-Contributors-green?style=for-the-badge)
![Community](https://img.shields.io/badge/Community-Driven-orange?style=for-the-badge)

**Thank you for your interest in contributing to HardFOC Utils and Drivers!**

</div>

---

## 🎯 Overview

We welcome contributions from the community! This guide will help you understand how to contribute to the HardFOC Utils and Drivers project, whether you're fixing bugs, adding features, improving documentation, or helping with testing.

---

## 📋 Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Code Standards](#code-standards)
- [Testing Guidelines](#testing-guidelines)
- [Documentation Guidelines](#documentation-guidelines)
- [Pull Request Process](#pull-request-process)
- [Release Process](#release-process)
- [Community Guidelines](#community-guidelines)

---

## 🤝 Code of Conduct

### Our Standards

We are committed to providing a welcoming and inspiring community for all. By participating in this project, you agree to:

- **Be respectful and inclusive** - Treat everyone with respect and dignity
- **Be collaborative** - Work together to achieve common goals
- **Be constructive** - Provide helpful, constructive feedback
- **Be professional** - Maintain professional behavior in all interactions

### Unacceptable Behavior

- Harassment, discrimination, or bullying
- Trolling, insulting, or derogatory comments
- Publishing others' private information without permission
- Any conduct inappropriate in a professional setting

### Enforcement

Violations of the Code of Conduct may result in temporary or permanent exclusion from the project community.

---

## 🚀 Getting Started

### Types of Contributions

We welcome various types of contributions:

- **🐛 Bug Fixes** - Fix issues and improve reliability
- **✨ New Features** - Add new functionality and capabilities
- **📚 Documentation** - Improve guides, examples, and API docs
- **🧪 Testing** - Add tests and improve test coverage
- **🔧 Tools** - Improve build tools and development workflow
- **🌐 Translations** - Help with internationalization
- **💡 Ideas** - Suggest improvements and new features

### Before You Start

1. **Check existing issues** - Search for similar issues or feature requests
2. **Discuss major changes** - Open an issue to discuss significant changes
3. **Read the documentation** - Understand the project structure and patterns
4. **Set up your environment** - Follow the development setup guide

---

## 🔧 Development Setup

### Prerequisites

- **C++17** compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- **CMake** 3.16 or higher
- **Git** for version control
- **Platform-specific tools** (ESP-IDF, STM32 HAL, Arduino)

### Setup Instructions

1. **Fork the repository:**
   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/YOUR_USERNAME/hf-utils-and-drivers.git
   cd hf-utils-and-drivers
   ```

2. **Add upstream remote:**
   ```bash
   git remote add upstream https://github.com/hardfoc/hf-utils-and-drivers.git
   ```

3. **Initialize submodules:**
   ```bash
   git submodule update --init --recursive
   ```

4. **Create development branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```

5. **Build the project:**
   ```bash
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

### Development Workflow

1. **Keep your fork updated:**
   ```bash
   git fetch upstream
   git checkout main
   git merge upstream/main
   ```

2. **Create feature branches:**
   ```bash
   git checkout -b feature/descriptive-feature-name
   ```

3. **Make your changes:**
   - Write code following our standards
   - Add tests for new functionality
   - Update documentation

4. **Test your changes:**
   ```bash
   # Run unit tests
   make test
   
   # Run specific test suite
   ctest -R "handler_tests"
   ```

5. **Commit your changes:**
   ```bash
   git add .
   git commit -m "feat: add new feature description"
   ```

---

## 📝 Code Standards

### C++ Coding Standards

#### General Guidelines

- **C++17** - Use modern C++ features appropriately
- **Consistent formatting** - Use clang-format with our configuration
- **Clear naming** - Use descriptive names for variables, functions, and classes
- **Documentation** - Document all public APIs with Doxygen comments

#### Code Style

```cpp
// Good: Clear, descriptive naming
class As5047uHandler {
private:
    BaseSpi& spi_interface_;
    std::string sensor_name_;
    mutable RtosMutex mutex_;
    
public:
    explicit As5047uHandler(BaseSpi& spi_interface, 
                           const std::string& sensor_name = "AS5047U") noexcept;
    
    As5047uError Initialize() noexcept;
    As5047uError ReadAngle(uint16_t& angle) noexcept;
};

// Good: Exception safety
void SafeOperation() noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    // Critical section
}

// Good: Error handling
As5047uError ReadSensorData() noexcept {
    if (!IsInitialized()) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    // Perform operation
    return As5047uError::SUCCESS;
}
```

#### Naming Conventions

- **Classes**: PascalCase (e.g., `As5047uHandler`)
- **Functions**: PascalCase (e.g., `Initialize()`, `ReadAngle()`)
- **Variables**: snake_case (e.g., `spi_interface_`, `sensor_name_`)
- **Constants**: UPPER_SNAKE_CASE (e.g., `MAX_RETRY_COUNT`)
- **Enums**: PascalCase (e.g., `As5047uError`)

#### File Organization

```
driver-handlers/
├── As5047uHandler.h      # Header file
├── As5047uHandler.cpp    # Implementation file
├── Bno08xHandler.h
├── Bno08xHandler.cpp
└── ...
```

### Documentation Standards

#### Doxygen Comments

```cpp
/**
 * @brief Unified handler for AS5047U magnetic rotary position sensor.
 *
 * This class provides a modern, unified interface for the AS5047U sensor
 * with SPI integration, thread safety, and comprehensive error handling.
 *
 * @example
 * ```cpp
 * BaseSpi& spi = GetPlatformSpi();
 * As5047uHandler encoder(spi, "MOTOR_ENCODER");
 * if (encoder.Initialize() == As5047uError::SUCCESS) {
 *     uint16_t angle;
 *     encoder.ReadAngle(angle);
 * }
 * ```
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 */
class As5047uHandler {
public:
    /**
     * @brief Initialize the sensor and establish communication.
     * @return As5047uError::SUCCESS if initialization successful
     * @note This method is thread-safe and can be called multiple times.
     */
    As5047uError Initialize() noexcept;
};
```

#### README Documentation

- Use clear, concise language
- Include code examples
- Provide troubleshooting sections
- Use consistent formatting with badges and emojis

---

## 🧪 Testing Guidelines

### Test Structure

```cpp
// Test file: test_as5047u_handler.cpp
#include <gtest/gtest.h>
#include "driver-handlers/As5047uHandler.h"

class As5047uHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup test fixtures
    }
    
    void TearDown() override {
        // Cleanup test fixtures
    }
    
    // Test fixtures
    MockSpi mock_spi_;
    As5047uHandler handler_;
};

TEST_F(As5047uHandlerTest, InitializationSuccess) {
    // Arrange
    EXPECT_CALL(mock_spi_, Initialize()).WillOnce(Return(true));
    
    // Act
    auto result = handler_.Initialize();
    
    // Assert
    EXPECT_EQ(result, As5047uError::SUCCESS);
}

TEST_F(As5047uHandlerTest, ReadAngleSuccess) {
    // Arrange
    handler_.Initialize();
    uint16_t expected_angle = 8192; // 180 degrees
    EXPECT_CALL(mock_spi_, Transfer(_, _, _))
        .WillOnce(DoAll(SetArrayArgument<1>(expected_angle), Return(true)));
    
    // Act
    uint16_t actual_angle;
    auto result = handler_.ReadAngle(actual_angle);
    
    // Assert
    EXPECT_EQ(result, As5047uError::SUCCESS);
    EXPECT_EQ(actual_angle, expected_angle);
}
```

### Test Categories

1. **Unit Tests** - Test individual functions and classes
2. **Integration Tests** - Test component interactions
3. **Hardware Tests** - Test with real hardware
4. **Performance Tests** - Test performance characteristics

### Running Tests

```bash
# Run all tests
make test

# Run specific test suite
ctest -R "as5047u_tests"

# Run with verbose output
ctest -V -R "as5047u_tests"

# Run with coverage
make coverage
```

---

## 📚 Documentation Guidelines

### Documentation Structure

```
docs/
├── README.md                 # Main project documentation
├── architecture.md          # System architecture overview
├── integration.md           # Integration guide
├── handlers/                # Handler documentation
│   ├── as5047u-handler.md
│   ├── bno08x-handler.md
│   └── ...
├── examples/                # Usage examples
│   ├── basic-setup.md
│   ├── advanced-examples.md
│   └── ...
└── api/                     # API reference
    ├── base-spi.md
    ├── base-i2c.md
    └── ...
```

### Documentation Standards

1. **Clear Structure** - Use consistent headings and organization
2. **Code Examples** - Include practical, working examples
3. **Visual Elements** - Use badges, diagrams, and emojis appropriately
4. **Cross-References** - Link related documentation
5. **Regular Updates** - Keep documentation current with code changes

### Writing Documentation

```markdown
# Component Name

<div align="center">

![Component](https://img.shields.io/badge/Component-Name-blue?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Stable-green?style=for-the-badge)

**Brief description of the component**

</div>

---

## 🎯 Overview

Detailed description of what the component does and why it's useful.

## 📋 API Reference

### Constructor

```cpp
explicit ComponentName(Interface& interface, const std::string& name = "Component");
```

**Parameters:**
- `interface`: Description of the interface parameter
- `name`: Optional name for the component

## 💡 Usage Examples

### Basic Usage

```cpp
// Example code here
```

## 🔧 Configuration

Configuration options and examples.

## 🐛 Troubleshooting

Common issues and solutions.
```

---

## 🔄 Pull Request Process

### Before Submitting

1. **Ensure code quality:**
   - Follow coding standards
   - Add appropriate tests
   - Update documentation
   - Run all tests successfully

2. **Check for issues:**
   - No compilation warnings
   - All tests passing
   - Documentation updated
   - Code reviewed by yourself

### Creating a Pull Request

1. **Fork and clone** the repository
2. **Create a feature branch** from main
3. **Make your changes** following our standards
4. **Test thoroughly** on multiple platforms
5. **Update documentation** as needed
6. **Commit with clear messages** using conventional commits
7. **Push to your fork** and create a pull request

### Pull Request Template

```markdown
## Description

Brief description of the changes made.

## Type of Change

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] Test addition or update

## Testing

- [ ] Unit tests added/updated
- [ ] Integration tests added/updated
- [ ] Hardware tests performed
- [ ] All tests passing

## Platforms Tested

- [ ] ESP32
- [ ] STM32
- [ ] Arduino
- [ ] Other: ________

## Checklist

- [ ] Code follows the project's style guidelines
- [ ] Self-review of code has been performed
- [ ] Code is self-documenting
- [ ] Documentation has been updated
- [ ] No new warnings are generated
- [ ] Tests have been added/updated
- [ ] All tests pass

## Related Issues

Closes #(issue number)
```

### Review Process

1. **Automated checks** must pass
2. **Code review** by maintainers
3. **Testing** on multiple platforms
4. **Documentation review** for completeness
5. **Final approval** from maintainers

---

## 🚀 Release Process

### Versioning

We follow [Semantic Versioning](https://semver.org/):

- **MAJOR** version for incompatible API changes
- **MINOR** version for backwards-compatible functionality additions
- **PATCH** version for backwards-compatible bug fixes

### Release Steps

1. **Prepare release branch:**
   ```bash
   git checkout -b release/v1.2.0
   ```

2. **Update version numbers:**
   - Update CMakeLists.txt version
   - Update documentation version references
   - Update changelog

3. **Final testing:**
   - Run full test suite
   - Test on all supported platforms
   - Verify documentation

4. **Create release:**
   - Merge to main branch
   - Create GitHub release
   - Tag the release
   - Update documentation

### Changelog Format

```markdown
# Changelog

## [1.2.0] - 2025-01-15

### Added
- New AS5047U calibration features
- Enhanced BNO08X motion detection
- Improved error handling system

### Changed
- Updated SPI interface for better performance
- Enhanced logging system with new features

### Fixed
- Fixed memory leak in TMC9660 handler
- Resolved thread safety issue in PCAL95555
- Corrected documentation errors

### Deprecated
- Old calibration methods (will be removed in 2.0.0)

### Removed
- Legacy support for deprecated interfaces
```

---

## 👥 Community Guidelines

### Communication

- **Be respectful** - Treat everyone with respect and dignity
- **Be helpful** - Provide constructive feedback and assistance
- **Be patient** - Understand that maintainers are volunteers
- **Be clear** - Use clear, concise language in communications

### Getting Help

1. **Check documentation** - Many questions are answered in the docs
2. **Search issues** - Look for similar questions or problems
3. **Create an issue** - Use the appropriate issue template
4. **Join discussions** - Participate in community discussions

### Recognition

We recognize and appreciate all contributions:

- **Code contributors** - Listed in commit history and release notes
- **Documentation contributors** - Acknowledged in documentation
- **Bug reporters** - Listed in issue trackers
- **Community members** - Recognized for ongoing support

---

## 📞 Contact

### Questions and Support

- **GitHub Issues** - For bugs and feature requests
- **GitHub Discussions** - For questions and community discussions
- **Email** - For private or sensitive matters

### Maintainers

- **HardFOC Team** - Core development and maintenance
- **Community Maintainers** - Active community contributors

---

## 🙏 Acknowledgments

Thank you for contributing to HardFOC Utils and Drivers! Your contributions help make this project better for everyone in the embedded systems community.

---

<div align="center">

**Happy coding! 🚀**

</div> 