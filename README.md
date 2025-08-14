# Universal Arduino Port & I2C Scanner

🔍 **Universal port and I2C device scanner** for Arduino, ESP32, and ESP8266 microcontrollers.

Automatically detects active ports, connected sensors, and I2C devices with automatic identification.

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat&logo=arduino&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-000000?style=flat&logo=espressif&logoColor=white)
![ESP8266](https://img.shields.io/badge/ESP8266-000000?style=flat&logo=espressif&logoColor=white)
![PlatformIO](https://img.shields.io/badge/PlatformIO-FF7F00?style=flat&logo=platformio&logoColor=white)

## ✨ Features

- **🔌 Digital Port Scanning** - detects active pins with logic level indication
- **📊 Analog Port Scanning** - shows ADC values and voltage readings
- **🔍 I2C Scanning** - finds devices by address with automatic identification
- **🎯 Board Auto-detection** - automatically configures for microcontroller type
- **💾 Memory Optimization** - works even on Arduino NANO with 2KB RAM
- **🔄 Real-time Monitoring** - automatic rescanning every 15 seconds

## 🎯 Supported Boards

| Board | Status | Digital Pins | Analog Pins | I2C Pins |
|-------|--------|--------------|-------------|----------|
| Arduino UNO | ✅ | D2-D13 | A0-A5 | SDA=A4, SCL=A5 |
| Arduino NANO | ✅ | D2-D13 | A0-A5 | SDA=A4, SCL=A5 |
| Arduino Pro Mini | ✅ | D2-D13 | A0-A5 | SDA=A4, SCL=A5 |
| Arduino MEGA | ✅ | D2-D53 | A0-A15 | SDA=20, SCL=21 |
| ESP32 | ✅ | GPIO2-GPIO39 | ADC1/ADC2 | SDA=21, SCL=22 |
| ESP8266 | ✅ | D1-D8 | A0 | SDA=4, SCL=5 |

## 🔧 Recognized I2C Devices (40+)

- **Displays:** OLED SSD1306 (0x3C, 0x3D)
- **Sensors:** BMP280/BME280 (0x76, 0x77), MPU6050 (0x68, 0x69)
- **Modules:** RTC DS1307 (0x68), ADS1115 ADC (0x48-0x4B)
- **Expanders:** MCP23017 I/O (0x20-0x27), PCA9685 PWM (0x40-0x41)
- **Memory:** EEPROM 24LC (0x50-0x57)
- **And many more...**

## 🚀 Usage

1. **Upload the sketch** to your board
2. **Open Serial Monitor** at **115200 baud**
3. **Connect devices** to ports or I2C bus
4. **Watch the results** of real-time scanning

## ⚙️ Configuration

### Activity Thresholds
You can modify activity detection thresholds in the code:
```
// Analog ports (change range)
if (value > 50 && value < (ADC_RESOLUTION - 100)) {
    // Port is considered active
}
```

### Scan Frequency
```
// In loop() function, change delay
delay(15000);  // 15 seconds between scans
```

## 🛠️ Technical Details

- **Language:** C++ (Arduino Framework)
- **Libraries:** Wire.h, Arduino.h
- **RAM Usage:** ~1.2KB
- **Flash Usage:** ~8KB
- **Compatibility:** Arduino IDE 1.8+, PlatformIO Core 6.0+

## 🤝 Contributing

Contributions are welcome! If you want to add:
- Support for new boards
- Detection of additional I2C devices
- Code optimization
- Bug fixes

Create a **Pull Request** or **Issue**.

## 🔧 Troubleshooting

### Common Issues

**"No I2C devices found"**
- Check wiring connections
- Ensure 4.7kΩ pull-up resistors on SDA and SCL lines
- Verify device power supply
- Test with a known working I2C device

**High analog readings on unused pins**
- This is normal "floating" behavior
- Connect unused analog pins to GND through 10kΩ resistor if needed
- Values between 50-950 (Arduino) or 50-4000 (ESP32) are considered "active"

**Compilation errors on Arduino NANO**
- Ensure you're using the latest Arduino IDE
- Check that Wire library is properly installed
- Reduce scan frequency if memory issues persist

### Pin Mapping Reference

**Arduino UNO/NANO/Pro Mini:**
- Digital: D0/D1 (Serial), D2-D13 (GPIO)
- Analog: A0-A5 (also usable as digital D14-D19)
- I2C: SDA=A4, SCL=A5

**ESP32:**
- Digital: GPIO0-GPIO39 (some are input-only)
- Analog: ADC1 (GPIO32-GPIO39), ADC2 (GPIO0,2,4,12-15,25-27)
- I2C: Default SDA=21, SCL=22 (configurable)

**ESP8266:**
- Digital: D0-D8 (NodeMCU labeling)
- Analog: A0 (single ADC input)
- I2C: Default SDA=4(D2), SCL=5(D1)

---
⭐ **If this project helped you, please give it a star!** ⭐
