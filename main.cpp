#include <Arduino.h>
#include <Wire.h>

#ifdef ESP32
  #define SDA_PIN 21
  #define SCL_PIN 22
  #define ANALOG_PINS 16
  #define DIGITAL_PINS 40
  #define SUPPLY_VOLTAGE 3.3
  #define ADC_RESOLUTION 4095
#elif defined(ESP8266)
  #define SDA_PIN 4
  #define SCL_PIN 5
  #define ANALOG_PINS 1
  #define DIGITAL_PINS 17
  #define SUPPLY_VOLTAGE 3.3
  #define ADC_RESOLUTION 1023
#elif defined(__AVR_ATmega2560__)
  #define SDA_PIN 20
  #define SCL_PIN 21
  #define ANALOG_PINS 16
  #define DIGITAL_PINS 54
  #define SUPPLY_VOLTAGE 5.0
  #define ADC_RESOLUTION 1023
#else
  #define SDA_PIN A4
  #define SCL_PIN A5
  #define ANALOG_PINS 6
  #define DIGITAL_PINS 14
  #define SUPPLY_VOLTAGE 5.0
  #define ADC_RESOLUTION 1023
#endif

const __FlashStringHelper* getBoardType();
const __FlashStringHelper* getDeviceName(uint8_t addr);
void scanDigitalPorts();
void scanAnalogPorts();
void scanI2C();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(1000);
  Serial.println(F("  PORT & I2C SCANNER"));
  Serial.print(F("Board: "));
  Serial.println(getBoardType());
  Serial.print(F("Supply: "));
  Serial.print(SUPPLY_VOLTAGE, 1);
  Serial.println(F("V"));
  Serial.println(F("\n"));
  scanDigitalPorts();
  scanAnalogPorts();
  scanI2C();
}

void loop() {
  delay(15000);
  Serial.println(F("\n  RESCAN"));
  scanDigitalPorts();
  scanAnalogPorts();
  scanI2C();
}

const __FlashStringHelper* getBoardType() {
  #ifdef ESP32
    return F("ESP32");
  #elif defined(ESP8266)
    return F("ESP8266");
  #elif defined(__AVR_ATmega2560__)
    return F("Arduino MEGA");
  #else
    return F("Arduino UNO/NANO/Pro Mini");
  #endif
}

void scanDigitalPorts() {
  Serial.println(F("\n  DIGITAL PORTS"));
  uint8_t activeCount = 0;
  for (uint8_t pin = 2; pin < DIGITAL_PINS; pin++) {
    if (pin == SDA_PIN || pin == SCL_PIN) continue;
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    bool reading1 = digitalRead(pin);
    pinMode(pin, INPUT);
    delay(5);
    bool reading2 = digitalRead(pin);
    if (reading1 != reading2 || !reading1) {
      Serial.print(F("Pin D"));
      Serial.print(pin);
      Serial.print(F(": ACTIVE ("));
      Serial.print(reading1 ? F("H") : F("L"));
      Serial.print(F("/"));
      Serial.print(reading2 ? F("H") : F("L"));
      Serial.println(F(")"));
      activeCount++;
    }
  }
  if (activeCount == 0) {
    Serial.println(F("No active digital ports"));
  } else {
    Serial.print(F("Active ports: "));
    Serial.println(activeCount);
  }
}

void scanAnalogPorts() {
  Serial.println(F("\n  ANALOG PORTS"));
  uint8_t activeCount = 0;
  for (uint8_t pin = 0; pin < ANALOG_PINS; pin++) {
    #ifdef ESP32
      int analogPin = pin;
    #elif defined(ESP8266)
      int analogPin = A0;
      if (pin > 0) break;
    #else
      int analogPin = A0 + pin;
      if (analogPin == SDA_PIN || analogPin == SCL_PIN) continue;
    #endif
    
    int value = analogRead(analogPin);
    float voltage = (value * SUPPLY_VOLTAGE) / ADC_RESOLUTION;
    
    if (value > 50 && value < (ADC_RESOLUTION - 100)) {
      Serial.print(F("Pin A"));
      Serial.print(pin);
      Serial.print(F(": ACTIVE "));
      Serial.print(value);
      Serial.print(F(" ["));
      Serial.print(voltage, 2);
      Serial.println(F("V]"));
      activeCount++;
    }
  }
  if (activeCount == 0) {
    Serial.println(F("No active analog ports"));
  } else {
    Serial.print(F("Active ports: "));
    Serial.println(activeCount);
  }
}

void scanI2C() {
  Serial.println(F("\n  I2C SCAN"));
  #ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
  #elif defined(ESP8266)
    Wire.begin(SDA_PIN, SCL_PIN);
  #else
    Wire.begin();
  #endif
  
  Serial.print(F("I2C Bus: SDA="));
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega2560__)
    if (SDA_PIN == A4) Serial.print(F("A4"));
    else Serial.print(SDA_PIN);
    Serial.print(F(", SCL="));
    if (SCL_PIN == A5) Serial.print(F("A5"));
    else Serial.print(SCL_PIN);
  #else
    Serial.print(SDA_PIN);
    Serial.print(F(", SCL="));
    Serial.print(SCL_PIN);
  #endif
  Serial.println();
  
  uint8_t deviceCount = 0;
  for (uint8_t address = 8; address < 120; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("Device: 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.print(F(" ("));
      Serial.print(address);
      Serial.print(F(") - "));
      Serial.println(getDeviceName(address));
      deviceCount++;
    }
  }
  if (deviceCount == 0) {
    Serial.println(F("No I2C devices found"));
    Serial.println(F("Check: connections, pullup resistors (4.7k to 5V)"));
  } else {
    Serial.print(F("Total devices: "));
    Serial.println(deviceCount);
  }
}

const __FlashStringHelper* getDeviceName(uint8_t addr) {
  switch(addr) {
    case 0x20: case 0x21: case 0x22: case 0x23: 
    case 0x24: case 0x25: case 0x26: case 0x27: 
      return F("MCP23017 I/O Expander");
    case 0x3C: case 0x3D: return F("OLED Display SSD1306");
    case 0x40: case 0x41: return F("PCA9685 PWM Driver");
    case 0x48: case 0x49: case 0x4A: case 0x4B: 
      return F("ADS1115 16-bit ADC");
    case 0x50: case 0x51: case 0x52: case 0x57: 
      return F("EEPROM 24LC series");
    case 0x53: return F("ADXL345 Accelerometer");
    case 0x5A: return F("MLX90614 IR Temperature");
    case 0x68: return F("DS1307 RTC / MPU6050 IMU");
    case 0x69: return F("MPU6050 IMU (Alt address)");
    case 0x70: case 0x71: case 0x72: case 0x73: 
    case 0x74: case 0x75: return F("TCA9548A I2C Multiplexer");
    case 0x76: case 0x77: return F("BMP280/BME280 Pressure");
    case 0x1E: return F("HMC5883L Magnetometer");
    case 0x39: return F("TSL2561 Light Sensor");
    case 0x60: return F("SI1145 UV/Light Sensor");
    default: return F("Unknown I2C Device");
  }
}