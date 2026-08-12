# ESP32-C3 Super Mini - Pin Reference

## Board Overview
The ESP32-C3 Super Mini is a compact development board powered by the ESP32-C3 FN4 microcontroller featuring:
- **Processor**: RISC-V single-core @ 160 MHz
- **Flash Memory**: 4MB
- **Connectivity**: Wi-Fi (2.4 GHz 802.11 b/g/n), BLE 5.0
- **Power**: 5V via USB-C, onboard 3.3V regulator
- **Operating Voltage**: 3.3V

## Available GPIO Pins

| GPIO | Function | Capabilities | Notes |
|------|----------|--------------|-------|
| **GPIO0** | General I/O | ADC1, PWM | Analog input capable |
| **GPIO1** | General I/O | ADC1, PWM | Analog input capable |
| **GPIO2** | General I/O | ADC1, Strapping Pin | Boot mode selection, analog capable |
| **GPIO3** | General I/O | ADC1, PWM | Analog input capable |
| **GPIO4** | General I/O | JTAG, ADC1 | Reserved for debugging, analog capable |
| **GPIO5** | General I/O | JTAG | Reserved for debugging |
| **GPIO8** | Status LED | Strapping Pin | **Built-in blue LED (inverted logic)** |
| **GPIO9** | Boot Button | Strapping Pin | **Connected to BOOT button** |
| **GPIO10** | General I/O | PWM | - |
| **GPIO20** | General I/O | - | - |
| **GPIO21** | General I/O | - | - |

## Peripheral Capabilities

### Analog to Digital Converter (ADC)
- **6 channels** of 12-bit SAR ADC
- Available on: GPIO0, GPIO1, GPIO2, GPIO3, GPIO4

### PWM
- **Up to 6 channels**
- Available on: GPIO0, GPIO1, GPIO3, GPIO10

### Communication Interfaces
| Interface | Count | Notes |
|-----------|-------|-------|
| **UART** | 2 | Configurable on GPIO pins |
| **SPI** | 3 | SPI0, SPI1 reserved for flash |
| **I²C** | 1 | Configurable on any GPIO |
| **I²S** | 1 | Full-duplex audio support |

### JTAG Debugging
- Available on GPIO4–GPIO7
- Used for debugging purposes

## Special Pin Notes

⚠️ **Important Pin Behaviors:**

- **GPIO8**: Connected to an **inverted blue status LED**
  - `LOW` = LED ON
  - `HIGH` = LED OFF

- **GPIO9**: Connected to the **BOOT button**
  - Used for entering bootloader mode
  - Can be used as input in your code

- **GPIO2**: Strapping pin affects boot mode
  - Be careful when using at startup

- **GPIO4-GPIO7**: Reserved for JTAG debugging
  - Can be used as GPIO if debugging not needed

## Quick Pin Usage in Arduino

```cpp
// Digital Output
pinMode(GPIO_NUM_10, OUTPUT);
digitalWrite(GPIO_NUM_10, HIGH);

// Digital Input
pinMode(GPIO_NUM_20, INPUT);
int state = digitalRead(GPIO_NUM_20);

// Analog Input (12-bit: 0-4095)
int value = analogRead(GPIO_NUM_0);

// PWM Output
ledcSetup(0, 5000, 8); // channel 0, 5kHz, 8-bit resolution
ledcAttachPin(GPIO_NUM_1, 0);
ledcWrite(0, 128); // 50% duty cycle

// Built-in LED (inverted)
pinMode(8, OUTPUT);
digitalWrite(8, LOW);  // Turn LED ON
digitalWrite(8, HIGH); // Turn LED OFF
```

## Resources
- [Full Tutorial](https://github.com/sidharthmohannair/Tutorial-ESP32-C3-Super-Mini)
- [ESP32-C3 Datasheet](https://www.espressif.com/en/support/documents/technical-documents)
