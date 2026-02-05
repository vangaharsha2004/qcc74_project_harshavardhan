# Real-Time Environmental Monitoring System with RTC and SPI Communication

## Project Overview
This project implements a **real-time environmental monitoring system** using an embedded microcontroller platform. It integrates multiple sensors and communication protocols to collect, process, and transmit environmental data reliably.

The system reads:
- **Temperature & Humidity** from a DHT11 sensor  
- **Air Quality (Gas level)** from an MQ135 sensor via ADC  
- **Time & Date** from a DS1307 RTC module using I²C  

All collected data is packaged and transmitted to another controller/device using **SPI communication**. The system also includes a **watchdog mechanism** to automatically recover the RTC if it gets stuck.

---

## Features
- Real-time clock management using **DS1307 (I²C)**
- Automatic RTC oscillator force-start on boot
- Temperature & humidity monitoring using **DHT11**
- Air quality measurement using **MQ135 (ADC)**
- SPI master communication for data transfer
- Watchdog logic to detect and recover RTC failure
- Bare-metal embedded C implementation

---

## Hardware Components
- QCC74x Series Microcontroller
- DS1307 RTC Module
- DHT11 Temperature & Humidity Sensor
- MQ135 Gas Sensor
- External SPI Slave Device
- Pull-up resistors for I²C lines
- Power supply (3.3V)

---

##  Pin Configuration

### SPI
| Signal | GPIO Pin |
|------|---------|
| CS   | GPIO12  |
| SCK  | GPIO13  |
| MISO | GPIO14  |
| MOSI | GPIO15  |

### I²C (RTC)
| Signal | GPIO Pin |
|------|---------|
| SCL  | GPIO10  |
| SDA  | GPIO11  |

### Sensors
| Sensor  | GPIO |
|-------|------|
| DHT11 | GPIO3 |
| MQ135 | GPIO2 (ADC Channel 2) |

---

## Communication Protocols Used
- **SPI** – Data transmission to external device
- **I²C** – RTC communication
- **ADC** – MQ135 analog gas sensor
- **GPIO** – DHT11 interface

---

## Data Packet Format (SPI)
| Byte | Description |
|----|------------|
| 0  | Start Frame (0xAA) |
| 1  | Temperature |
| 2  | Humidity |
| 3  | Hour |
| 4  | Minute |
| 5  | Second |
| 6  | AQI Voltage MSB |
| 7  | AQI Voltage LSB |

---

## RTC Watchdog Mechanism
- Continuously monitors RTC time
- If time remains `00:00:00` for more than 5 seconds:
  - Automatically restarts the RTC oscillator
  - Prevents system lock due to RTC failure

---

## Sample UART Output


---

## Build & Run Steps
1. Initialize board and peripherals
2. Configure GPIO, SPI, I²C, ADC
3. Force-start RTC oscillator
4. Read sensors periodically
5. Transmit data via SPI every second
6. Display status over UART

---

## Software Tools
- QCC74x SDK
- Embedded C (Bare Metal)
- GCC Toolchain
- Minicom (for UART output)

---

## Applications
- Environmental monitoring systems
- Smart city air quality tracking
- Industrial safety monitoring
- IoT edge devices
- Embedded systems learning projects

---

## Author
**Embedded Systems Project – Fresher Level**  
Designed & implemented using register-level Embedded C.

---

## License
This project is for educational and learning purposes.

