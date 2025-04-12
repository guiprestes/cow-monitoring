# Cattle Monitoring Project with ESP32 and LoRa

This project implements a cattle monitoring solution using an ESP32, accelerometer and temperature sensors, and LoRa communication to send the data to a remote gateway. The project is developed using PlatformIO.

## Features

- Monitors cattle activity using an accelerometer sensor.
- Tracks cattle body temperature to assess animal well-being.
- Sends data via LoRa to a remote gateway for further analysis.
- Uses ESP32 microcontroller to integrate sensors and LoRa communication.

## Components Used

- **ESP32**: Main microcontroller responsible for reading sensor data and sending it to the gateway via LoRa.
- **Accelerometer (e.g. ADXL345)**: Used to monitor cattle movement, identifying patterns such as rumination and resting.
- **Temperature Sensor (e.g. DS18B20)**: Measures body temperature to detect potential health issues.
- **LoRa Module (e.g. SX1278)**: Long-range wireless communication module for sending data to the gateway.
- **LoRa Gateway**: Receives and forwards the data sent by the devices in the field.

## Requirements

### Hardware
- ESP32 (e.g. Heltec Wireless Stick)
- Accelerometer Sensor (ADXL345 or similar)
- Temperature Sensor (DS18B20 or similar)
- LoRa Module (SX1278 or similar)
- LoRa Antenna

### Software
- [PlatformIO](https://platformio.org/) with ESP32 support
- Accelerometer library (e.g. [Adafruit ADXL345](https://github.com/adafruit/Adafruit_ADXL345))
- Temperature sensor libraries (e.g. [OneWire](https://github.com/PaulStoffregen/OneWire) and [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library))
- LoRa library (e.g. [LoRa by Sandeep Mistry](https://github.com/sandeepmistry/arduino-LoRa))

## Installation

1. **Clone this repository**:
   ```bash
   git clone https://github.com/your-username/your-repository.git
   cd your-repository
