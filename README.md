# Bioreactor Control System

An Arduino-based bioreactor control system featuring real-time monitoring and control of pH, temperature, stirring speed, and user interface.

## Subsystems

### 1. pH Control Subsystem
- Real-time pH monitoring
- Automated acid/alkali pump control
- PID-based pH adjustment
- Working range: 3-9 pH

### 2. Temperature Control Subsystem
- Thermistor-based temperature sensing
- PID-controlled heating system
- Temperature range: 15-45°C
- Real-time thermal regulation

### 3. Stirring Subsystem
- RPM control and monitoring
- Adjustable stirring speed (0-1500 RPM)
- Encoder-based feedback
- Motor torque monitoring

### 4. User Interface Subsystem
- LCD display for real-time data
- Push-button controls
- Remote web interface via ESP32
- Data logging capabilities

## Hardware Components

- Arduino Mega 2560
- 20x4 LCD Display
- pH sensor (Atlas Scientific)
- DS18B20 temperature sensor
- 12V DC motor with encoder
- Dual peristaltic pumps
- ESP32 (Wi-Fi connectivity)

## Pin Configuration

| Component      | Pin    |
|----------------|--------|
| pH Sensor      | A0     |
| Temperature    | D4     |
| Heater         | D6     |
| Acid Pump      | D9     |
| Base Pump      | D10    |
| Motor          | D11    |
| LCD (I2C)      | A4, A5 |

## Setup Instructions

1. Wire components per pin configuration
2. Install libraries:
    - PID_v1
    - LiquidCrystal_I2C
    - ESP32WebServer
3. Upload firmware
4. Calibrate sensors

## Operation

- Remote control through web dashboard
- Automatic PID control for all parameters
- Data logging to SD card

## Communication

- I2C for local peripherals
- WiFi for remote access
- MQTT for data transmission
- Serial debugging interface

