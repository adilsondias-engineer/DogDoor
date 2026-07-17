# DogDoor - Automated Dog Door Controller

An ESP32-based automated dog door system that uses motion sensors, ultrasonic distance detection, and a stepper motor to automatically open and close a dog door. Built on ESP-IDF and FreeRTOS, running four concurrent, core-pinned real-time tasks coordinating over a shared door-state machine.

This is a full custom-designed automated dog door (ESP32/FreeRTOS) — self-designed aluminium frame, CO2-laser-cut acrylic panel, lead-screw linear actuation, custom PCB, and multi-task RTOS firmware with WiFi/MQTT/HTTP. In production 3+ years.

## Author

**Adilson Dias** ([@adilsondias-engineer](https://github.com/adilsondias-engineer))

## Features

- **Automated Door Control**: Automatically opens when motion is detected and closes when no motion is present
- **Safety Features**:
  - Ultrasonic sensor to detect obstacles
  - Limit switches for door position detection
  - Motion detection prevents closing if movement detected, and reverses direction (closing → opening) if movement is detected mid-close
- **Remote Control**:
  - HTTP web interface for manual control
  - MQTT support for home automation integration
- **Network Connectivity**: WiFi and Ethernet support
- **OTA Updates**: Over-the-air firmware updates
- **Real-time Monitoring**: Web dashboard showing door status, sensor readings, and system uptime

## Software Architecture

The controller runs four concurrent FreeRTOS tasks, pinned to a dedicated core for deterministic timing, coordinating over a shared door-state machine (`OPEN → OPENING → CLOSING → FORCECLOSING → CLOSED`):

```
┌─────────────────────┐     ┌────────────────────────┐     ┌────────────────────┐
│  Check_Sensor_Task  │     │ Check_Door_Opened_Task │     │Check_Door_Closed_  │
│  (PIR in/out)       │     │  (limit switch)        │     │Task (limit switch) │
└──────────┬──────────┘     └───────────┬────────────┘     └─────────┬──────────┘
           │                            │                            │
           └─────────────┬──────────────┴──────────────┬─────────────┘
                         ▼                              ▼
                 ┌──────────────────────────────────────────────┐
                 │            currentDoorStatus                 │
                 │  (shared state: OPEN/OPENING/CLOSING/...)    │
                 └───────────────────┬──────────────────────────┘
                                     ▼
                        ┌──────────────────────────┐
                        │     Handle_Door_Task     │
                        │  (stepper motor control, │
                        │   accel/decel profiles)  │
                        └──────────────────────────┘

              WiFi ──▶ HTTP server / MQTT client / NTP sync / OTA
```

- **Check_Sensor_Task** — polls indoor/outdoor PIR sensors, determines trigger source and sets `movimentDetected`
- **Check_Door_Opened_Task / Check_Door_Closed_Task** — continuously poll the open/closed limit switches
- **Handle_Door_Task** — owns the stepper motor: sets speed/accel/decel profile, drives open/close motion, and monitors limit-switch + motion-sensor flags mid-motion to trigger safety stops or direction reversal

## Hardware

### Physical Door Design

The automated dog door features a custom mechanical design built with the following components:

1. **Aluminium Frame**: Structural framework providing rigidity and mounting points
2. **NEMA17 Stepper Motor**: High-torque bipolar stepper motor for precise door control
3. **5mm Thick Acrylic**: Transparent door panel material
4. **T20 Lead Screw Optical Axis**: Linear motion mechanism for smooth vertical door movement
5. **Custom PCB**: Electronics integration board (see below)
6. **Proximity Sensors**: Motion detection for automatic door operation (indoor/outdoor)
7. **Door Status Sensors**: Limit switches for open/closed position detection

### Custom PCB
This project features a **custom PCB designed on EasyEDA by Adilson Dias**, with all components professionally soldered by Adilson Dias. The PCB integrates all necessary connections for the ESP32, stepper motor driver, sensors, and power management in a compact, reliable design.

### Electronic Components

- **Microcontroller**: ESP32 development board
- **Motor System**:
  - NEMA17 bipolar stepper motor
  - STEP/DIR compatible stepper driver
  - T20 lead screw (20mm pitch) for linear motion
- **Sensors**:
  - 2x Proximity sensors (PIR motion sensors - indoor and outdoor)
  - HC-SR04 or compatible ultrasonic distance sensor (obstacle detection)
  - 2x Door status sensors (limit switches for open/closed position)
- **Network**: WiFi or Ethernet connection
- **Power Supply**: Appropriate voltage for stepper motor and ESP32

### Pin Configuration

| Component | GPIO Pin |
|-----------|----------|
| Stepper Motor STEP | GPIO 13 |
| Stepper Motor DIR | GPIO 14 |
| Stepper Motor ENABLE | GPIO 15 |
| Indoor Motion Sensor | GPIO 18 |
| Outdoor Motion Sensor | GPIO 19 |
| Closed Limit Switch | GPIO 32 |
| Open Limit Switch | GPIO 33 |
| Ultrasonic Trigger | GPIO 5 |
| Ultrasonic Echo | GPIO 23 |

## Software Dependencies

### ESP-IDF Framework
This project is built using the Espressif IoT Development Framework (ESP-IDF).

### Required Components
- **espressif/mdns**: Multicast DNS service
- **ESP-IDF Components**:
  - FreeRTOS
  - ESP WiFi/Ethernet
  - ESP HTTP Server/Client
  - ESP MQTT Client
  - ESP HTTPS OTA
  - NVS (Non-Volatile Storage)

## Building and Flashing

### Prerequisites
- ESP-IDF v5.0 or later
- Python 3.7+

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/adilsondias-engineer/DogDoor.git
cd DogDoor

# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Configure the project
idf.py menuconfig

# Build the project
idf.py build

# Flash to device
idf.py -p <PORT> flash monitor
```

### Configuration

Use `idf.py menuconfig` to configure:
- WiFi SSID and password (Component config → Example Connection Configuration)
- MQTT broker settings
- Network interface (WiFi/Ethernet)

## Usage

### Web Interface
After connecting to your network, access the web interface at:
```
http://<device-ip>/
```

Available endpoints:
- `/` - System status dashboard
- `/openDoor` - Manually open door
- `/closeDoor` - Manually close door

### MQTT Integration
The device publishes status updates to the configured MQTT broker and can receive commands for home automation integration.

## Libraries and Third-Party Code

### DendoStepper Library
- **Author**: Denis Voltmann
- **Copyright**: Copyright (C) 2022 Denis Voltmann
- **License**: GNU General Public License v3.0 or later
- **Description**: ESP-IDF library for bipolar stepper motor drivers with STEP/DIR interface
- **Files**: `main/DendoStepper.cpp`, `main/DendoStepper.h`

### Ultrasonic Sensor Library
- **Author**: Ruslan V. Uss
- **Copyright**: Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
- **License**: BSD 3-Clause License
- **Description**: ESP-IDF driver for ultrasonic range meters (HC-SR04, HY-SRF05, etc.)
- **Source**: Ported from esp-open-rtos
- **Files**: `components/ultrasonic/ultrasonic.c`, `components/ultrasonic/ultrasonic.h`

### Protocol Common Component
- **Source**: ESP-IDF Examples
- **Copyright**: Espressif Systems
- **License**: Apache License 2.0
- **Description**: Common connection utilities for ESP-IDF examples (WiFi, Ethernet, PPP)
- **Files**: `components/protocol_common/*`

## License

The project uses components under different licenses:

- **DendoStepper**: GPL-3.0 or later
- **Ultrasonic Sensor Library**: BSD 3-Clause License
- **ESP-IDF Components**: Apache License 2.0
- **Custom Application Code**: Apache License 2.0

Please refer to individual source files for specific copyright and license information.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Safety Notice

This automated door system is designed for pet access. Always ensure:
- The door mechanism is safe and won't injure pets
- Motion sensors are properly calibrated
- Emergency manual override is available
- Regular maintenance and testing is performed

## Acknowledgments

- **Adilson Dias** for the custom PCB design and hardware assembly
- Denis Voltmann for the DendoStepper library
- Ruslan V. Uss for the ultrasonic sensor driver
- Espressif Systems for ESP-IDF framework and examples
- The ESP32 community for continued support and contributions

## Support

For issues, questions, or contributions, please [open an issue](https://github.com/adilsondias-engineer/DogDoor/issues) or contact the maintainer.

---

**Project Status**: Active Development