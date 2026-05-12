# DVS-Wireless-DIY-DJ-System

A low-cost DIY wireless DVS system inspired by modern wireless vinyl control solutions.

This project is **NOT a replacement for Phase DJ** and does not aim to achieve the same industrial precision, latency, reliability, or calibration quality. Instead, it is an accessible and affordable alternative for makers, DJs, students, and developers who want to experiment with wireless DVS technology using ESP32 boards.

If you want the same level of refinement, precision, robustness, and professional reliability as Phase DJ, the best option is still purchasing an original commercial system.

This project focuses on:

* simplicity
* low cost
* DIY accessibility
* experimentation
* open development
* learning

---

# Features

* Wireless DVS control
* ESP-NOW ultra low latency communication
* Real-time timecode generation
* Stereo 16-bit WAV timecode playback
* Reverse scratching support
* Automatic RPM calibration
* Pitch tracking
* Real vinyl movement detection
* PCM5102 I2S DAC support
* ESP32-based architecture
* Fully DIY and customizable

---

# How It Works

The system is divided into two devices:

## 1. ESP32-C3 Transmitter (Mounted on Vinyl)

The transmitter:

* reads platter movement using an MPU6050 gyroscope
* detects platter speed and direction
* calculates RPM in real time
* sends movement data wirelessly using ESP-NOW

The ESP32-C3 is mounted directly on top of the vinyl.

---

## 2. ESP32-S3 Receiver (Audio Generator)

The receiver:

* receives RPM data from the ESP32-C3
* controls playback speed of a stereo timecode WAV
* generates real-time DVS audio through I2S
* outputs audio to a PCM5102 DAC
* supports reverse playback and scratching

The generated timecode signal can then be connected to DVS software through a DJ mixer or audio interface.

---

# Important Notes

This is a DIY experimental project.

Compared to professional systems like Phase DJ:

* latency may vary
* precision may vary
* gyro drift may occur
* tracking stability depends on calibration and hardware quality
* scratch performance depends on tuning

Professional systems use:

* proprietary firmware
* industrial calibration
* custom RF protocols
* advanced filtering
* specialized hardware

This project uses affordable off-the-shelf components and community-developed code.

---

# Hardware Required

## Receiver

* ESP32-S3
* PCM5102 DAC

## Transmitter

* ESP32-C3
* MPU6050 gyroscope

## Other

* Turntable
* RCA cables
* USB power supply
* Vinyl record

---

# Wiring

## PCM5102 → ESP32-S3

```text
BCK   -> GPIO 2
LRCK  -> GPIO 1
DATA  -> GPIO 42
```

---

## MPU6050 → ESP32-C3

```text
SDA -> GPIO 8
SCL -> GPIO 9
```

---

# Arduino IDE Setup

## Install ESP32 Board Package

Arduino IDE → Preferences:

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then install:

* ESP32 by Espressif Systems

---

# Required Libraries

The project uses libraries already included in the ESP32 package:

```text
WiFi.h
esp_now.h
Wire.h
driver/i2s.h
Adafruit_NeoPixel.h
```

No additional external libraries are required.

---

# Recommended Boards

Best setup:

* ESP32-C3 → transmitter
* ESP32-S3 → receiver/audio

Mini versions also work well:

* XIAO ESP32-S3
* ESP32-S3 Zero
* WeAct S3 Mini

---

# Audio Configuration

Current implementation:

* 44.1kHz
* 16-bit stereo WAV
* Real-time interpolation
* Low latency DMA buffers
* Reverse playback support

---

# ESP32 MAC Address Configuration

Before using the system, you must discover the MAC address of the ESP32-S3 receiver board.

The ESP32-C3 transmitter uses this MAC address to send RPM data through ESP-NOW.

You must replace the receiver MAC address inside the ESP32-C3 transmitter code.


---

# Project Goals

* Create an affordable wireless DVS system
* Learn about digital vinyl systems
* Experiment with ESP32 real-time audio
* Explore wireless scratch technology
* Provide a platform for community improvements


---

# Disclaimer

This project is an independent DIY research project.

It is not affiliated with, endorsed by, or associated with:

* Phase DJ
* Serato
* Pioneer DJ
* Native Instruments
* any related company

All trademarks belong to their respective owners.

---

# License

CC BY-NC-SA 4.0

You are free to:

* study
* modify
* share

Under the following conditions:

* attribution required
* non-commercial use only
* derivatives must use the same license

Commercial use or resale is prohibited without explicit permission from the author.
