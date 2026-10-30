# BMS Temperature Monitoring Module

## Overview

An STM32-based distributed battery temperature monitoring module designed to monitor 13 NTC thermistors per board and communicate temperature telemetry and fault information over CAN bus.

The firmware uses ADC with DMA for continuous multi-channel temperature acquisition, Beta-equation-based NTC temperature conversion, fault detection with configurable thresholds, and CAN-based communication with a central BMS controller.

The architecture supports multiple monitoring boards, each identified by a unique CAN board ID.

## Hardware

- **Microcontroller:** STM32F407VET6
- **Temperature Sensors:** 10kΩ NTC thermistors (MF55)
- **Reference Resistors:** 10kΩ
- **ADC:** 12-bit, 13-channel scan conversion
- **ADC Acquisition:** DMA with continuous conversion
- **Communication:** CAN 2.0
- **CAN Transceiver:** TJA1051
- **CAN Bitrate:** 500 kbps

## Features

- 13-channel NTC temperature acquisition per board.
- Continuous ADC sampling using DMA.
- Interrupt-driven acquisition through ADC conversion-complete callback.
- NTC temperature calculation using the Beta equation.
- Configurable overtemperature thresholds.
- Sensor fault detection for abnormal ADC readings.
- Per-channel fault latching and debounce logic.
- CAN transmission of temperature telemetry.
- Dedicated CAN fault frames.
- Support for multiple monitoring boards through configurable board IDs.

## System Architecture

```text
       13 × NTC Thermistors
                |
                v
       Voltage Divider Network
                |
                v
          STM32 ADC1
                |
                v
          DMA Acquisition
                |
                v
       ADC Conversion Callback
                |
         +------+------+
         |             |
         v             v
    Fault Detection  ADC Snapshot
         |             |
         v             v
    Fault Latching  Temperature
         |           Conversion
         v             |
    CAN Fault Frame    v
                  Fixed-Point
                  Temperature
                       |
                       v
                  CAN Telemetry
                       |
                       v
                Central BMS ECU
```

## Temperature Measurement

Each NTC thermistor is connected through a voltage divider consisting of:

- 10kΩ NTC connected to 3.3V.
- 10kΩ fixed reference resistor connected to GND.
- ADC connected to the divider midpoint.

The firmware calculates NTC resistance using:

\[
R_{NTC}=R_{FIXED}\frac{V_{REF}-V_{OUT}}{V_{OUT}}
\]

Temperature is calculated using the Beta equation:

\[
\frac{1}{T}=\frac{1}{T_{25}}+
\frac{1}{B}\ln\left(\frac{R_{NTC}}{R_{25}}\right)
\]

Where:

- \(R_{25}=10k\Omega\)
- \(B=3950K\)
- \(T_{25}=298.15K\)

The resulting temperature is converted into fixed-point format (°C × 100) before CAN transmission.

## Fault Detection

The firmware implements three fault categories:

| Fault | Detection condition |
|---|---|
| Overtemperature | ADC ≥ 3155 |
| Sensor short | ADC ≤ 50 |
| Sensor open | ADC ≥ 4045 |

Overtemperature detection uses consecutive-sample debounce logic, while sensor faults use a separately configurable debounce count.

Detected faults are latched independently for each channel and reported through CAN fault frames.

The overtemperature clear threshold is configured separately to provide hysteresis.

## CAN Communication

The firmware uses standard 11-bit CAN identifiers.

### Temperature Telemetry

Temperature values are transmitted as signed 16-bit integers in big-endian format, representing temperature in °C × 100.

Each board transmits four telemetry frames:

| Frame | CAN ID | Channels | Payload |
|---|---|---|---|
| 0 | 0x100 + (BOARD_ID × 0x10) | 0–3 | 8 bytes |
| 1 | 0x101 + (BOARD_ID × 0x10) | 4–7 | 8 bytes |
| 2 | 0x102 + (BOARD_ID × 0x10) | 8–11 | 8 bytes |
| 3 | 0x103 + (BOARD_ID × 0x10) | 12 | 2 bytes |

Telemetry transmission is scheduled every 100 ms.

### Fault Frames

Fault frames use CAN identifiers from `0x000` to `0x005`, assigned according to the board ID.

| Byte | Description |
|---|---|
| Byte 0 | Faulted NTC channel |
| Byte 1 | Fault type |
| Byte 2–7 | Reserved |

Fault type encoding:

| Value | Fault |
|---|---|
| 0 | No fault |
| 1 | Overtemperature |
| 2 | Sensor short |
| 3 | Sensor open |

## Multi-Board Configuration

Each monitoring board must have a unique `BOARD_ID`.

```c
#define NTC_CHANNELS  13
#define BOARD_ID      0
```

Supported board IDs: `0–5`.

Each board independently transmits its temperature telemetry and fault information using its assigned CAN identifiers.

## Firmware Architecture

| Function | Responsibility |
|---|---|
| `MX_ADC1_Init()` | Configures 13-channel ADC scan conversion |
| `MX_DMA_Init()` | Initializes DMA acquisition |
| `HAL_ADC_ConvCpltCallback()` | Processes ADC samples and detects faults |
| `ADC_ToTemperature()` | Converts ADC readings into temperature |
| `Temp_ToFixedPoint()` | Converts temperature into fixed-point representation |
| `Send_Telemetry_Frames()` | Packs and transmits temperature readings |
| `CAN_SendFault()` | Transmits channel-specific fault information |
| `Fault_Reset()` | Clears the latched fault state for a channel |

## Future Extensions

- Integration with the central BMS controller.
- CAN-based fault acknowledgement and reset.
- CAN bus error and bus-off recovery.
- Watchdog integration.
- Improved transmission retry handling.
- Temperature-based battery protection and thermal management.

## License

This project is intended for battery management system development and experimental embedded firmware applications.
