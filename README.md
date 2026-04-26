# LoRA-Particle-Node

A low-power Particle LoRa node firmware for Boron-class remote counters and related Particle devices.

This code is for a low-power LoRa node that communicates with a gateway. The node wakes on its reporting cadence, initiates communication, sends measurements and counts, and receives acknowledgement data that can update clock and runtime configuration.

In this system, every communication is initiated by the node. The gateway receives the transmission and replies with an acknowledgement. That acknowledgement can set the node clock, send configuration updates, and define the next reporting interval.

## Current Release

- Firmware release: `v14.00`
- Particle product version: `14`
- Target Device OS in the current workspace: `6.3.5`

## v14 Highlights

- Added a reusable, time-agnostic power management module in `src/power_management.h` and `src/power_management.cpp`.
- Centralized charging safety policy and PMIC recovery so application code no longer configures power directly.
- Added low-battery behavior that reduces listening time and transmit retries to protect battery life.
- Hardened LoRa radio initialization and receive buffer handling for better field reliability.
- Improved battery-state reporting so logged and transmitted charge state reflects post-policy charger state.

## Power Management Module

The `power_management` module is designed to be reusable across LoRa, cellular, and Wi-Fi Particle firmware.

- Input is provided as a raw `PowerObservation` snapshot.
- Charging expectation is derived from input power presence, temperature safety, and battery state of charge.
- Charging anomalies are tracked using event-based counters only; the module does not depend on RTC time, time zones, or daily windows.
- Remediation is a bounded charger toggle on supported PMIC platforms.
- Unsupported PMIC platforms degrade safely without attempting unsupported charger configuration.

The public API is documented in `src/power_management.h`, including a Doxygen usage example.

## Files of Interest

- `src/LoRA-Particle-Node.cpp`: Main node state machine and power-management call site.
- `src/LoRA_Functions.cpp`: Radio initialization, send/receive, and message parsing.
- `src/take_measurements.cpp`: Sensor and battery telemetry collection.
- `src/power_management.h`: Reusable public power-management API.
- `src/power_management.cpp`: Power-management implementation and remediation logic.

## Changelog

See `CHANGELOG.md` for release history.

