# LoRA-Particle-Node

Low-power Particle Boron node firmware for remote counters that report to a LoRa gateway.

This repository contains the node side of the system. The node sleeps most of the time, wakes on a reporting cadence or a local interrupt, takes measurements, sends a LoRa packet to the gateway, receives an acknowledgement, updates its local configuration from that acknowledgement, and goes back to sleep.

The gateway is not implemented in this repository. This firmware expects a compatible gateway that uses the same packet layout and radio settings.

## Current Release

- Firmware release: `v22.00`
- Particle product version: `22`
- Target Device OS used in this workspace: `6.4.0`
- System mode: `MANUAL`
- Normal operating model: LoRa-only wake, no cloud dependency during normal reporting
- Time authority model: boot RTC restore is provisional; only a valid gateway ACK timestamp updates system time, the AB1805 RTC, and `Energy24h` timing

## What This Repo Does

This firmware runs on a Boron-class node and does the following:

1. Stores node identity and runtime settings in FRAM.
2. Uses an AB1805 RTC/watchdog for timing and wake support.
3. Samples enclosure temperature and battery telemetry.
4. Powers and monitors an attached field sensor.
5. Sends visitor/event counts and battery status to a LoRa gateway.
6. Accepts gateway acknowledgements that can set time, report interval, open/closed hours, sensor type, and alert behavior.
7. Sleeps aggressively between report windows to conserve power.

In code, the main pieces are:

- `src/LoRA-Particle-Node.cpp`: main node state machine
- `src/LoRA_Functions.cpp`: LoRa radio setup, send, receive, and ack handling
- `src/MyPersistentData.cpp`: FRAM-backed persistent identity and schedule storage
- `src/take_measurements.cpp`: temperature, battery, and count telemetry
- `src/power_management.cpp`: Boron battery and PMIC policy

## Radio and Protocol Expectations

The node is hard-coded to the following LoRa network assumptions:

- Gateway address: `0`
- Assigned node addresses: `1` through `10`
- Unconfigured node address: `11` or greater triggers join behavior
- RF frequency: `926.84 MHz`
- Modem config: `Bw125Cr45Sf2048`
- Manager timeout: `2000 ms`
- TX power: `23 dBm`

The node sends two primary message types:

1. `JOIN_REQ`
2. `DATA_RPT`

The gateway must respond with:

1. `JOIN_ACK`
2. `DATA_ACK`

Packet field layout is documented directly in `src/LoRA_Functions.h` and is what the node expects on the air.

## Hardware Used By The Node

This firmware is written for a Boron-based carrier with these key connections:

- `D5`: RFM95 chip select
- `D6`: RFM95 reset
- `D2`: RFM95 interrupt
- `D0` / `D1`: I2C for FRAM and AB1805
- `D4`: user button
- `D7`: on-board blue LED
- `D8`: AB1805 wake/watchdog output
- `A4`: TMP36 enclosure temperature sensor
- `A1`: sensor interrupt input
- `A2`: sensor power control
- `A3`: sensor LED / secondary sensor control

The code currently supports two sensor modes:

- `0`: pressure sensor
- `1`: PIR sensor

## Dependencies

The release build uses the vendored library copies under `lib/`:

- `AB1805_RK`
- `MB85RC256V-FRAM-RK`
- `RF9X-RK`
- `CryptoLW-RK`
- `StorageHelperRK`

## Configuration Knobs

Compile-time configuration lives in `src/config.h`.

Current relevant options:

- `POWER_MANAGER_INPUT_PROFILE`
- `VERBOSE_SYSTEM_LOGS`
- `FIELD_DEBUG_BUILD`

Defaults are set for production-friendly operation:

- `POWER_MANAGER_INPUT_PROFILE_USB_BENCH`
- `VERBOSE_SYSTEM_LOGS=0`
- `FIELD_DEBUG_BUILD=0`

## How The Node Starts

On boot, the node does this:

1. Initializes GPIO.
2. Loads persistent `sysStatus` and `current` state from FRAM.
3. Starts power management.
4. Takes an initial measurement.
5. Initializes the LoRa radio.
6. Decides whether it already has a valid node identity and valid time.

If either of these is true, the node initiates a join flow:

- `nodeNumber > 10`
- local time is invalid

The default FRAM initialization sets:

- `nodeNumber = 11`
- `frequencyMinutes = 60`
- `alertCodeNode = 1`
- `openHours = true`

That means a brand-new node naturally comes up in join-needed state.

## How To Set Up A Node

### 1. Prepare the hardware

1. Assemble a Boron-based node with the pinout listed above.
2. Connect the RFM95 radio module to `D5/D6/D2` plus SPI.
3. Connect the FRAM and AB1805 to the I2C bus on `D0/D1`.
4. Connect the TMP36 to `A4`.
5. Connect the field sensor interrupt to `A1` and sensor power control to `A2`.

### 2. Build the firmware

Use Particle Workbench or your local Particle build flow to compile the project in this repository.

This repo is structured as a standard Particle user application with library dependencies declared in `project.properties`.

### 3. Flash the node

Flash the firmware to the Boron. After flashing, the node should boot, initialize FRAM, initialize the LoRa radio, and log its startup state.

### 4. Verify the node is in a sane default state

For a new node, expected behavior is:

1. The node reports invalid or unconfigured identity/time at startup.
2. `alertCodeNode` is set so the node enters join behavior.
3. The node sleeps and wakes on its normal cadence unless the user button or sensor interrupt changes the cycle.

### 5. Optional maintenance mode

If the user button is held during startup, the node temporarily connects to Particle Cloud for update/maintenance, stays online briefly, then disconnects and resets back into normal offline node operation.

That mode is intended for maintenance only, not for normal field reporting.

## How The Node Connects To The Gateway

This node does not connect to the gateway the way a Wi-Fi or cellular client connects to a server. It joins the LoRa network by sending a `JOIN_REQ` to gateway address `0` and waiting for a `JOIN_ACK`.

### Gateway requirements

A compatible gateway must:

1. Be listening on the same RF frequency and modem settings.
2. Use gateway address `0`.
3. Understand the join and data packet layouts defined in `src/LoRA_Functions.h`.
4. Reply to join requests with a valid `JOIN_ACK`.
5. Assign a node number in the range `1` to `10`.
6. Provide valid non-zero time and report interval data in acknowledgements.

If an ACK timestamp is zero or outside the accepted sane epoch range, the node ignores it, does not update time or RTC state, and continues with the existing retry/fallback scheduling behavior.

### Node join sequence

The node-side join flow is:

1. Node wakes with `nodeNumber > 10` or invalid time.
2. Node sends `JOIN_REQ` to gateway address `0`.
3. Gateway returns `JOIN_ACK`.
4. Node stores:
   - `nodeNumber`
   - `sensorType`
   - `alertCodeNode`
   - current time, only when the ACK timestamp is valid
   - report interval
5. Node switches its mesh manager address to the assigned node number.
6. Node enters normal report mode.

Expected success log:

- `Join request sent to gateway successfully ...`
- `Node X Join request acknowledged and sensor set to Y`

### Normal reporting sequence

After joining, each report cycle is:

1. Wake from sleep.
2. Take measurements.
3. Send `DATA_RPT`.
4. Listen for `DATA_ACK`.
5. Update time, frequency, alert code, and `openHours` from the gateway ack.
   Invalid ACK timestamps are ignored and do not advance local time or long-window energy timing.
6. Save state to FRAM.
7. Return to sleep.

The gateway acknowledgment can also:

- trigger an alert action
- update sensor type
- mark the park/site closed
- change the next reporting interval

When the gateway reports closed hours, the node resets counts and uses the closed-hours sleep schedule provided by the gateway.

## Expected Field Behavior

In normal field operation:

1. The cellular modem is not required for reporting.
2. The node wakes, transmits over LoRa, receives a gateway ack, and sleeps.
3. The node stores identity and schedule state in FRAM so it survives reset and power loss.
4. If the gateway cannot be reached, the node retries, then falls back to sleep and later recovery behavior.

## Operational Notes

- Success-rate logging is clamped to `0.00` to `100.00`.
- Production logging is centralized through `Log.*`.
- `Energy24h` provides a compact 24-hour energy-balance summary for soak validation and field drift analysis.
- Verbose field diagnostics should be enabled only through `VERBOSE_SYSTEM_LOGS` or `FIELD_DEBUG_BUILD`.
- The node will try to rejoin if its identity is invalid or if time is not valid.

## Quick Setup Checklist

1. Wire the Boron, RFM95, AB1805, FRAM, TMP36, and sensor according to `src/device_pinout.cpp`.
2. Build and flash this repo to the node.
3. Ensure the gateway is running separately with address `0`, `926.84 MHz`, and `Bw125Cr45Sf2048`.
4. Power the node and watch for startup logs.
5. Confirm the node sends a join request and receives a join acknowledgement.
6. Confirm the node gets a valid assigned node number from `1` to `10`.
7. Confirm a later `DATA_RPT` / `DATA_ACK` exchange succeeds.

## Files To Read First

- `src/LoRA-Particle-Node.cpp`
- `src/LoRA_Functions.cpp`
- `src/LoRA_Functions.h`
- `src/MyPersistentData.h`
- `src/device_pinout.cpp`
- `src/config.h`

## Changelog

See `CHANGELOG.md` for release history.

## Release Notes

### v22.00

- Added `Energy24h` long-window energy trend instrumentation for outdoor solar soak validation and field stability analysis.
- Improved PMIC fault/reset visibility, charging recovery summaries, and compact source-selection logging.
- Reduced serial truncation with connect, sleep, wake, and PMIC log cleanup.
- No intended occupancy, sleep-policy, watchdog, or connection-budget behavior changes.
- Successful outdoor soak validation prior to release packaging.

### v20

- Hardened node success-rate reporting so logged delivery percentages are zero-safe, bounded to `0.00`-`100.00`, and no longer exceed 100 after resets or low-sample startup windows.
- Normalized runtime firmware logging to the centralized `Log.*` handler in app code and active runtime support libraries, reducing partial-line output that could interleave with system logs on Boron-class devices.
- Added production-safe debug gates for field-only diagnostics using `FIELD_DEBUG_BUILD` and the existing `VERBOSE_SYSTEM_LOGS` controls.

### v18

- Added node resiliency hardening for LoRa retry, direct-to-gateway post-ACK sleep, and closed-hours scheduling behavior.
- Preserved node identity and last known schedule across firmware updates by repairing invalid persisted frequency values in place instead of reinitializing sysStatus.
- Added reusable node PowerManager support for normalized battery telemetry, conservative fallback SOC handling, and Boron PMIC input profile selection for UsbBench, Solar35W, and Auto modes.
- Reduced PMIC log noise by caching the last applied profile and only reapplying SystemPowerConfiguration on cold boot or profile changes.

