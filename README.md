# LoRA-Particle-Node

Low-power Particle Boron node firmware for remote counters that report to a LoRa gateway.

This repository contains the node side of the system. The node sleeps most of the time, wakes on a reporting cadence or a local interrupt, takes measurements, sends a LoRa packet to the gateway, receives an acknowledgement, updates its local configuration from that acknowledgement, and goes back to sleep.

The gateway is not implemented in this repository. This firmware expects a compatible gateway that uses the same packet layout and radio settings.

## Current Release

- Firmware release: `v25.0.1-production`
- Particle product version: `25`
- Target Device OS used in this workspace: `6.4.0`
- System mode: `MANUAL`
- Normal operating model: LoRa-only wake, no cloud dependency during normal reporting
- Time authority model: boot RTC restore is provisional; only a valid gateway ACK timestamp updates system time, the AB1805 RTC, and `Energy24h` timing
- Production status: bench validated, extended soak completed, overnight scheduling regression fixed, Raleigh validation complete, ready for NC State Parks deployment

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
5. Decode `openHours` before interpreting the DATA_ACK schedule field.
   - During open hours, bytes 6-7 are treated as the persistent reporting cadence.
   - During closed hours, bytes 6-7 are treated as a one-shot sleep interval for the next overnight wake only.
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

## Production Stabilization Summary

The v25 production stabilization work focused on making scheduling, recovery, power-source detection, and field diagnostics deterministic enough for deployment.

### v25.0.0

- **ACK cadence guard**: ACK schedule values are persisted only when they are valid 60-480 minute reporting cadences in 60-minute increments.
- **Transient schedule rejection**: Gateway one-off timing hints such as 56, 59, 30, 17, and 12 minutes no longer overwrite persistent cadence.
- **Boot-time cadence repair**: Invalid persisted `frequencyMinutes` values are repaired to the default 60-minute cadence during boot.
- **Lightweight discovery recovery**: Nodes with invalid time, no successful ACK, or stale ACK history use a conservative short discovery sleep instead of immediately escalating to heavier recovery behavior.
- **Boron USB source override**: USB-powered Boron bench devices are classified correctly to avoid power-source misdiagnosis during validation.
- **Logging improvements**: Scheduling, ACK, sleep calculation, power, and transaction diagnostics were tightened for soak testing and field triage.

### v25.0.1

v25.0.1 fixed the overnight scheduling regression found after the v25.0.0 cadence hardening.

Root cause: DATA_ACK bytes 6-7 have dual semantics. During open hours they carry the persistent reporting cadence. During closed hours they carry the one-shot sleep interval until the next overnight wake. Cadence validation was happening before DATA_ACK `openHours` context was applied, so closed-hours values such as 479 minutes were rejected as invalid cadences, while 480 minutes could incorrectly overwrite the persistent reporting cadence.

Architecture decision: this was fixed on the Node only. There were no gateway protocol changes, FRAM layout changes, NodeDB changes, or wire protocol changes. Preserving ACK v1 compatibility was more important than introducing a protocol revision during production stabilization.

Implementation summary:

- DATA_ACK now decodes `openHours` before interpreting bytes 6-7.
- Closed-hours intervals are stored in the RAM-only `gatewayOneShotSleepMinutes` pending sleep variable.
- Persistent cadence is never modified during closed-hours ACK handling.
- Open-hours ACKs clear pending overnight sleep state before normal cadence handling resumes.
- JOIN_ACK behavior remains unchanged and continues to use bytes 6-7 as the persistent cadence field.

Validation summary:

- Overnight bench test passed.
- Extended overnight soak completed.
- Raleigh validation completed.
- Correct overnight closed-hours sleep was restored.
- Normal morning reporting cadence resumed after open-hours ACK.

## Changelog

See `CHANGELOG.md` for release history.

## Release Notes

### Current Production

- Gateway: `v27.0.0-production`
- Node: `v25.0.1-production`
- Status: bench validated, extended soak completed, overnight regression fixed, Raleigh validation complete, ready for NC State Parks deployment

### v25.0.1

- **Overnight schedule fix**: DATA_ACK schedule bytes are interpreted after `openHours` is decoded, preventing closed-hours sleep intervals from being rejected or persisted as cadence.
- **RAM-only overnight state**: Closed-hours one-shot sleep is held only until the next sleep calculation and does not change FRAM-backed cadence.
- **Protocol compatibility preserved**: No gateway, NodeDB, FRAM, or wire-format changes were required.

### v25.0.0

- **Lightweight discovery recovery mode**: Automatically recovers from stale or missing gateway ACKs without requiring full join sequence
- **ACK cadence persistence guard**: Only accepts valid 60–480 minute cadence values in 60-minute increments; rejects transient schedule hints (56, 59, 30, 17, 12)
- **Boot-time invalid frequency repair**: Automatically repairs invalid persisted frequencyMinutes to default 60 on boot
- **Boron USB source override**: USB-powered Borons correctly identified, preventing misclassification as bench power
- **Corrected ACK schedule diagnostics**: NextBoundaryUtc calculations now use persisted cadence, preserving SleepCalc alignment after transient gateway hints

For prior release history, see `CHANGELOG.md`.

## Future Enhancements

These items were intentionally deferred. They are not required for the current production deployment.

### 1. Multi-node collision reduction

Proposal: delay the first transmission after wake by `nodeNumber x 2 seconds` so nodes that wake together do not all transmit at once. This should reduce simultaneous LoRa collisions in larger deployments without changing the reporting cadence.

### 2. Gateway visibility of Node firmware

Future gateway work should make Node firmware version visible for fleet management. Possible approaches include adding it to telemetry, exposing it through a Particle Function, recording it in NodeDB, or using another gateway-side reporting path. No implementation has been selected yet.

### 3. Gateway uptime diagnostics

Gateway uptime counter behavior after reflashing appeared inconsistent and needs investigation. Retained state, RTC-derived uptime, or reset/reflash interactions may be involved.

### 4. RecoveryListen observations

RecoveryListen exists to give the node a lightweight way to rediscover the gateway when ACK history is stale, missing, or time is invalid. It intentionally remains conservative: short discovery sleeps, power-policy gating, no retry storm, and no aggressive escalation during discovery attempts. The goal is to regain contact while protecting battery and avoiding unnecessary radio churn.

### 5. ACK protocol evolution

ACK v1 overloads bytes 6-7 for both persistent cadence and next sleep interval. A future protocol could separate persistent reporting cadence from transient next-sleep instructions. No ACK protocol changes are planned for the current production release.

### 6. NodeDB persistence

Gateway-side NodeDB save timing was discussed because LoRa receive-window timing is sensitive. Current testing found no evidence of a production issue. Treat any NodeDB persistence adjustment as a future optimization only if field data shows it is warranted.

## Lessons Learned

- Persistent configuration and transient operational state must be kept separate. The overnight bug came from treating a one-shot closed-hours sleep interval like a persistent cadence candidate.
- RAM-only state was preferable to FRAM for closed-hours sleep because the value is consumed once, should not survive unrelated resets as configuration, and should not churn persistent storage.
- Node-only fixes are preferred when protocol compatibility can be preserved. This reduced deployment risk and avoided gateway, NodeDB, FRAM, and wire-format changes during stabilization.
- Production observability matters. The added ACK, cadence, sleep, recovery, power, and transaction logs made bench validation and overnight soak results much easier to interpret.

