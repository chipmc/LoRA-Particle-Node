# Changelog

All notable changes to this project will be documented in this file.

## v28.00 - 2026-08-13

### Added
- Per-node reporting stagger for collision avoidance: adds `nodeNumber * 2` seconds to the computed sleep duration.
- Stagger is applied after cadence validation and persistence, never touching the FRAM-backed `frequencyMinutes` value.
- Covers open-hours boundary-aligned reporting, closed-hours relative sleep, and closed-hours one-shot sleep.

### Changed
- Power manager input profile detection now defaults to AUTO (previously hardcoded to USB_BENCH), enabling dynamic detection based on actual power source at runtime.

### Validated
- Node 1 and Node 2 stagger formula verification across multiple sleep cycles.
- Wake time offset consistency and zero drift/compounding behavior.

## v25.0.1 - 2026-07-12

### Fixed
- Fixed the overnight scheduling regression caused by interpreting DATA_ACK bytes 6-7 as persistent cadence before decoding DATA_ACK `openHours` context.
- Closed-hours DATA_ACK schedule values are now treated as one-shot sleep intervals, so overnight values such as 479 minutes are accepted for the next sleep without being persisted.
- Prevented closed-hours 480-minute ACK values from overwriting the persistent reporting cadence.

### Changed
- DATA_ACK handling now decodes `openHours` before interpreting the schedule field.
- Closed-hours sleep intervals are stored in RAM-only pending sleep state and cleared after use.
- Open-hours DATA_ACK clears pending overnight state and resumes normal cadence handling.
- JOIN_ACK cadence handling is unchanged.

### Compatibility
- Node-only fix.
- No gateway protocol change.
- No FRAM layout change.
- No NodeDB change.
- No wire protocol change.

### Validated
- Overnight bench test passed.
- Extended overnight soak completed.
- Raleigh validation completed.
- Correct overnight sleep restored.
- Normal morning cadence resumed.

## v25.0.0 - 2026-06-25

### Added
- Lightweight discovery recovery mode for stale/missing gateway ACKs
- Boron-only USB source override for USB bench misclassification

### Changed
- ACK cadence persistence now accepts only 60–480 minute values in 60-minute increments
- Transient gateway schedule hints (56, 59, 30, 17, 12) no longer overwrite persisted cadence
- ACK schedule diagnostics now calculate nextBoundaryUtc using persisted cadence

### Fixed
- Invalid persisted frequencyMinutes repaired to default 60 at boot
- SleepCalc alignment preserved after transient gateway ACK schedule hints

### Validated
- ACK cadence guard behavior in production soak
- Persisted cadence repair on boot
- Boron USB source override functionality
- Corrected ACK schedule diagnostics
- Lightweight discovery recovery mode

## v24.00 - 2026-06-08

### Fixed
- Stopped the LoRa transmit guard from trapping internally scheduled retry sends in a runaway retry-wait loop after a failed transmit.

### Validated
- Promoted the retry-guard hardening fix for release packaging as firmware product version 24.

## v22.00 - 2026-05-17

### Changed
- Added battery wake stabilization and long-window `Energy24h` trend instrumentation for outdoor solar soak and field stability analysis.
- Hardened PMIC fault remediation logging, charging reset/recovery visibility, and compact power-source reporting without intended occupancy or sleep-policy behavior changes.
- Reduced routine serial truncation with compact PMIC, connect, sleep, and wake log cleanup.

### Fixed
- Made gateway ACK time authoritative for both system clock and AB1805 RTC updates, with boot RTC restore treated as provisional until a valid ACK is received.
- Added `CurrentData` FRAM migration support so v3 persisted data upgrades cleanly to the v22 layout without losing legacy counts and telemetry fields.
- Hardened `Energy24h` baseline handling to reject invalid, future, or stale timestamps and to suppress summary timing until an authoritative gateway ACK has occurred this boot.
- Ignored zero or invalid gateway ACK timestamps without updating system time, RTC, or Energy24h baseline timing, while preserving the existing retry and fallback scheduling behavior.

### Validated
- Successful soak validation with no intended changes to occupancy handling, sleep timing, connect sequencing, watchdog behavior, or connection budgets.

## v14.1 - 2026-04-26

### Changed
- Split power-management observation updates from explicit charger safety-policy application.
- Added defensive remediation guards so charging toggle recovery is skipped when temperature is unsafe.
- Added fail-safe handling to avoid leaving charging disabled if the observation goes stale during a toggle.

## v14.00 - 2026-04-26

### Added
- Reusable `power_management` module with Doxygen documentation and a public observation/remediation API.
- Time-agnostic power anomaly detection based on raw power observations instead of RTC or hourly logic.
- PMIC charge-toggle recovery path with compact alert codes for caller-controlled escalation.
- Low-battery mode helpers that shorten the listening window and reduce transmit retries.

### Changed
- Centralized all application power configuration inside `src/power_management.cpp`.
- Initialized power management explicitly during `setup()` before the first measurement cycle.
- Refreshed battery telemetry after power policy application so logged/reporting state reflects the current charger state.
- Added Particle platform capability gating so unsupported PMIC targets degrade safely.

### Fixed
- Propagated LoRa radio initialization failures instead of continuing with a bad setup.
- Hardened LoRa receive buffer handling and message length validation.
- Fixed sensor power gating around sleep and closed-hours behavior.
- Removed stale measurement helpers that no longer represented authoritative charging state.