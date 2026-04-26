# Changelog

All notable changes to this project will be documented in this file.

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