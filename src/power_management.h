/**
 * @file power_management.h
 * @author Chip McClelland
 * @brief Reusable, time-agnostic power management API for Particle devices.
 *
 * @details
 * This module accepts raw power observations from the application, applies
 * charging safety policy, tracks charging anomalies using event-based counters,
 * and performs bounded remediation when the application decides it is safe to
 * do so. The public API is intentionally compact so the same module can be
 * reused across LoRa, cellular, and Wi-Fi firmware.
 *
 * @par Example Usage
 * @code{.cpp}
 * void setup() {
 *     initializePowerManagement();
 * }
 *
 * void collectPowerData() {
 *     PowerObservation obs = {};
 *     obs.batterySoc = System.batteryCharge();
 *     obs.batteryVoltage = fuelGauge.getVCell();
 *     obs.temperatureF = measuredTemperatureF;
 *     obs.inputPowerPresent = externalPowerPresent;
 *     obs.batteryIsCharging = System.batteryState() == 2;
 *     obs.batteryNotCharging = System.batteryState() == 1;
 *     obs.batteryFault = System.batteryState() == 5;
 *
 *     updatePowerManagementObservation(obs);
 * }
 *
 * void loop() {
 *     if (safeToRemediate) {
 *         PowerManagementAlertCode alert = runPowerManagementRemediation();
 *         if (alert == PowerManagementAlertCode::REQUEST_POWER_CYCLE) {
 *             // Escalate using application-specific recovery.
 *         }
 *     }
 * }
 * @endcode
 */
#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "Particle.h"

/**
 * @brief Snapshot of raw battery and power inputs for one measurement cycle.
 */
struct PowerObservation {
	/** Battery state of charge in percent from 0 to 100. */
	float batterySoc;
	/** Battery terminal voltage in volts. */
	float batteryVoltage;
	/** Measured battery or enclosure temperature in degrees Fahrenheit. */
	float temperatureF;
	/** True when external input power such as VIN or USB host power is present. */
	bool inputPowerPresent;
	/** True when the platform reports that the battery is actively charging. */
	bool batteryIsCharging;
	/** True when the platform reports that the battery is not charging. */
	bool batteryNotCharging;
	/** True when the platform reports a battery or charger fault. */
	bool batteryFault;
};

/**
 * @brief Compact power-management outcomes exposed to the application.
 */
enum class PowerManagementAlertCode : uint8_t {
	NONE = 0,
	OBSERVING = 1,
	CHARGE_TOGGLE_DONE = 2,
	REQUEST_POWER_CYCLE = 3,
	SERVICE_REQUIRED = 4
};

/**
 * @brief Initializes module state and applies a safe startup charging policy.
 *
 * @details
 * On platforms with supported PMIC charge control, this initializes the power
 * configuration with charging disabled until the first observation is applied.
 * On unsupported platforms, this resets internal state and returns success
 * without attempting hardware charge control.
 *
 * @return `true` if initialization completed successfully for the current
 * platform, otherwise `false`.
 */
bool initializePowerManagement();

/**
 * @brief Updates the module with the latest raw power observation.
 *
 * @details
 * This function owns charging expectation evaluation, temperature-based charge
 * enable or disable decisions, and anomaly counter maintenance. It should be
 * called once per measurement cycle with fresh telemetry.
 *
 * @param obs Raw battery and power inputs for the current cycle.
 */
void updatePowerManagementObservation(const PowerObservation& obs);

/**
 * @brief Performs bounded remediation for a sustained charging anomaly.
 *
 * @details
 * The caller is responsible for deciding when it is safe to run remediation.
 * If thresholds are exceeded, the module attempts a short charging toggle and
 * returns a compact alert describing the outcome. This function never resets
 * the MCU or directly power cycles hardware.
 *
 * @return The most recent remediation outcome.
 */
PowerManagementAlertCode runPowerManagementRemediation();

/**
 * @brief Returns the most recent power-management alert.
 *
 * @return The last alert code produced by observation or remediation logic.
 */
PowerManagementAlertCode getLastPowerManagementAlert();

/**
 * @brief Clears the stored power-management alert.
 */
void clearPowerManagementAlert();

/**
 * @brief Returns the current count of consecutive expected-but-not-charging samples.
 *
 * @return The number of consecutive samples where charging was expected but not observed.
 */
uint8_t getConsecutiveChargingExpectedNotChargingSamples();

/**
 * @brief Returns the total number of remediation attempts performed.
 *
 * @return The cumulative count of charging recovery attempts since startup.
 */
uint16_t getTotalPowerRecoveryAttempts();

#endif