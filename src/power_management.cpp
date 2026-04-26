/**
 * @file power_management.cpp
 * @author Chip McClelland
 * @brief Implementation of reusable, time-agnostic power management helpers.
 *
 * @details
 * This file contains the local policy and state used to evaluate charging
 * safety, detect sustained charging anomalies from observation samples, and
 * perform a bounded charger toggle recovery on supported Particle platforms.
 */
#include "power_management.h"

#include <math.h>

namespace {

const int MAX_CURRENT_FROM_PANEL = 900;
const int POWER_SOURCE_MIN_VOLTAGE_MV = 5080;
const int BATTERY_CHARGE_VOLTAGE_MV = 4208;
const float MIN_SAFE_CHARGE_TEMP_F = 32.0f;
const float MAX_SAFE_CHARGE_TEMP_F = 100.0f;
const float HIGH_BATTERY_SOC_THRESHOLD = 90.0f;
const float MIN_VALID_BATTERY_SOC = 0.0f;
const float MAX_VALID_BATTERY_SOC = 100.0f;
const float MIN_VALID_BATTERY_VOLTAGE = 3.0f;
const float MAX_VALID_BATTERY_VOLTAGE = 4.5f;
const uint8_t CHARGING_EXPECTED_NOT_CHARGING_THRESHOLD_SAMPLES = 3;
const uint8_t CHARGE_FAULT_THRESHOLD_SAMPLES = 2;
const uint8_t MAX_RECOVERY_ATTEMPTS_PER_CYCLE = 1;
const uint32_t CHARGING_TOGGLE_DELAY_MS = 1000;

PowerObservation lastObservation = {};
bool hasObservation = false;
uint8_t consecutiveChargingExpectedNotChargingSamples = 0;
uint8_t consecutiveChargeFaultSamples = 0;
uint16_t totalPowerRecoveryAttempts = 0;
uint8_t recoveryAttemptsThisCycle = 0;
PowerManagementAlertCode lastPowerAlertCode = PowerManagementAlertCode::NONE;
bool loggedUnsupportedChargingControl = false;
bool chargingSafetyStateInitialized = false;
bool chargingSafetyEnabled = true;

bool isValidBatterySoc(float batterySoc) {
	return !isnan(batterySoc) && batterySoc >= MIN_VALID_BATTERY_SOC && batterySoc <= MAX_VALID_BATTERY_SOC;
}

bool isValidBatteryVoltage(float batteryVoltage) {
	return !isnan(batteryVoltage) && batteryVoltage >= MIN_VALID_BATTERY_VOLTAGE && batteryVoltage <= MAX_VALID_BATTERY_VOLTAGE;
}

bool isTemperatureSafeForCharging(float temperatureF) {
	return !isnan(temperatureF) && temperatureF >= MIN_SAFE_CHARGE_TEMP_F && temperatureF <= MAX_SAFE_CHARGE_TEMP_F;
}

bool areReadingsValid(const PowerObservation& obs) {
	return isValidBatterySoc(obs.batterySoc) &&
		isValidBatteryVoltage(obs.batteryVoltage) &&
		!isnan(obs.temperatureF);
}

bool isChargingExpected(const PowerObservation& obs) {
	return areReadingsValid(obs) &&
		obs.inputPowerPresent &&
		isTemperatureSafeForCharging(obs.temperatureF) &&
		obs.batterySoc < HIGH_BATTERY_SOC_THRESHOLD;
}

bool supportsChargingControl() {
#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	return true;
#else
	return false;
#endif
}

void logUnsupportedChargingControlOnce() {
	if (!loggedUnsupportedChargingControl) {
		Log.info("Power management running without PMIC charge control on this platform");
		loggedUnsupportedChargingControl = true;
	}
}

bool applySolarPowerConfiguration(bool enableCharging) {
	if (!supportsChargingControl()) {
		logUnsupportedChargingControlOnce();
		return false;
	}

	SystemPowerConfiguration conf;

	conf.feature(SystemPowerFeature::PMIC_DETECTION)
		.powerSourceMaxCurrent(MAX_CURRENT_FROM_PANEL)
		.powerSourceMinVoltage(POWER_SOURCE_MIN_VOLTAGE_MV)
		.batteryChargeCurrent(MAX_CURRENT_FROM_PANEL)
		.batteryChargeVoltage(BATTERY_CHARGE_VOLTAGE_MV)
		.feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST);

	if (!enableCharging) {
		conf.feature(SystemPowerFeature::DISABLE_CHARGING);
	}

	int result = System.setPowerConfiguration(conf);
	if (result == SYSTEM_ERROR_NONE) {
		Log.info("Power configuration applied (%s)", enableCharging ? "charging enabled" : "charging disabled");
		return true;
	}

	Log.error("Power configuration failed (%s): %d", enableCharging ? "charging enabled" : "charging disabled", result);
	return false;
}

bool applyChargingSafetyTransition(const PowerObservation& obs) {
	if (!supportsChargingControl()) {
		logUnsupportedChargingControlOnce();
		return true;
	}

	if (isnan(obs.temperatureF)) {
		return true;
	}

	bool enableCharging = isTemperatureSafeForCharging(obs.temperatureF);
	if (chargingSafetyStateInitialized && chargingSafetyEnabled == enableCharging) {
		return true;
	}

	bool result = applySolarPowerConfiguration(enableCharging);
	if (result) {
		chargingSafetyStateInitialized = true;
		chargingSafetyEnabled = enableCharging;
		if (!enableCharging) {
			Log.info("Charging disabled by temperature safety policy at %.1fF", obs.temperatureF);
		}
	}
	return result;
}

void resetAnomalyCounters() {
	consecutiveChargingExpectedNotChargingSamples = 0;
	consecutiveChargeFaultSamples = 0;
}

void resetRecoveryCycleState() {
	recoveryAttemptsThisCycle = 0;
}


bool performChargingToggle() {
    if (!applySolarPowerConfiguration(false)) {
        return false;
    }

    delay(CHARGING_TOGGLE_DELAY_MS);

    if (!hasObservation || !areReadingsValid(lastObservation)) {
		// Fail safe to avoid leaving charging disabled if the observation goes stale mid-toggle.
        chargingSafetyStateInitialized = false;
        applySolarPowerConfiguration(true);  // fail-safe: do not leave charging disabled
        return false;
    }

    chargingSafetyStateInitialized = false;
    return applyChargingSafetyTransition(lastObservation);
}

}

bool initializePowerManagement() {
	hasObservation = false;
	lastObservation = {};
	resetAnomalyCounters();
	resetRecoveryCycleState();
	lastPowerAlertCode = PowerManagementAlertCode::NONE;
	chargingSafetyStateInitialized = false;
	chargingSafetyEnabled = true;

	if (!supportsChargingControl()) {
		logUnsupportedChargingControlOnce();
		return true;
	}

	Log.info("Initializing power management with charging enabled solar configuration");
	bool result = applySolarPowerConfiguration(true);
	if (result) {
		chargingSafetyStateInitialized = true;
		chargingSafetyEnabled = true;
	}
	return result;
}

bool applyPowerManagementSafetyPolicy() {
	if (!hasObservation || !areReadingsValid(lastObservation)) {
		return true;
	}

	if (!supportsChargingControl()) {
		logUnsupportedChargingControlOnce();
		return true;
	}

	return applyChargingSafetyTransition(lastObservation);
}

void resetPowerManagementRecoveryCycle() {
	resetRecoveryCycleState();
}

void updatePowerManagementObservation(const PowerObservation& obs) {
	lastObservation = obs;
	hasObservation = true;

	if (!areReadingsValid(obs)) {
		resetAnomalyCounters();
		lastPowerAlertCode = PowerManagementAlertCode::NONE;
		return;
	}

	if (!isChargingExpected(obs) || obs.batteryIsCharging) {
		resetAnomalyCounters();
		lastPowerAlertCode = PowerManagementAlertCode::NONE;
		return;
	}

	if (obs.batteryFault) {
		consecutiveChargingExpectedNotChargingSamples = 0;
		if (consecutiveChargeFaultSamples < UINT8_MAX) {
			consecutiveChargeFaultSamples++;
		}
		lastPowerAlertCode = PowerManagementAlertCode::OBSERVING;
		return;
	}

	consecutiveChargeFaultSamples = 0;
	if (obs.batteryNotCharging) {
		if (consecutiveChargingExpectedNotChargingSamples < UINT8_MAX) {
			consecutiveChargingExpectedNotChargingSamples++;
		}
		lastPowerAlertCode = PowerManagementAlertCode::OBSERVING;
		return;
	}

	resetAnomalyCounters();
	lastPowerAlertCode = PowerManagementAlertCode::NONE;
}

PowerManagementAlertCode runPowerManagementRemediation() {
	if (!hasObservation || !areReadingsValid(lastObservation)) {
		lastPowerAlertCode = PowerManagementAlertCode::NONE;
		return lastPowerAlertCode;
	}

	if (!isTemperatureSafeForCharging(lastObservation.temperatureF)) {
		lastPowerAlertCode = PowerManagementAlertCode::NONE;
		return lastPowerAlertCode;
	}

	if (!isChargingExpected(lastObservation) || lastObservation.batteryIsCharging) {
		resetAnomalyCounters();
		lastPowerAlertCode = PowerManagementAlertCode::NONE;
		return lastPowerAlertCode;
	}

	bool chargeFaultThresholdExceeded = consecutiveChargeFaultSamples >= CHARGE_FAULT_THRESHOLD_SAMPLES;
	bool expectedNotChargingThresholdExceeded =
		consecutiveChargingExpectedNotChargingSamples >= CHARGING_EXPECTED_NOT_CHARGING_THRESHOLD_SAMPLES;

	if (!chargeFaultThresholdExceeded && !expectedNotChargingThresholdExceeded) {
		lastPowerAlertCode = (consecutiveChargeFaultSamples > 0 || consecutiveChargingExpectedNotChargingSamples > 0) ?
			PowerManagementAlertCode::OBSERVING :
			PowerManagementAlertCode::NONE;
		return lastPowerAlertCode;
	}

	if (recoveryAttemptsThisCycle >= MAX_RECOVERY_ATTEMPTS_PER_CYCLE) {
		lastPowerAlertCode = chargeFaultThresholdExceeded ?
			PowerManagementAlertCode::REQUEST_POWER_CYCLE :
			PowerManagementAlertCode::SERVICE_REQUIRED;
		return lastPowerAlertCode;
	}

	if (!supportsChargingControl()) {
		logUnsupportedChargingControlOnce();
		lastPowerAlertCode = PowerManagementAlertCode::SERVICE_REQUIRED;
		return lastPowerAlertCode;
	}

	totalPowerRecoveryAttempts++;
	recoveryAttemptsThisCycle++;
	if (!performChargingToggle()) {
		lastPowerAlertCode = PowerManagementAlertCode::REQUEST_POWER_CYCLE;
		return lastPowerAlertCode;
	}

	resetAnomalyCounters();
	lastPowerAlertCode = PowerManagementAlertCode::CHARGE_TOGGLE_DONE;
	return lastPowerAlertCode;
}

PowerManagementAlertCode getLastPowerManagementAlert() {
	return lastPowerAlertCode;
}

void clearPowerManagementAlert() {
	lastPowerAlertCode = PowerManagementAlertCode::NONE;
}

uint8_t getConsecutiveChargingExpectedNotChargingSamples() {
	return consecutiveChargingExpectedNotChargingSamples;
}

uint16_t getTotalPowerRecoveryAttempts() {
	return totalPowerRecoveryAttempts;
}