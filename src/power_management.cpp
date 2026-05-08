#include "power_management.h"

#include <math.h>

#include "device_pinout.h"

namespace {

const float MIN_VALID_BATTERY_VOLTAGE = 3.0f;
const float MAX_VALID_BATTERY_VOLTAGE = 4.5f;
const float SUSPICIOUS_EMPTY_VOLTAGE = 3.45f;
#if HAL_PLATFORM_FUELGAUGE_MAX17043
FuelGauge fuelGauge;
#endif

#if !defined(POWER_MANAGER_VBAT_SCALE)
#define POWER_MANAGER_VBAT_SCALE 1.0f
#endif

#if defined(POWER_MANAGER_VBAT_PIN)
const bool PLATFORM_SUPPORTS_VBAT_ADC = true;
#else
const bool PLATFORM_SUPPORTS_VBAT_ADC = false;
#endif

bool isValidSocValue(float soc) {
	return !isnan(soc) && soc >= 0.0f && soc <= 100.0f;
}

bool isValidBatteryVoltageValue(float voltage) {
	return !isnan(voltage) && voltage >= MIN_VALID_BATTERY_VOLTAGE && voltage <= MAX_VALID_BATTERY_VOLTAGE;
}

float constrainPercent(float value) {
	if (isnan(value)) return value;
	if (value < 0.0f) return 0.0f;
	if (value > 100.0f) return 100.0f;
	return value;
}

void serviceDelay(uint16_t delayMs) {
	for (uint32_t start = millis(); millis() - start < delayMs; Particle.process()) {
	}
}

bool platformSupportsFuelGauge() {
#if HAL_PLATFORM_FUELGAUGE_MAX17043
	return true;
#else
	return false;
#endif
}

bool platformSupportsChargingControl() {
#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	return true;
#else
	return false;
#endif
}

bool platformSupportsBatteryContext() {
#if HAL_PLATFORM_POWER_MANAGEMENT
	return true;
#else
	return false;
#endif
}

bool platformSupportsPmicPowerConfiguration() {
#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	return true;
#else
	return false;
#endif
}

float readSystemBatterySoc() {
#if HAL_PLATFORM_POWER_MANAGEMENT
	return System.batteryCharge();
#else
	return NAN;
#endif
}

int readSystemBatteryState() {
#if HAL_PLATFORM_POWER_MANAGEMENT
	return (int)System.batteryState();
#else
	return -1;
#endif
}

int readSystemPowerSource() {
#if HAL_PLATFORM_POWER_MANAGEMENT
	return (int)System.powerSource();
#else
	return 0;
#endif
}

float readBatteryVoltage() {
#if HAL_PLATFORM_FUELGAUGE_MAX17043
	return fuelGauge.getVCell();
#elif defined(POWER_MANAGER_VBAT_PIN)
	int raw = analogRead(POWER_MANAGER_VBAT_PIN);
	if (raw < 0) return NAN;
	return (((float)raw) * 3.3f / 4095.0f) * POWER_MANAGER_VBAT_SCALE;
#else
	return NAN;
#endif
}

float estimateSocFromVoltage(float voltage) {
	if (!isValidBatteryVoltageValue(voltage)) return NAN;
	if (voltage <= 3.30f) return 0.0f;
	if (voltage <= 3.60f) return (voltage - 3.30f) * (20.0f / 0.30f);
	if (voltage <= 3.80f) return 20.0f + (voltage - 3.60f) * (40.0f / 0.20f);
	if (voltage <= 4.00f) return 60.0f + (voltage - 3.80f) * (25.0f / 0.20f);
	if (voltage <= 4.20f) return 85.0f + (voltage - 4.00f) * (15.0f / 0.20f);
	return 100.0f;
}

PowerBatteryContext mapBatteryContext(int rawState) {
	switch (rawState) {
	case 1:
		return PowerBatteryContext::NotCharging;
	case 2:
		return PowerBatteryContext::Charging;
	case 3:
		return PowerBatteryContext::Charged;
	case 4:
		return PowerBatteryContext::Discharging;
	case 5:
		return PowerBatteryContext::Fault;
	case 6:
		return PowerBatteryContext::Disconnected;
	default:
		return PowerBatteryContext::Unknown;
	}
}

bool isContextConsistentWithEmpty(PowerBatteryContext context) {
	return context == PowerBatteryContext::Discharging ||
		context == PowerBatteryContext::Fault ||
		context == PowerBatteryContext::Disconnected;
}

const char *powerSourceLabel(int powerSource) {
#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	switch (powerSource) {
	case POWER_SOURCE_USB_HOST:
		return "USB_HOST";
	case POWER_SOURCE_USB_ADAPTER:
		return "USB_ADAPTER";
	case POWER_SOURCE_VIN:
		return "VIN";
	case POWER_SOURCE_BATTERY:
		return "BATTERY";
	#ifdef POWER_SOURCE_USB_OTG
	case POWER_SOURCE_USB_OTG:
		return "USB_OTG";
	#endif
	default:
		return "UNKNOWN";
	}
#else
	(void)powerSource;
	return "UNSUPPORTED";
#endif
}

#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
struct PmicProfileSettings {
	PowerInputProfile profile;
	const char *label;
	int powerSourceMaxCurrentMa;
	int powerSourceMinVoltageMv;
	int batteryChargeCurrentMa;
	int batteryChargeVoltageMv;
};

const PmicProfileSettings USB_BENCH_PROFILE = {
	PowerInputProfile::UsbBench,
	"UsbBench",
	500,
	4200,
	512,
	4112,
};

const PmicProfileSettings SOLAR_35W_PROFILE = {
	PowerInputProfile::Solar35W,
	"Solar35W",
	550,
	4800,
	384,
	4112,
};

const PmicProfileSettings &settingsForProfile(PowerInputProfile profile) {
	switch (profile) {
	case PowerInputProfile::UsbBench:
		return USB_BENCH_PROFILE;
	case PowerInputProfile::Solar35W:
	case PowerInputProfile::Auto:
	case PowerInputProfile::NotApplicable:
	default:
		return SOLAR_35W_PROFILE;
	}
}
#endif

} // namespace

PowerManager &PowerManager::instance() {
	static PowerManager instance;
	return instance;
}

bool PowerManager::setup(const PowerManagerConfig &config) {
	config_ = config;
	lastReport_ = {};
	lastReport_.capabilities = detectCapabilities();
	lastReport_.activeInputProfile = PowerInputProfile::NotApplicable;
	lastReport_.policy.mode = LowPowerMode::Survival;
	lastReport_.policy.shouldUseReducedRetry = true;
	lastReport_.policy.shouldUseLongSleep = true;
	lastReport_.action.chargingControlStatus = lastReport_.capabilities.hasChargingControl ? PowerAvailability::Unknown : PowerAvailability::NotAvailable;
	lastReport_.action.pmicRemediationStatus = PowerAvailability::NotAvailable;
	lastAppliedChargingCommand_ = ChargingCommand::NoAction;
	wakeCycleStartedAtMs_ = millis();
	wokeFromSleep_ = false;
	configured_ = true;
	lastKnownGoodSoc_ = NAN;
	lastKnownGoodVoltage_ = NAN;
	lastKnownGoodContext_ = PowerBatteryContext::Unknown;
	activeInputProfile_ = PowerInputProfile::NotApplicable;
	lastAppliedInputProfile_ = PowerInputProfile::NotApplicable;
	pmicProfileAppliedThisBoot_ = false;
	bool profileApplied = applyBasePowerProfile();
	lastReport_.activeInputProfile = activeInputProfile_;
	return profileApplied;
}

void PowerManager::beginWakeCycle(bool wokeFromSleep) {
	wakeCycleStartedAtMs_ = millis();
	wokeFromSleep_ = wokeFromSleep;
}

const PowerReport &PowerManager::sample(float enclosureTempC) {
	if (!configured_) setup();
	lastReport_.capabilities = detectCapabilities();
	applyBasePowerProfile();
	lastReport_.reading = collectReading(enclosureTempC);
	lastReport_.policy = buildPolicy(lastReport_.reading);
	lastReport_.action = buildAction(lastReport_.reading, enclosureTempC);
	lastReport_.activeInputProfile = activeInputProfile_;
	return lastReport_;
}

bool PowerManager::applyRecommendedAction() {
	if (!configured_) return false;
	if (lastReport_.action.chargingCommand == ChargingCommand::NotAvailable ||
		lastReport_.action.chargingCommand == ChargingCommand::NoAction) {
		return true;
	}
	if (lastAppliedChargingCommand_ == lastReport_.action.chargingCommand) {
		return true;
	}
	bool enableCharging = lastReport_.action.chargingCommand == ChargingCommand::Enable;
	bool success = setChargingEnabled(enableCharging);
	if (success) {
		lastAppliedChargingCommand_ = lastReport_.action.chargingCommand;
		lastReport_.action.chargingControlStatus = PowerAvailability::Valid;
	}
	else {
		lastReport_.action.chargingControlStatus = PowerAvailability::Unknown;
	}
	return success;
}

const PowerReport &PowerManager::lastReport() const {
	return lastReport_;
}

const PowerPolicy &PowerManager::lastPolicy() const {
	return lastReport_.policy;
}

uint8_t PowerManager::encodeBatteryContext(PowerBatteryContext context) {
	return (uint8_t) context;
}

const char *PowerManager::batteryContextLabel(PowerBatteryContext context) {
	switch (context) {
	case PowerBatteryContext::Unknown:
		return "Unknown";
	case PowerBatteryContext::NotCharging:
		return "Not Charging";
	case PowerBatteryContext::Charging:
		return "Charging";
	case PowerBatteryContext::Charged:
		return "Charged";
	case PowerBatteryContext::Discharging:
		return "Discharging";
	case PowerBatteryContext::Fault:
		return "Fault";
	case PowerBatteryContext::Disconnected:
		return "Disconnected";
	case PowerBatteryContext::NotApplicable:
		return "N/A";
	default:
		return "Unknown";
	}
}

const char *PowerManager::batteryContextLabel(uint8_t encodedContext) {
	return batteryContextLabel((PowerBatteryContext) encodedContext);
}

const char *PowerManager::availabilityLabel(PowerAvailability availability) {
	switch (availability) {
	case PowerAvailability::Valid:
		return "valid";
	case PowerAvailability::Unknown:
		return "unknown";
	case PowerAvailability::NotAvailable:
		return "n/a";
	case PowerAvailability::Fallback:
		return "fallback";
	default:
		return "unknown";
	}
}

const char *PowerManager::powerInputProfileLabel(PowerInputProfile profile) {
	switch (profile) {
	case PowerInputProfile::UsbBench:
		return "UsbBench";
	case PowerInputProfile::Solar35W:
		return "Solar35W";
	case PowerInputProfile::Auto:
		return "Auto";
	case PowerInputProfile::NotApplicable:
	default:
		return "n/a";
	}
}

PowerCapabilities PowerManager::detectCapabilities() const {
	PowerCapabilities capabilities;
	capabilities.hasSoc = platformSupportsFuelGauge() || PLATFORM_SUPPORTS_VBAT_ADC;
	capabilities.hasBatteryVoltage = platformSupportsFuelGauge() || PLATFORM_SUPPORTS_VBAT_ADC;
	capabilities.hasBatteryContext = platformSupportsBatteryContext();
	capabilities.hasChargingControl = platformSupportsChargingControl();
	capabilities.hasPmicPowerConfiguration = platformSupportsPmicPowerConfiguration();
	capabilities.hasPmicRemediation = false;
	return capabilities;
}

bool PowerManager::applyBasePowerProfile() {
	if (!platformSupportsPmicPowerConfiguration()) {
		activeInputProfile_ = PowerInputProfile::NotApplicable;
		return true;
	}

	int powerSource = readSystemPowerSource();
	const char *reason = "unsupported";
	PowerInputProfile resolvedProfile = resolvePowerInputProfile(powerSource, reason);
	activeInputProfile_ = resolvedProfile;

#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	bool shouldApplyProfile = !pmicProfileAppliedThisBoot_ || lastAppliedInputProfile_ != resolvedProfile;
	if (!shouldApplyProfile) {
		#if VERBOSE_SYSTEM_LOGS
		Log.info("Power source raw=%d selected=%s reason=%s reused=1", powerSource, powerInputProfileLabel(resolvedProfile), reason);
		#endif
		return true;
	}

	const PmicProfileSettings &settings = settingsForProfile(resolvedProfile);
	SystemPowerConfiguration conf;
	conf.powerSourceMaxCurrent(settings.powerSourceMaxCurrentMa)
		.powerSourceMinVoltage(settings.powerSourceMinVoltageMv)
		.batteryChargeCurrent(settings.batteryChargeCurrentMa)
		.batteryChargeVoltage(settings.batteryChargeVoltageMv);
	int result = System.setPowerConfiguration(conf);
	if (result == SYSTEM_ERROR_NONE) {
		lastAppliedInputProfile_ = resolvedProfile;
		pmicProfileAppliedThisBoot_ = true;
		Log.info("PMIC profile: %s source=%s", settings.label, powerSourceLabel(powerSource));
		#if VERBOSE_SYSTEM_LOGS
		Log.info("Power source raw=%d selected=%s reason=%s reused=0", powerSource, powerInputProfileLabel(resolvedProfile), reason);
		Log.info("PMIC settings: input=%dmA minV=%d charge=%d chargeV=%d",
			settings.powerSourceMaxCurrentMa,
			settings.powerSourceMinVoltageMv,
			settings.batteryChargeCurrentMa,
			settings.batteryChargeVoltageMv);
		#endif
		return true;
	}
	Log.error("PMIC profile apply failed: %s (%d)", settings.label, result);
	return false;
#else
	return true;
#endif
}

PowerInputProfile PowerManager::configuredFallbackInputProfile() const {
	if (config_.inputProfile == PowerInputProfile::UsbBench ||
		config_.inputProfile == PowerInputProfile::Solar35W) {
		return config_.inputProfile;
	}
	return POWER_MANAGER_CONFIGURED_FALLBACK_INPUT_PROFILE;
}

PowerInputProfile PowerManager::resolvePowerInputProfile(int &powerSource, const char *&reason) const {
	if (!platformSupportsPmicPowerConfiguration()) {
		reason = "unsupported_platform";
		return PowerInputProfile::NotApplicable;
	}
	if (config_.inputProfile == PowerInputProfile::UsbBench ||
		config_.inputProfile == PowerInputProfile::Solar35W) {
		reason = "manual_override";
		return config_.inputProfile;
	}

	switch (powerSource) {
	case POWER_SOURCE_USB_HOST:
		reason = "usb_power_source";
		return PowerInputProfile::UsbBench;
	case POWER_SOURCE_USB_ADAPTER:
		reason = "usb_power_source";
		return PowerInputProfile::UsbBench;
	#ifdef POWER_SOURCE_USB_OTG
	case POWER_SOURCE_USB_OTG:
		reason = "usb_power_source";
		return PowerInputProfile::UsbBench;
	#endif
	case POWER_SOURCE_VIN:
		reason = "vin_power_source";
		return PowerInputProfile::Solar35W;
	case POWER_SOURCE_BATTERY:
		if (pmicProfileAppliedThisBoot_ && lastAppliedInputProfile_ != PowerInputProfile::NotApplicable) {
			reason = "battery_keep_last_applied";
			return lastAppliedInputProfile_;
		}
		reason = "battery_default_profile";
		return configuredFallbackInputProfile();
	default:
		reason = "default_profile";
		return configuredFallbackInputProfile();
	}
}

bool PowerManager::setChargingEnabled(bool enableCharging) {
	if (!platformSupportsChargingControl()) {
		return false;
	}

#if HAL_PLATFORM_POWER_MANAGEMENT && HAL_PLATFORM_PMIC_BQ24195
	PMIC pmic;
	return enableCharging ? pmic.enableCharging() : pmic.disableCharging();
#else
	return false;
#endif
}

PowerReading PowerManager::collectReading(float enclosureTempC) {
	PowerReading reading = collectRawReading(enclosureTempC);
	bool suspiciousSocLogged = false;
	bool suspiciousSoc = isSocSuspicious(reading, true);
	suspiciousSocLogged = suspiciousSoc;
	bool needsStabilization = reading.socStatus == PowerAvailability::Unknown || suspiciousSoc;
	if (needsStabilization && platformSupportsFuelGauge()) {
		uint32_t stabilizationStartedAt = millis();
		fuelGauge.quickStart();
		reading.quickStartUsed = true;
		for (uint8_t attempt = 0; attempt < config_.maxStabilizationAttempts; attempt++) {
			uint16_t delayMs = config_.stabilizationDelayMs[attempt];
			if (millis() - stabilizationStartedAt + delayMs > config_.stabilizationBudgetMs) break;
			serviceDelay(delayMs);
			reading = collectRawReading(enclosureTempC);
			reading.quickStartUsed = true;
			reading.stabilizationAttempts = attempt + 1;
			suspiciousSoc = isSocSuspicious(reading, !suspiciousSocLogged);
			if (suspiciousSoc) suspiciousSocLogged = true;
			needsStabilization = reading.socStatus == PowerAvailability::Unknown || suspiciousSoc;
			if (!needsStabilization) break;
		}
	}
	suspiciousSoc = isSocSuspicious(reading, false);
	needsStabilization = reading.socStatus == PowerAvailability::Unknown || suspiciousSoc;
	if (needsStabilization) {
		applyFallback(reading);
		suspiciousSoc = isSocSuspicious(reading, false);
		needsStabilization = reading.socStatus == PowerAvailability::Unknown || suspiciousSoc;
		if (needsStabilization && !reading.usedSocFallback) {
			reading.socStatus = lastReport_.capabilities.hasSoc ? PowerAvailability::Unknown : PowerAvailability::NotAvailable;
		}
	}
	updateLastKnownGood(reading);
	return reading;
}

PowerReading PowerManager::collectRawReading(float enclosureTempC) {
	(void) enclosureTempC;
	PowerReading reading;
	reading.socStatus = PowerAvailability::NotAvailable;
	reading.batteryVoltageStatus = PowerAvailability::NotAvailable;
	reading.batteryContextStatus = PowerAvailability::NotAvailable;

	float batteryVoltage = readBatteryVoltage();
	if (isValidBatteryVoltageValue(batteryVoltage)) {
		reading.batteryVoltage = batteryVoltage;
		reading.batteryVoltageStatus = PowerAvailability::Valid;
	}
	else if (lastReport_.capabilities.hasBatteryVoltage) {
		reading.batteryVoltage = batteryVoltage;
		reading.batteryVoltageStatus = PowerAvailability::Unknown;
	}

	if (platformSupportsFuelGauge()) {
		float rawSoc = readSystemBatterySoc();
		reading.soc = rawSoc;
		reading.socStatus = isValidSocValue(rawSoc) ? PowerAvailability::Valid : PowerAvailability::Unknown;
	}
	else if (PLATFORM_SUPPORTS_VBAT_ADC && reading.batteryVoltageStatus == PowerAvailability::Valid) {
		reading.soc = constrainPercent(estimateSocFromVoltage(reading.batteryVoltage));
		reading.socStatus = isValidSocValue(reading.soc) ? PowerAvailability::Valid : PowerAvailability::Unknown;
	}

	if (platformSupportsBatteryContext()) {
		int rawState = readSystemBatteryState();
		reading.batteryContext = mapBatteryContext(rawState);
		reading.batteryContextStatus = reading.batteryContext == PowerBatteryContext::Unknown ? PowerAvailability::Unknown : PowerAvailability::Valid;
	}
	else {
		reading.batteryContext = PowerBatteryContext::NotApplicable;
		reading.batteryContextStatus = PowerAvailability::NotAvailable;
	}

	return reading;
}

PowerPolicy PowerManager::buildPolicy(const PowerReading &reading) const {
	PowerPolicy policy;
	policy.mode = LowPowerMode::Survival;
	policy.shouldUseReducedRetry = true;
	policy.shouldUseLongSleep = true;
	bool usableSoc = (reading.socStatus == PowerAvailability::Valid || reading.socStatus == PowerAvailability::Fallback) &&
		isValidSocValue(reading.soc);
	if (!usableSoc) {
		return policy;
	}
	if (reading.soc >= config_.normalMinSoc) policy.mode = LowPowerMode::Normal;
	else if (reading.soc >= config_.conserveMinSoc) policy.mode = LowPowerMode::Conserve;
	else if (reading.soc >= config_.criticalMinSoc) policy.mode = LowPowerMode::Critical;
	else policy.mode = LowPowerMode::Survival;

	policy.shouldAllowDiscoveryMode =
		(reading.socStatus == PowerAvailability::Valid && reading.soc >= config_.minSocForDiscoveryMode) ||
		(reading.socStatus == PowerAvailability::Fallback && reading.soc >= config_.normalMinSoc);
	policy.shouldAllowRecoveryPowerdown = reading.socStatus == PowerAvailability::Valid && reading.soc >= config_.minSocForRecoveryPowerdown;
	policy.shouldUseReducedRetry = policy.mode == LowPowerMode::Critical || policy.mode == LowPowerMode::Survival;
	policy.shouldUseLongSleep = policy.mode == LowPowerMode::Critical || policy.mode == LowPowerMode::Survival;
	return policy;
}

PowerAction PowerManager::buildAction(const PowerReading &reading, float enclosureTempC) const {
	PowerAction action;
	(void) reading;
	action.pmicRemediationStatus = PowerAvailability::NotAvailable;
	if (!platformSupportsChargingControl()) {
		action.chargingCommand = ChargingCommand::NotAvailable;
		action.chargingControlStatus = PowerAvailability::NotAvailable;
		return action;
	}
	action.chargingControlStatus = PowerAvailability::Valid;
	if (isnan(enclosureTempC)) {
		action.chargingCommand = ChargingCommand::NoAction;
		return action;
	}
	action.chargingCommand = (enclosureTempC < config_.minSafeChargeTempC || enclosureTempC > config_.maxSafeChargeTempC) ?
		ChargingCommand::Disable : ChargingCommand::Enable;
	return action;
}

bool PowerManager::shouldStabilize(const PowerReading &reading) const {
	return reading.socStatus == PowerAvailability::Unknown || isSocSuspicious(reading, false);
}

bool PowerManager::isSocSuspicious(const PowerReading &reading, bool logIt) const {
	if (!isValidSocValue(reading.soc)) return false;
	if (reading.soc == 0.0f) {
		if (reading.batteryVoltageStatus == PowerAvailability::Valid && reading.batteryVoltage > SUSPICIOUS_EMPTY_VOLTAGE) return true;
		if (reading.batteryContextStatus == PowerAvailability::Valid && !isContextConsistentWithEmpty(reading.batteryContext)) return true;
	}
	if (reading.soc == 100.0f) {
		bool voltageClearlyFull = reading.batteryVoltageStatus == PowerAvailability::Valid &&
			reading.batteryVoltage >= config_.fullBatteryVoltageValidSoc100;
		bool contextCharged = reading.batteryContextStatus == PowerAvailability::Valid &&
			reading.batteryContext == PowerBatteryContext::Charged;
		bool suspiciousContext = reading.batteryContextStatus != PowerAvailability::Valid ||
			reading.batteryContext == PowerBatteryContext::Charging ||
			reading.batteryContext == PowerBatteryContext::NotCharging ||
			reading.batteryContext == PowerBatteryContext::Discharging ||
			reading.batteryContext == PowerBatteryContext::Unknown ||
			reading.batteryContext == PowerBatteryContext::Fault;

		if (!voltageClearlyFull && (suspiciousContext || contextCharged)) {
			if (logIt) Log.info("SOC 100 suspicious: context=%s voltage=%.2f", batteryContextLabel(reading.batteryContext), reading.batteryVoltage);
			return true;
		}
	}
	return false;
}

float PowerManager::estimateConservativeFallbackSocFromVoltage(float batteryVoltage) const {
	if (!isValidBatteryVoltageValue(batteryVoltage)) return NAN;
	if (batteryVoltage <= 3.50f) return 10.0f;
	if (batteryVoltage >= 4.18f) return 98.0f;

	struct VoltageSocPoint {
		float voltage;
		float soc;
	};

	static const VoltageSocPoint points[] = {
		{3.50f, 10.0f},
		{3.70f, 40.0f},
		{3.85f, 60.0f},
		{4.00f, 80.0f},
		{4.10f, 90.0f},
		{4.18f, 98.0f},
	};

	for (size_t index = 1; index < sizeof(points) / sizeof(points[0]); index++) {
		if (batteryVoltage <= points[index].voltage) {
			float lowerVoltage = points[index - 1].voltage;
			float upperVoltage = points[index].voltage;
			float lowerSoc = points[index - 1].soc;
			float upperSoc = points[index].soc;
			float position = (batteryVoltage - lowerVoltage) / (upperVoltage - lowerVoltage);
			float estimate = lowerSoc + position * (upperSoc - lowerSoc);
			if (estimate > 98.0f) estimate = 98.0f;
			return estimate;
		}
	}

	return 98.0f;
}

void PowerManager::updateLastKnownGood(const PowerReading &reading) {
	if (reading.socStatus == PowerAvailability::Valid && isValidSocValue(reading.soc) && !isSocSuspicious(reading, false)) {
		lastKnownGoodSoc_ = reading.soc;
	}
	if (reading.batteryVoltageStatus == PowerAvailability::Valid && isValidBatteryVoltageValue(reading.batteryVoltage)) {
		lastKnownGoodVoltage_ = reading.batteryVoltage;
	}
	if (reading.batteryContextStatus == PowerAvailability::Valid) {
		lastKnownGoodContext_ = reading.batteryContext;
	}
}

void PowerManager::applyFallback(PowerReading &reading) const {
	if (isnan(reading.soc) && !isnan(lastKnownGoodSoc_)) {
		reading.soc = lastKnownGoodSoc_;
		reading.socStatus = PowerAvailability::Fallback;
		reading.usedSocFallback = true;
	}
	else if ((reading.socStatus == PowerAvailability::Unknown || isSocSuspicious(reading, false)) && !isnan(lastKnownGoodSoc_)) {
		reading.soc = lastKnownGoodSoc_;
		reading.socStatus = PowerAvailability::Fallback;
		reading.usedSocFallback = true;
	}
	else if ((reading.socStatus == PowerAvailability::Unknown || isSocSuspicious(reading, false)) &&
		reading.batteryVoltageStatus == PowerAvailability::Valid) {
		float estimatedSoc = estimateConservativeFallbackSocFromVoltage(reading.batteryVoltage);
		if (!isnan(estimatedSoc)) {
			reading.soc = estimatedSoc;
			reading.socStatus = PowerAvailability::Fallback;
			reading.usedSocFallback = true;
		}
	}
	if (reading.batteryVoltageStatus == PowerAvailability::Unknown && !isnan(lastKnownGoodVoltage_)) {
		reading.batteryVoltage = lastKnownGoodVoltage_;
		reading.batteryVoltageStatus = PowerAvailability::Fallback;
		reading.usedVoltageFallback = true;
	}
	if (reading.batteryContextStatus == PowerAvailability::Unknown && lastKnownGoodContext_ != PowerBatteryContext::Unknown) {
		reading.batteryContext = lastKnownGoodContext_;
		reading.batteryContextStatus = PowerAvailability::Fallback;
	}
	if (reading.socStatus == PowerAvailability::Unknown && isnan(lastKnownGoodSoc_)) {
		reading.soc = NAN;
	}
}