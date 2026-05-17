#include "energy_trend.h"

#include "MyPersistentData.h"

#include <math.h>
#include <stdio.h>

namespace {

uint16_t saturatingIncrement16(uint16_t value) {
	return (value == 0xFFFF) ? value : (uint16_t)(value + 1);
}

uint32_t saturatingAdd32(uint32_t currentValue, uint32_t delta) {
	if (UINT32_MAX - currentValue < delta) {
		return UINT32_MAX;
	}
	return currentValue + delta;
}

uint32_t roundedMinutes(uint32_t durationMs) {
	return (durationMs + 30000UL) / 60000UL;
}

void formatAge(char *buffer, size_t bufferSize, time_t ageSeconds) {
	if (ageSeconds <= 0) {
		snprintf(buffer, bufferSize, "0m");
	}
	else if (ageSeconds < 2 * 60 * 60) {
		snprintf(buffer, bufferSize, "%lum", (unsigned long)((ageSeconds + 30) / 60));
	}
	else {
		snprintf(buffer, bufferSize, "%luh", (unsigned long)((ageSeconds + 1800) / 3600));
	}
}

} // anonymous namespace

EnergyTrend &EnergyTrend::instance() {
	static EnergyTrend instance;
	return instance;
}

void EnergyTrend::setup() {
	if (initialized_) {
		return;
	}
	initialized_ = true;
	awakeCycleOpen_ = false;
	lowPowerModeActive_ = false;
	lastBatteryContext_ = PowerBatteryContext::Unknown;
	haveLastBatteryContext_ = false;
	uint8_t encodedContext = current.get_batteryState();
	if (encodedContext <= (uint8_t)PowerBatteryContext::NotApplicable) {
		lastBatteryContext_ = (PowerBatteryContext)encodedContext;
		haveLastBatteryContext_ = true;
	}
}

void EnergyTrend::ensureInitialized() {
	if (!initialized_) {
		setup();
	}
}

void EnergyTrend::noteWake() {
	ensureInitialized();
	current.set_energyWakeCount(saturatingIncrement16(current.get_energyWakeCount()));
	awakeStartedAtMs_ = millis();
	awakeCycleOpen_ = true;
}

void EnergyTrend::noteConnection(uint32_t durationMs) {
	ensureInitialized();
	current.set_energyConnectionCount(saturatingIncrement16(current.get_energyConnectionCount()));
	current.set_energyConnectionMs(saturatingAdd32(current.get_energyConnectionMs(), durationMs));
	if (durationMs > current.get_energyLongestConnectionMs()) {
		current.set_energyLongestConnectionMs(durationMs);
	}
	maybeEmit24hEnergySummary(nullptr, "cloud", false);
}

void EnergyTrend::noteConnectionFailure() {
	ensureInitialized();
	current.set_energyCloudConnectionFailures(saturatingIncrement16(current.get_energyCloudConnectionFailures()));
}

void EnergyTrend::noteOccupancyTrigger() {
	ensureInitialized();
	current.set_energyOccupancyTriggerCount(saturatingIncrement16(current.get_energyOccupancyTriggerCount()));
}

void EnergyTrend::noteFaultReset() {
	ensureInitialized();
	current.set_energyFaultResetCount(saturatingIncrement16(current.get_energyFaultResetCount()));
	maybeEmit24hEnergySummary(nullptr, "fault", true);
}

void EnergyTrend::closeAwakeCycle() {
	if (!awakeCycleOpen_) {
		return;
	}
	uint32_t awakeDurationMs = millis() - awakeStartedAtMs_;
	current.set_energyAwakeMs(saturatingAdd32(current.get_energyAwakeMs(), awakeDurationMs));
	if (awakeDurationMs > current.get_energyLongestAwakeMs()) {
		current.set_energyLongestAwakeMs(awakeDurationMs);
	}
	awakeCycleOpen_ = false;
}

void EnergyTrend::noteSleepEntry() {
	ensureInitialized();
	closeAwakeCycle();
}

void EnergyTrend::noteResetImminent(const char *reason) {
	ensureInitialized();
	closeAwakeCycle();
	maybeEmit24hEnergySummary(nullptr, reason ? reason : "reset", true);
}

bool EnergyTrend::ensureBaseline(time_t sampleTime, float soc, float vcell) {
	if (hasUsableBaseline()) {
		return false;
	}
	resetWindow(sampleTime, soc, vcell);
	return true;
}

void EnergyTrend::resetWindow(time_t sampleTime, float soc, float vcell) {
	current.set_energyBaselineTimestamp(sampleTime);
	current.set_energyBaselineSoc(soc);
	current.set_energyBaselineVcell(vcell);
	current.set_energyWakeCount(0);
	current.set_energyConnectionCount(0);
	current.set_energyConnectionMs(0);
	current.set_energyAwakeMs(0);
	current.set_energyFaultResetCount(0);
	current.set_energyChargeFaultCount(0);
	current.set_energyCloudConnectionFailures(0);
	current.set_energyOccupancyTriggerCount(0);
	current.set_energyMinSoc(soc);
	current.set_energyMinVcell(vcell);
	current.set_energyLongestAwakeMs(0);
	current.set_energyLongestConnectionMs(0);
	if (awakeCycleOpen_) {
		awakeStartedAtMs_ = millis();
	}
}

void EnergyTrend::updateMinima(float soc, float vcell) {
	if (!isnan(soc)) {
		float minimumSoc = current.get_energyMinSoc();
		if (isnan(minimumSoc) || soc < minimumSoc) {
			current.set_energyMinSoc(soc);
		}
	}
	if (!isnan(vcell)) {
		float minimumVcell = current.get_energyMinVcell();
		if (isnan(minimumVcell) || vcell < minimumVcell) {
			current.set_energyMinVcell(vcell);
		}
	}
}

bool EnergyTrend::hasUsableBaseline() const {
	return current.get_energyBaselineTimestamp() > 0 &&
		!isnan(current.get_energyBaselineSoc()) &&
		!isnan(current.get_energyBaselineVcell());
}

bool EnergyTrend::isLowPowerMode(LowPowerMode mode) const {
	return mode == LowPowerMode::Critical || mode == LowPowerMode::Survival;
}

float EnergyTrend::currentSocValue(const PowerReport *report) const {
	if (report && !isnan(report->reading.soc)) {
		return report->reading.soc;
	}
	double persistedSoc = current.get_stateOfCharge();
	return (persistedSoc == persistedSoc) ? (float)persistedSoc : NAN;
}

float EnergyTrend::currentVcellValue(const PowerReport *report) const {
	if (report && !isnan(report->reading.batteryVoltage)) {
		return report->reading.batteryVoltage;
	}
	return current.get_batteryVoltage();
}

time_t EnergyTrend::currentSampleTime() const {
	time_t sampleTime = current.get_lastSampleTime();
	if (sampleTime > 0) {
		return sampleTime;
	}
	return Time.isValid() ? Time.now() : 0;
}

EnergyTrendClassification EnergyTrend::classify(float deltaSoc) const {
	if (deltaSoc < ENERGY_TREND_CRITICAL_SOC_DELTA) {
		return EnergyTrendClassification::Critical;
	}
	if (deltaSoc < ENERGY_TREND_NEGATIVE_SOC_DELTA) {
		return EnergyTrendClassification::Negative;
	}
	if (deltaSoc > ENERGY_TREND_POSITIVE_SOC_DELTA) {
		return EnergyTrendClassification::Positive;
	}
	return EnergyTrendClassification::Stable;
}

const char *EnergyTrend::classificationLabel(EnergyTrendClassification classification) {
	switch (classification) {
	case EnergyTrendClassification::Positive:
		return "POSITIVE";
	case EnergyTrendClassification::Stable:
		return "STABLE";
	case EnergyTrendClassification::Negative:
		return "NEGATIVE";
	case EnergyTrendClassification::Critical:
	default:
		return "CRITICAL";
	}
}

void EnergyTrend::noteMeasurement(const PowerReport &report) {
	ensureInitialized();
	float soc = currentSocValue(&report);
	float vcell = currentVcellValue(&report);
	time_t sampleTime = currentSampleTime();
	current.set_batteryVoltage(vcell);
	bool baselineCreated = ensureBaseline(sampleTime, soc, vcell);
	updateMinima(soc, vcell);

	if (report.reading.batteryContext == PowerBatteryContext::Fault && (!haveLastBatteryContext_ || lastBatteryContext_ != PowerBatteryContext::Fault)) {
		current.set_energyChargeFaultCount(saturatingIncrement16(current.get_energyChargeFaultCount()));
	}

	bool nowLowPowerMode = isLowPowerMode(report.policy.mode);
	if (baselineCreated) {
		lowPowerModeActive_ = nowLowPowerMode;
		lastBatteryContext_ = report.reading.batteryContext;
		haveLastBatteryContext_ = true;
		return;
	}

	if (nowLowPowerMode && !lowPowerModeActive_) {
		maybeEmit24hEnergySummary(&report, "low", true);
	}
	else {
		maybeEmit24hEnergySummary(&report, nullptr, false);
	}

	lowPowerModeActive_ = nowLowPowerMode;
	lastBatteryContext_ = report.reading.batteryContext;
	haveLastBatteryContext_ = true;
}

void EnergyTrend::maybeEmit24hEnergySummary(const PowerReport *report, const char *reason, bool force) {
	ensureInitialized();
	float soc = currentSocValue(report);
	float vcell = currentVcellValue(report);
	time_t sampleTime = currentSampleTime();
	if (ensureBaseline(sampleTime, soc, vcell)) {
		return;
	}
	if (!hasUsableBaseline()) {
		return;
	}

	time_t baselineTimestamp = current.get_energyBaselineTimestamp();
	time_t ageSeconds = (sampleTime > baselineTimestamp) ? (sampleTime - baselineTimestamp) : 0;
	bool overdue = ageSeconds >= ENERGY_TREND_INTERVAL_SECONDS;
	if (!force && !overdue) {
		return;
	}

	float baselineSoc = current.get_energyBaselineSoc();
	float baselineVcell = current.get_energyBaselineVcell();
	if (isnan(soc) || isnan(vcell) || isnan(baselineSoc) || isnan(baselineVcell)) {
		return;
	}

	uint32_t awakeMs = current.get_energyAwakeMs();
	if (awakeCycleOpen_) {
		uint32_t inFlightAwakeMs = millis() - awakeStartedAtMs_;
		awakeMs = saturatingAdd32(awakeMs, inFlightAwakeMs);
	}

	float deltaSoc = soc - baselineSoc;
	float deltaVcell = vcell - baselineVcell;
	EnergyTrendClassification trend = classify(deltaSoc);
	char ageBuffer[12];
	formatAge(ageBuffer, sizeof(ageBuffer), ageSeconds);
	const char *summaryReason = reason ? reason : "24h";
	Log.info(
		"Energy24h[%s]: age=%s dsoc=%.1f dv=%.3f wake=%u conn=%lum awake=%lum occ=%u fault=%u rst=%u fail=%u trend=%s",
		summaryReason,
		ageBuffer,
		deltaSoc,
		deltaVcell,
		current.get_energyWakeCount(),
		(unsigned long)roundedMinutes(current.get_energyConnectionMs()),
		(unsigned long)roundedMinutes(awakeMs),
		current.get_energyOccupancyTriggerCount(),
		current.get_energyChargeFaultCount(),
		current.get_energyFaultResetCount(),
		current.get_energyCloudConnectionFailures(),
		classificationLabel(trend));

	resetWindow(sampleTime, soc, vcell);
}
