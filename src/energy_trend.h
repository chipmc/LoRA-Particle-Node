#ifndef ENERGY_TREND_H
#define ENERGY_TREND_H

#include "Particle.h"
#include "power_management.h"

constexpr time_t ENERGY_TREND_INTERVAL_SECONDS = 24 * 60 * 60;
constexpr float ENERGY_TREND_POSITIVE_SOC_DELTA = 1.0f;
constexpr float ENERGY_TREND_NEGATIVE_SOC_DELTA = -1.0f;
constexpr float ENERGY_TREND_CRITICAL_SOC_DELTA = -5.0f;

enum class EnergyTrendClassification : uint8_t {
	Positive,
	Stable,
	Negative,
	Critical,
};

class EnergyTrend {
public:
	static EnergyTrend &instance();

	void setup();
	void noteWake();
	void noteConnection(uint32_t durationMs);
	void noteConnectionFailure();
	void noteOccupancyTrigger();
	void noteFaultReset();
	void noteSleepEntry();
	void noteResetImminent(const char *reason = "reset");
	void noteMeasurement(const PowerReport &report);
	void maybeEmit24hEnergySummary(const PowerReport *report = nullptr, const char *reason = nullptr, bool force = false);

	static const char *classificationLabel(EnergyTrendClassification classification);

private:
	EnergyTrend() = default;

	void ensureInitialized();
	bool ensureBaseline(time_t sampleTime, float soc, float vcell);
	void resetWindow(time_t sampleTime, float soc, float vcell);
	void updateMinima(float soc, float vcell);
	void closeAwakeCycle();
	bool hasUsableBaseline() const;
	bool isLowPowerMode(LowPowerMode mode) const;
	float currentSocValue(const PowerReport *report) const;
	float currentVcellValue(const PowerReport *report) const;
	time_t currentSampleTime() const;
	EnergyTrendClassification classify(float deltaSoc) const;

	bool initialized_ = false;
	bool awakeCycleOpen_ = false;
	bool lowPowerModeActive_ = false;
	uint32_t awakeStartedAtMs_ = 0;
	bool haveLastBatteryContext_ = false;
	PowerBatteryContext lastBatteryContext_ = PowerBatteryContext::Unknown;
};

#endif
