#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "Particle.h"
#include "config.h"

enum class PowerAvailability : uint8_t {
	Valid,
	Unknown,
	NotAvailable,
	Fallback,
};

enum class PowerBatteryContext : uint8_t {
	Unknown = 0,
	NotCharging = 1,
	Charging = 2,
	Charged = 3,
	Discharging = 4,
	Fault = 5,
	Disconnected = 6,
	NotApplicable = 7,
};

enum class LowPowerMode : uint8_t {
	Normal,
	Conserve,
	Critical,
	Survival,
};

enum class ChargingCommand : uint8_t {
	NoAction,
	Enable,
	Disable,
	NotAvailable,
};

enum class PowerInputProfile : uint8_t {
	UsbBench,
	Solar35W,
	Auto,
	NotApplicable,
};

#if POWER_MANAGER_INPUT_PROFILE == POWER_MANAGER_INPUT_PROFILE_USB_BENCH
constexpr PowerInputProfile POWER_MANAGER_DEFAULT_INPUT_PROFILE = PowerInputProfile::UsbBench;
#elif POWER_MANAGER_INPUT_PROFILE == POWER_MANAGER_INPUT_PROFILE_SOLAR35W
constexpr PowerInputProfile POWER_MANAGER_DEFAULT_INPUT_PROFILE = PowerInputProfile::Solar35W;
#elif POWER_MANAGER_INPUT_PROFILE == POWER_MANAGER_INPUT_PROFILE_AUTO
constexpr PowerInputProfile POWER_MANAGER_DEFAULT_INPUT_PROFILE = PowerInputProfile::Auto;
#else
#error "Unsupported POWER_MANAGER_INPUT_PROFILE selection"
#endif

#if POWER_MANAGER_INPUT_PROFILE == POWER_MANAGER_INPUT_PROFILE_SOLAR35W
constexpr PowerInputProfile POWER_MANAGER_CONFIGURED_FALLBACK_INPUT_PROFILE = PowerInputProfile::Solar35W;
#else
constexpr PowerInputProfile POWER_MANAGER_CONFIGURED_FALLBACK_INPUT_PROFILE = PowerInputProfile::UsbBench;
#endif

struct PowerCapabilities {
	bool hasSoc = false;
	bool hasBatteryVoltage = false;
	bool hasBatteryContext = false;
	bool hasChargingControl = false;
	bool hasPmicPowerConfiguration = false;
	bool hasPmicRemediation = false;
};

struct PowerReading {
	float soc = NAN;
	PowerAvailability socStatus = PowerAvailability::Unknown;
	float batteryVoltage = NAN;
	PowerAvailability batteryVoltageStatus = PowerAvailability::Unknown;
	PowerBatteryContext batteryContext = PowerBatteryContext::Unknown;
	PowerAvailability batteryContextStatus = PowerAvailability::Unknown;
	bool quickStartUsed = false;
	bool usedSocFallback = false;
	bool usedVoltageFallback = false;
	uint8_t stabilizationAttempts = 0;
};

struct PowerPolicy {
	LowPowerMode mode = LowPowerMode::Survival;
	bool shouldAllowDiscoveryMode = false;
	bool shouldAllowRecoveryPowerdown = false;
	bool shouldUseReducedRetry = true;
	bool shouldUseLongSleep = true;
};

struct PowerAction {
	ChargingCommand chargingCommand = ChargingCommand::NoAction;
	PowerAvailability chargingControlStatus = PowerAvailability::NotAvailable;
	PowerAvailability pmicRemediationStatus = PowerAvailability::NotAvailable;
};

struct PowerReport {
	PowerCapabilities capabilities;
	PowerReading reading;
	PowerPolicy policy;
	PowerAction action;
	PowerInputProfile activeInputProfile = PowerInputProfile::NotApplicable;
};

struct PowerManagerConfig {
	float minSafeChargeTempC = 0.0f;
	float maxSafeChargeTempC = 37.0f;
	float minSocForDiscoveryMode = 30.0f;
	float minSocForRecoveryPowerdown = 40.0f;
	float fullBatteryVoltageValidSoc100 = 4.15f;
	float normalMinSoc = 50.0f;
	float conserveMinSoc = 35.0f;
	float criticalMinSoc = 20.0f;
	uint8_t maxStabilizationAttempts = 3;
	uint16_t stabilizationDelayMs[3] = {100, 300, 600};
	uint16_t stabilizationBudgetMs = 1000;
	PowerInputProfile inputProfile = POWER_MANAGER_DEFAULT_INPUT_PROFILE;
};

class PowerManager {
public:
	static PowerManager &instance();

	bool setup(const PowerManagerConfig &config = PowerManagerConfig());
	void beginWakeCycle(bool wokeFromSleep = true);
	const PowerReport &sample(float enclosureTempC);
	bool applyRecommendedAction();
	const PowerReport &lastReport() const;
	const PowerPolicy &lastPolicy() const;

	static uint8_t encodeBatteryContext(PowerBatteryContext context);
	static const char *batteryContextLabel(PowerBatteryContext context);
	static const char *batteryContextLabel(uint8_t encodedContext);
	static const char *availabilityLabel(PowerAvailability availability);
	static const char *powerInputProfileLabel(PowerInputProfile profile);

private:
	PowerManager() = default;

	PowerCapabilities detectCapabilities() const;
	PowerReading collectReading(float enclosureTempC);
	PowerReading collectRawReading(float enclosureTempC);
	PowerPolicy buildPolicy(const PowerReading &reading) const;
	PowerAction buildAction(const PowerReading &reading, float enclosureTempC) const;
	bool shouldStabilize(const PowerReading &reading) const;
	bool isSocSuspicious(const PowerReading &reading, bool logIt = false) const;
	float estimateConservativeFallbackSocFromVoltage(float batteryVoltage) const;
	bool applyBasePowerProfile();
	PowerInputProfile configuredFallbackInputProfile() const;
	PowerInputProfile resolvePowerInputProfile(int &powerSource, const char *&reason) const;
	bool setChargingEnabled(bool enableCharging);
	void updateLastKnownGood(const PowerReading &reading);
	void applyFallback(PowerReading &reading) const;

	PowerManagerConfig config_;
	PowerReport lastReport_;
	ChargingCommand lastAppliedChargingCommand_ = ChargingCommand::NoAction;
	uint32_t wakeCycleStartedAtMs_ = 0;
	bool wokeFromSleep_ = false;
	bool configured_ = false;
	float lastKnownGoodSoc_ = NAN;
	float lastKnownGoodVoltage_ = NAN;
	PowerBatteryContext lastKnownGoodContext_ = PowerBatteryContext::Unknown;
	PowerInputProfile activeInputProfile_ = PowerInputProfile::NotApplicable;
	PowerInputProfile lastAppliedInputProfile_ = PowerInputProfile::NotApplicable;
	bool pmicProfileAppliedThisBoot_ = false;
};

#endif