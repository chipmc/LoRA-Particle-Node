#include "Particle.h"
#include "MB85RC256V-FRAM-RK.h"
#include "StorageHelperRK.h"
#include "config.h"
#include "MyPersistentData.h"

#include <math.h>

MB85RC64 fram(Wire, 0);  

// *******************  SysStatus Storage Object **********************
//
// ********************************************************************

sysStatusData *sysStatusData::_instance;

// [static]
sysStatusData &sysStatusData::instance() {
    if (!_instance) {
        _instance = new sysStatusData();
    }
    return *_instance;
}

sysStatusData::sysStatusData() : StorageHelperRK::PersistentDataFRAM(::fram, 0, &sysData.sysHeader, sizeof(SysData), SYS_DATA_MAGIC, SYS_DATA_VERSION) {

};

sysStatusData::~sysStatusData() {
}

void sysStatusData::setup() {
    fram.begin();

    repairedFrequencyMinutes_ = false;

    sysStatus
    //    .withLogData(true)
        .withSaveDelayMs(100)
        .load();

    if (sysStatus.get_firmwareRelease() != NODE_FIRMWARE_RELEASE) {
        sysStatus.set_firmwareRelease(NODE_FIRMWARE_RELEASE);
    }

    if (repairedFrequencyMinutes_) {
        sysStatus.flush(true);
        repairedFrequencyMinutes_ = false;
    }

    Log.info(
        "sysStatus valid: node=%u fw=%u frequency=%u openHours=%d lastConnection=%lu",
        sysStatus.get_nodeNumber(),
        sysStatus.get_firmwareRelease(),
        sysStatus.get_frequencyMinutes(),
        sysStatus.get_openHours() ? 1 : 0,
        (unsigned long)sysStatus.get_lastConnection());

    // Log.info("sizeof(SysData): %u", sizeof(SysData));
}

void sysStatusData::loop() {
    sysStatus.flush(false);
}

bool sysStatusData::validate(size_t dataSize) {
    bool valid = PersistentDataFRAM::validate(dataSize);
    if (valid) {
        uint16_t frequencyMinutes = sysStatus.get_frequencyMinutes();
        if (frequencyMinutes < MIN_REPORT_FREQUENCY_MINUTES || frequencyMinutes > MAX_REPORT_FREQUENCY_MINUTES) {
            Log.info("sysStatus repaired frequencyMinutes from %u to 60", frequencyMinutes);
            sysStatus.set_frequencyMinutes(DEFAULT_REPORT_FREQUENCY_MINUTES);
            repairedFrequencyMinutes_ = true;
        }
        if (sysStatus.get_nodeNumber() < 1 || sysStatus.get_nodeNumber() > 11) {
            Log.info("data not valid node number =%d", sysStatus.get_nodeNumber());
            valid = false;
        }
    }
    Log.info("sysStatus data is %s",(valid) ? "valid": "not valid");
    return valid;
}

void sysStatusData::initialize() {
    PersistentDataFRAM::initialize();

    Log.info("data initialized");

    // Initialize the default value to 10 if the structure is reinitialized.
    // Be careful doing this, because when MyData is extended to add new fields,
    // the initialize method is not called! This is only called when first
    // initialized.
    Log.info("Loading system defaults");              // Letting us know that defaults are being loaded
    sysStatus.set_nodeNumber(11);
    sysStatus.set_structuresVersion(1);
    sysStatus.set_magicNumber(27617);
    sysStatus.set_firmwareRelease(NODE_FIRMWARE_RELEASE);
    sysStatus.set_resetCount(0);
    sysStatus.set_frequencyMinutes(DEFAULT_REPORT_FREQUENCY_MINUTES);
    sysStatus.set_alertCodeNode(1);
    sysStatus.set_alertTimestampNode(0);
    sysStatus.set_openHours(true);

    // If you manually update fields here, be sure to update the hash
    updateHash();
}

uint8_t sysStatusData::get_nodeNumber() const {
    return getValue<uint8_t>(offsetof(SysData, nodeNumber));
}

void sysStatusData::set_nodeNumber(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, nodeNumber), value);
}

uint8_t sysStatusData::get_structuresVersion() const {
    return getValue<uint8_t>(offsetof(SysData, structuresVersion));
}

void sysStatusData::set_structuresVersion(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, structuresVersion), value);
}

uint16_t sysStatusData::get_magicNumber() const {
    return getValue<uint16_t>(offsetof(SysData, magicNumber));
}

void sysStatusData::set_magicNumber(uint16_t value) {
    setValue<uint16_t>(offsetof(SysData, magicNumber), value);
}


uint8_t sysStatusData::get_firmwareRelease() const {
    return getValue<uint8_t>(offsetof(SysData, firmwareRelease));
}

void sysStatusData::set_firmwareRelease(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, firmwareRelease), value);
}

uint8_t sysStatusData::get_resetCount() const {
    return getValue<uint8_t>(offsetof(SysData, resetCount));
}

void sysStatusData::set_resetCount(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, resetCount), value);
}

time_t sysStatusData::get_lastConnection() const {
    return getValue<time_t>(offsetof(SysData, lastConnection));
}

void sysStatusData::set_lastConnection(time_t value) {
    setValue<time_t>(offsetof(SysData, lastConnection), value);
}

uint16_t sysStatusData::get_frequencyMinutes() const {
    return getValue<uint16_t>(offsetof(SysData,frequencyMinutes));
}

void sysStatusData::set_frequencyMinutes(uint16_t value) {
    setValue<uint16_t>(offsetof(SysData, frequencyMinutes), value);
}

uint8_t sysStatusData::get_alertCodeNode() const {
    return getValue<uint8_t>(offsetof(SysData, alertCodeNode));
}

void sysStatusData::set_alertCodeNode(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, alertCodeNode), value);
}

time_t sysStatusData::get_alertTimestampNode() const {
    return getValue<time_t>(offsetof(SysData, alertTimestampNode));
}

void sysStatusData::set_alertTimestampNode(time_t value) {
    setValue<time_t>(offsetof(SysData, alertTimestampNode), value);
}

uint8_t sysStatusData::get_sensorType() const {
    return getValue<uint8_t>(offsetof(SysData, sensorType));
}

void sysStatusData::set_sensorType(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, sensorType), value);
}

bool sysStatusData::get_openHours() const {
    return getValue<bool>(offsetof(SysData, openHours));
}

void sysStatusData::set_openHours(bool value) {
    setValue<bool>(offsetof(SysData, openHours), value);
}

// *****************  Current Status Storage Object *******************
// Offset of 100 bytes - make room for SysStatus
// ********************************************************************

currentStatusData *currentStatusData::_instance;

// [static]
currentStatusData &currentStatusData::instance() {
    if (!_instance) {
        _instance = new currentStatusData();
    }
    return *_instance;
}

currentStatusData::currentStatusData() : StorageHelperRK::PersistentDataFRAM(::fram, 100, &currentData.currentHeader, sizeof(CurrentData), CURRENT_DATA_MAGIC, CURRENT_DATA_VERSION) {
};

currentStatusData::~currentStatusData() {
}

void currentStatusData::setup() {
    fram.begin();

    current
    //    .withLogData(true)
        .withSaveDelayMs(250)
        .load();
}

void currentStatusData::loop() {
    current.flush(false);
}

void currentStatusData::resetEverything() {                             // The device is waking up in a new day or is a new install
  current.set_dailyCount(0);                                            // Reset the counts in FRAM as well
  current.set_hourlyCount(0);
  current.set_lastCountTime(Time.now());
  sysStatus.set_resetCount(0);                                          // Reset the reset count as well
  current.set_messageCount(0);
  current.set_successCount(0);
    current.set_batteryVoltage(NAN);
    current.set_energyBaselineTimestamp(0);
    current.set_energyBaselineSoc(NAN);
    current.set_energyBaselineVcell(NAN);
    current.set_energyWakeCount(0);
    current.set_energyConnectionCount(0);
    current.set_energyConnectionMs(0);
    current.set_energyAwakeMs(0);
    current.set_energyFaultResetCount(0);
    current.set_energyChargeFaultCount(0);
    current.set_energyCloudConnectionFailures(0);
    current.set_energyOccupancyTriggerCount(0);
    current.set_energyMinSoc(NAN);
    current.set_energyMinVcell(NAN);
    current.set_energyLongestAwakeMs(0);
    current.set_energyLongestConnectionMs(0);
}

bool currentStatusData::validate(size_t dataSize) {
    bool valid = PersistentDataFRAM::validate(dataSize);
    if (valid) {
        if (current.get_hourlyCount() < 0 || current.get_hourlyCount()  > 1024) {
            Log.info("current data not valid hourlyCount=%d" , current.get_hourlyCount());
            valid = false;
        }
    }
    Log.info("current data is %s",(valid) ? "valid": "not valid");
    return valid;
}

void currentStatusData::initialize() {
    PersistentDataFRAM::initialize();

    Log.info("Current Data Initialized");

    currentStatusData::resetEverything();

    // If you manually update fields here, be sure to update the hash
    updateHash();
}


int8_t currentStatusData::get_internalTempC() const {
    return getValue<int8_t>(offsetof(CurrentData, internalTempC));
}

void currentStatusData::set_internalTempC(int8_t value) {
    setValue<int8_t>(offsetof(CurrentData, internalTempC), value);
}

double currentStatusData::get_stateOfCharge() const {
    return getValue<double>(offsetof(CurrentData, stateOfCharge));
}

void currentStatusData::set_stateOfCharge(double value) {
    setValue<double>(offsetof(CurrentData, stateOfCharge), value);
}

float currentStatusData::get_batteryVoltage() const {
	return getValue<float>(offsetof(CurrentData, batteryVoltage));
}

void currentStatusData::set_batteryVoltage(float value) {
	setValue<float>(offsetof(CurrentData, batteryVoltage), value);
}

uint8_t currentStatusData::get_batteryState() const {
    return getValue<uint8_t>(offsetof(CurrentData, batteryState));
}

void currentStatusData::set_batteryState(uint8_t value) {
    setValue<uint8_t>(offsetof(CurrentData, batteryState), value);
}

time_t currentStatusData::get_lastSampleTime() const {
    return getValue<time_t>(offsetof(CurrentData, lastSampleTime));
}

void currentStatusData::set_lastSampleTime(time_t value) {
    setValue<time_t>(offsetof(CurrentData, lastSampleTime), value);
}

int16_t currentStatusData::get_RSSI() const {
    return getValue<int16_t>(offsetof(CurrentData, RSSI));
}

void currentStatusData::set_RSSI(int16_t value) {
    setValue<int16_t>(offsetof(CurrentData, RSSI), value);
}

int16_t currentStatusData::get_SNR() const {
    return getValue<int16_t>(offsetof(CurrentData, SNR));
}

void currentStatusData::set_SNR(int16_t value) {
    setValue<int16_t>(offsetof(CurrentData, SNR), value);
}

uint8_t currentStatusData::get_messageCount() const {
    return getValue<uint8_t>(offsetof(CurrentData, messageCount));
}

void currentStatusData::set_messageCount(uint8_t value) {
    setValue<uint8_t>(offsetof(CurrentData, messageCount), value);
}

uint8_t currentStatusData::get_successCount() const {
    return getValue<uint8_t>(offsetof(CurrentData, successCount));
}

void currentStatusData::set_successCount(uint8_t value) {
    setValue<uint8_t>(offsetof(CurrentData, successCount), value);
}

time_t currentStatusData::get_lastCountTime() const {
    return getValue<time_t>(offsetof(CurrentData, lastCountTime));
}

void currentStatusData::set_lastCountTime(time_t value) {
    setValue<time_t>(offsetof(CurrentData, lastCountTime), value);
}

uint16_t currentStatusData::get_hourlyCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, hourlyCount));
}

void currentStatusData::set_hourlyCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, hourlyCount), value);
}

uint16_t currentStatusData::get_dailyCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, dailyCount));
}

void currentStatusData::set_dailyCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, dailyCount), value);
}

time_t currentStatusData::get_energyBaselineTimestamp() const {
    return getValue<time_t>(offsetof(CurrentData, energyBaselineTimestamp));
}

void currentStatusData::set_energyBaselineTimestamp(time_t value) {
    setValue<time_t>(offsetof(CurrentData, energyBaselineTimestamp), value);
}

float currentStatusData::get_energyBaselineSoc() const {
    return getValue<float>(offsetof(CurrentData, energyBaselineSoc));
}

void currentStatusData::set_energyBaselineSoc(float value) {
    setValue<float>(offsetof(CurrentData, energyBaselineSoc), value);
}

float currentStatusData::get_energyBaselineVcell() const {
    return getValue<float>(offsetof(CurrentData, energyBaselineVcell));
}

void currentStatusData::set_energyBaselineVcell(float value) {
    setValue<float>(offsetof(CurrentData, energyBaselineVcell), value);
}

uint16_t currentStatusData::get_energyWakeCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyWakeCount));
}

void currentStatusData::set_energyWakeCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyWakeCount), value);
}

uint16_t currentStatusData::get_energyConnectionCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyConnectionCount));
}

void currentStatusData::set_energyConnectionCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyConnectionCount), value);
}

uint32_t currentStatusData::get_energyConnectionMs() const {
    return getValue<uint32_t>(offsetof(CurrentData, energyConnectionMs));
}

void currentStatusData::set_energyConnectionMs(uint32_t value) {
    setValue<uint32_t>(offsetof(CurrentData, energyConnectionMs), value);
}

uint32_t currentStatusData::get_energyAwakeMs() const {
    return getValue<uint32_t>(offsetof(CurrentData, energyAwakeMs));
}

void currentStatusData::set_energyAwakeMs(uint32_t value) {
    setValue<uint32_t>(offsetof(CurrentData, energyAwakeMs), value);
}

uint16_t currentStatusData::get_energyFaultResetCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyFaultResetCount));
}

void currentStatusData::set_energyFaultResetCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyFaultResetCount), value);
}

uint16_t currentStatusData::get_energyChargeFaultCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyChargeFaultCount));
}

void currentStatusData::set_energyChargeFaultCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyChargeFaultCount), value);
}

uint16_t currentStatusData::get_energyCloudConnectionFailures() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyCloudConnectionFailures));
}

void currentStatusData::set_energyCloudConnectionFailures(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyCloudConnectionFailures), value);
}

uint16_t currentStatusData::get_energyOccupancyTriggerCount() const {
    return getValue<uint16_t>(offsetof(CurrentData, energyOccupancyTriggerCount));
}

void currentStatusData::set_energyOccupancyTriggerCount(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, energyOccupancyTriggerCount), value);
}

float currentStatusData::get_energyMinSoc() const {
    return getValue<float>(offsetof(CurrentData, energyMinSoc));
}

void currentStatusData::set_energyMinSoc(float value) {
    setValue<float>(offsetof(CurrentData, energyMinSoc), value);
}

float currentStatusData::get_energyMinVcell() const {
    return getValue<float>(offsetof(CurrentData, energyMinVcell));
}

void currentStatusData::set_energyMinVcell(float value) {
    setValue<float>(offsetof(CurrentData, energyMinVcell), value);
}

uint32_t currentStatusData::get_energyLongestAwakeMs() const {
    return getValue<uint32_t>(offsetof(CurrentData, energyLongestAwakeMs));
}

void currentStatusData::set_energyLongestAwakeMs(uint32_t value) {
    setValue<uint32_t>(offsetof(CurrentData, energyLongestAwakeMs), value);
}

uint32_t currentStatusData::get_energyLongestConnectionMs() const {
    return getValue<uint32_t>(offsetof(CurrentData, energyLongestConnectionMs));
}

void currentStatusData::set_energyLongestConnectionMs(uint32_t value) {
    setValue<uint32_t>(offsetof(CurrentData, energyLongestConnectionMs), value);
}

