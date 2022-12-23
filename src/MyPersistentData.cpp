#include "Particle.h"
#include "MB85RC256V-FRAM-RK.h"
#include "StorageHelperRK.h"
#include "MyPersistentData.h"

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
    sysStatus.load();
}

void sysStatusData::loop() {
    sysStatus.flush(true);
}

/**
 * @brief This function is called in setup if the version of the FRAM stoage map has been changed
 * 
 */
void sysStatusData::loadSystemDefaults() {                         // This code is only executed with a new device or a new storage object structure
  Log.info("Loading system defaults");              // Letting us know that defaults are being loaded
  sysStatus.set_nodeNumber(11);
  sysStatus.set_structuresVersion(1);
  sysStatus.set_magicNumber(27617);
  // sysStatus.set_firmwareRelease(1);
  sysStatus.set_resetCount(0);
  sysStatus.set_frequencyMinutes(5);
  sysStatus.set_alertCodeNode(1);
  sysStatus.set_alertTimestampNode(0);
  sysStatus.set_openHours(true);
}

void sysStatusData::checkSystemValues() {               // Values out of bounds indicates an initialization error - will reload defaults
    bool reset = false;

    Log.info("freq = %d, type = %d, node = %d, current %4.2f",sysStatus.get_frequencyMinutes(), sysStatus.get_sensorType(),sysStatus.get_nodeNumber(), current.get_stateOfCharge() );
 
    if (sysStatus.get_frequencyMinutes() <=0 || sysStatus.get_frequencyMinutes() > 60) reset = true;
    if (sysStatus.get_sensorType() < 0 || sysStatus.get_sensorType() >2) reset = true;
    if (sysStatus.get_nodeNumber() > 11) reset = true;

    if (reset) sysStatusData::loadSystemDefaults();
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
// Offset of 50 bytes - make room for SysStatus
// ********************************************************************

currentStatusData *currentStatusData::_instance;

// [static]
currentStatusData &currentStatusData::instance() {
    if (!_instance) {
        _instance = new currentStatusData();
    }
    return *_instance;
}

currentStatusData::currentStatusData() : StorageHelperRK::PersistentDataFRAM(::fram, 50, &currentData.currentHeader, sizeof(CurrentData), CURRENT_DATA_MAGIC, CURRENT_DATA_VERSION) {
};

currentStatusData::~currentStatusData() {
}

void currentStatusData::setup() {
    fram.begin();
    current.load();
}

void currentStatusData::loop() {
    current.flush(true);
}

void currentStatusData::loadCurrentDefaults() {                         // This code is only executed with a new device or a new storage object structure
  Log.info("Loading current defaults");                                 // Letting us know that defaults are being loaded
}

void currentStatusData::resetEverything() {                             // The device is waking up in a new day or is a new install
  current.set_dailyCount(0);                                            // Reset the counts in FRAM as well
  current.set_hourlyCount(0);
  current.set_lastCountTime(Time.now());
  sysStatus.set_resetCount(0);                                          // Reset the reset count as well
  current.set_messageCount(0);
  current.set_successCount(0);
}


uint8_t currentStatusData::get_internalTempC() const {
    return getValue<uint8_t>(offsetof(CurrentData, internalTempC));
}

void currentStatusData::set_internalTempC(uint8_t value) {
    setValue<uint8_t>(offsetof(CurrentData, internalTempC), value);
}

double currentStatusData::get_stateOfCharge() const {
    return getValue<double>(offsetof(CurrentData, stateOfCharge));
}

void currentStatusData::set_stateOfCharge(double value) {
    setValue<double>(offsetof(CurrentData, stateOfCharge), value);
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

uint16_t currentStatusData::get_RSSI() const {
    return getValue<uint16_t>(offsetof(CurrentData, RSSI));
}

void currentStatusData::set_RSSI(uint16_t value) {
    setValue<uint16_t>(offsetof(CurrentData, RSSI), value);
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


