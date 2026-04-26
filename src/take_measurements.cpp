
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

//Particle Functions
#include "Particle.h"
#include "take_measurements.h"
#include "device_pinout.h"
#include "MyPersistentData.h"
#include "power_management.h"

FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state 

bool hasExternalPowerSource();
void updatePowerManagementFromObservation();
void refreshBatteryTelemetry();

bool takeMeasurements() { 

    fuelGauge.quickStart();                           // Start the fuel gauge
    softDelay(1000);                                  // Give the fuel gauge time to start

    // Temperature inside the enclosure
    current.set_internalTempC((int)tmp36TemperatureC(analogRead(TMP36_SENSE_PIN)));

    refreshBatteryTelemetry();
	updatePowerManagementFromObservation();
    refreshBatteryTelemetry();
	Log.info("Battery State: %s, SOC: %2.0f%%",batteryContext[current.get_batteryState()],current.get_stateOfCharge());

    if (sysStatus.get_nodeNumber() == 0 ) getSignalStrength();

    return 1;

}


float tmp36TemperatureC (int adcValue) { 
    // Analog inputs have values from 0-4095, or
    // 12-bit precision. 0 = 0V, 4095 = 3.3V, 0.0008 volts (0.8 mV) per unit
    // The temperature sensor docs use millivolts (mV), so use 3300 as the factor instead of 3.3.
    float mV = ((float)adcValue) * 3300 / 4095;

    // According to the TMP36 docs:
    // Offset voltage 500 mV, scaling 10 mV/deg C, output voltage at 25C = 750 mV (77F)
    // The offset voltage is subtracted from the actual voltage, allowing negative temperatures
    // with positive voltages.

    // Example value=969 mV=780.7 tempC=28.06884765625 tempF=82.52392578125

    // With the TMP36, with the flat side facing you, the pins are:
    // Vcc | Analog Out | Ground
    // You must put a 0.1 uF capacitor between the analog output and ground or you'll get crazy
    // inaccurate values!
    return (mV - 500) / 10;
}
void refreshBatteryTelemetry() {
	current.set_stateOfCharge(System.batteryCharge());
	current.set_batteryState(System.batteryState());
}


void updatePowerManagementFromObservation() {
  PowerObservation obs = {};
  int batteryStateCode = current.get_batteryState();

  obs.batterySoc = current.get_stateOfCharge();
  obs.batteryVoltage = fuelGauge.getVCell();
  obs.temperatureF = (current.get_internalTempC() * 9.0f / 5.0f) + 32.0f;
  obs.inputPowerPresent = hasExternalPowerSource();
  obs.batteryIsCharging = batteryStateCode == 2;
  obs.batteryNotCharging = batteryStateCode == 1;
  obs.batteryFault = batteryStateCode == 5;

  updatePowerManagementObservation(obs);
}


void getSignalStrength() {
  char signalStr[16];
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(signalStr,sizeof(signalStr), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
  Log.info(signalStr);
}


bool recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the device
{
  pinSetFast(BLUE_LED);                                                                               // Turn on the blue LED

  current.set_lastCountTime(Time.now());
  current.set_hourlyCount(current.get_hourlyCount() +1);                                              // Increment the PersonCount
  current.set_dailyCount(current.get_dailyCount() +1);                                               // Increment the PersonCount
  Log.info("Count, hourly: %i. daily: %i",current.get_hourlyCount(),current.get_dailyCount());
  
  delay(500);
  pinResetFast(BLUE_LED);

  return true;
}

/**
 * @brief soft delay let's us process Particle functions and service the sensor interrupts while pausing
 * 
 * @details takes a single unsigned long input in millis
 * 
 */
void softDelay(uint32_t t) {
  for (uint32_t ms = millis(); millis() - ms < t; Particle.process());  //  safer than a delay()
}

bool hasExternalPowerSource() {
  int powerSource = System.powerSource();
  return powerSource == POWER_SOURCE_VIN || powerSource == POWER_SOURCE_USB_HOST;
}