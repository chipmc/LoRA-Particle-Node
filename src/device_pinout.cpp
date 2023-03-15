//Particle Functions
#include "Particle.h"
#include "device_pinout.h"

/**********************************************************************************************************************
 * ******************************         Boron Pinout Example               ******************************************
 * https://docs.particle.io/reference/datasheets/b-series/boron-datasheet/#pins-and-button-definitions
 *
 Left Side (16 pins)
 * !RESET -
 * 3.3V -
 * !MODE -
 * GND -
 * D19 - A0 -               
 * D18 - A1 -               INT_PIN - PIR sensor interrupt pin       
 * D17 - A2 -               MODULE_POWER_PIN - Bringing this low powers up the PIR sensor          
 * D16 - A3 -               LED_POWER_PIN - Enables or disables the LED on the PIR module
 * D15 - A4 -               Internal (TMP32) Temp Sensor
 * D14 - A5 / SPI SS -      
 * D13 - SCK - SPI Clock -  RFM9x SPI
 * D12 - MO - SPI MOSI -    RFM9x SPI
 * D11 - MI - SPI MISO -    RFM9x SPI
 * D10 - UART RX -
 * D9 - UART TX -

 Right Size (12 pins)
 * Li+
 * ENABLE
 * VUSB -
 * D8 -                     Wake Connected to Watchdog Timer
 * D7 -                     Blue Led
 * D6 -                     RFM9x - Reset Pin
 * D5 -                     RFM9x - Chip Select
 * D4 -                     User Switch
 * D3 - 
 * D2 -                     RFM9x Interrupt Pin
 * D1 - SCL - I2C Clock -   FRAM / RTC and I2C Bus
 * D0 - SDA - I2C Data -    FRAM / RTX and I2C Bus
***********************************************************************************************************************/
//Define pins for the RFM9x on my Particle carrier board
const pin_t RFM95_CS =      D5;                     // SPI Chip select pin - Standard SPI pins otherwise was A5
const pin_t RFM95_RST =     D6;                     // Radio module reset was D3
const pin_t RFM95_INT =     D2;                     // Interrupt from radio

// Carrier Board standard pins
const pin_t TMP36_SENSE_PIN   = A4;
const pin_t BUTTON_PIN        = D4;
const pin_t BLUE_LED          = D7;
const pin_t WAKEUP_PIN        = D8;

// Sensor specific Pins
extern const pin_t INT_PIN = A1;                   // May need to change this
extern const pin_t MODULE_POWER_PIN = A2;          // Make sure we document this above
const pin_t LED_POWER_PIN = A3;

bool initializePinModes() {
    Log.info("Initalizing the pinModes");
    // Define as inputs or outputs
    pinMode(BUTTON_PIN,INPUT);               // User button on the carrier board - active LOW
    pinMode(WAKEUP_PIN,INPUT);                      // This pin is active HIGH
    pinMode(BLUE_LED,OUTPUT);                       // On the Boron itself
    pinMode(INT_PIN, INPUT);
    pinMode(MODULE_POWER_PIN, OUTPUT);
    pinMode(LED_POWER_PIN,OUTPUT);
    return true;
}

void sensorControl(int sensorType, bool enableSensor) { // What is the sensor type - 0-Pressure Sensor, 1-PIR Sensor

  if (enableSensor) {
    digitalWrite(MODULE_POWER_PIN,false);           // Enable or disable the sensor

    if (sensorType == 0) {                          // This is the pressure sensor and we are enabling it
        digitalWrite(LED_POWER_PIN,HIGH);        // For the pressure sensor, this is how you activate it
    }
    else {
        digitalWrite(LED_POWER_PIN,LOW);         // Turns on the LED on the PIR sensor board
    }
  }

  else {
    digitalWrite(MODULE_POWER_PIN,true);

    if (sensorType == 0) {                          // This is the pressure sensor and we are enabling it
        digitalWrite(LED_POWER_PIN,LOW);         // Turns off the LED on the pressure sensor board
    }
    else {
        digitalWrite(LED_POWER_PIN,HIGH);        // Turns off the LED on the PIR sensor board
    }
  }

}


bool initializePowerCfg() {
    Log.info("Initializing Power Config");
    const int maxCurrentFromPanel = 900;            // Not currently used (100,150,500,900,1200,2000 - will pick closest) (550mA for 3.5W Panel, 340 for 2W panel)
    SystemPowerConfiguration conf;
    System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration

    conf.powerSourceMaxCurrent(maxCurrentFromPanel) // Set maximum current the power source can provide  3.5W Panel (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(maxCurrentFromPanel) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
}
                
