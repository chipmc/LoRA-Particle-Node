/*
 * Project LoRA-Particle-Node - basic example of a remote counter
 * Description: Will send counts each our to server to be relayed to Particle / Ubidots
 * Author: Chip McClelland
 * Date: 7-25-22
 */

// Version list 
// v0.01 - initial attempt - no sleep
// v0.02 - Adding sleep and scheduling using localTimeRK


// Included libraries
#include <RHMesh.h>
#include <RH_RF95.h>								// https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "LocalTimeRK.h"					        // https://rickkas7.github.io/LocalTimeRK/
#include "Particle.h"

#include "sys_status.h"

struct systemStatus_structure sysStatus;

const uint8_t firmwareMajor = 0;
const char firmwareMinor = 0.02;
const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

// System Mode Calls
SYSTEM_MODE(MANUAL);								// For testing - allows us to monitor the device
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
retained uint8_t resetCount;


SerialLogHandler logHandler(LOG_LEVEL_INFO);		// For monitoring / debugging, you have some options on the next few lines

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void stayAwakeTimerISR(void);                       // interrupt service routine for sleep control timer
int setLowPowerMode(String command);                // Enable sleep
void flashTheLEDs();                                // Indication that nodes are heard
void userSwitchISR();                               // interrupt service routime for the user switch
void sensorISR();
void recordCount();
void sendMessage();
int secondsUntilNextEvent(); 						// Time till next scheduled event

// Initialize Local Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
LocalTimeSchedule publishSchedule;					// These allow us to enable a schedule and to use local time
LocalTimeConvert localTimeConvert_NOW;

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

// Set up pins
const pin_t tmp36Pin =      A4;                     // Simple Analog temperature sensor - on the carrier board - inside the enclosure
const pin_t wakeUpPin =     D8;                     // This is the Particle Electron WKP pin
const pin_t blueLED =       D7;                     // This LED is on the Electron itself
const pin_t userSwitch =    D4;                     // User switch with a pull-up resistor
const pin_t intPin =		A0;						// PIR or Car counter
//Define pins for the RFM9x on my Particle carrier board
const pin_t RFM95_CS =      A5;                     // SPI Chip select pin - Standard SPI pins otherwise
const pin_t RFM95_RST =     D3;                     // Radio module reset
const pin_t RFM95_INT =     D2;                     // Interrupt from radio

//  ***************  LoRA Setup Section ********************
// In this implementation - we have one gateway and two nodes (will generalize later) - nodes are numbered 2 to ...
// 2- 9 for new nodes making a join request
// 10 - 255 nodes assigned to the mesh
#define GATEWAY_ADDRESS 1					        // The gateway for each mesh is #1
#define NODE2_ADDRESS 10					        // First Node
#define NODE3_ADDRESS 11					        // Second Node
#define RF95_FREQ 915.0								// Frequency - ISM

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE2_ADDRESS);
const uint16_t devID = 65534;

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#ifndef RH_MAX_MESSAGE_LEN
#define RH_MAX_MESSAGE_LEN 255
#endif

// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];               // Related to max message size 
//********************************************************

// Program variables
volatile bool sensorDetect = false;
volatile bool userSwitchDectected = false;		
bool systemStatusWriteNeeded = false;	
uint16_t hourly = 0;
uint16_t daily = 0;
uint8_t alerts = 0;
uint16_t nextReportSeconds = 10;					// Default send every 1 minute until configured by the gateway
system_tick_t lastReportMillis = 0;						// Ensures we send right away

void setup() 
{
	pinMode(blueLED,OUTPUT);						// Blue led signals sends
	pinMode(intPin, INPUT_PULLDOWN);				// Initialize sensor interrupt pin

	// Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  	if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    	resetCount++;
    	if (resetCount > 6) alerts = 13;            // Excessive resets
  	}

  	// Set up the Radio Module
	if (!manager.init()) Log.info("init failed"); // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
	
	driver.setFrequency(RF95_FREQ);					// Setup ISM frequency - typically 868.0 or 915.0 in the Americas, or 433.0 in the EU

	driver.setTxPower(23, false);                     // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm (13dBm default):

	// Watchdog Timer and Real Time Clock Initialization
  	ab1805.withFOUT(D8).setup();                    // The carrier board has D8 connected to FOUT for wake interrupts
  	ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);    // Enable watchdog

	// Next we will load FRAM and check or reset variables to their correct values
	fram.begin();                                                        // Initialize the FRAM module
	byte tempVersion;
	fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
	if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
		Log.info("Erasing FRAM");
		fram.erase();                                                      // Reset the FRAM to correct the issue
		fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
		fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
		if (tempVersion != FRAMversionNumber) {
		state = ERROR_STATE;                                             // Device will not work without FRAM will need to reset
		// resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
		// current.alerts = 12;                                             // FRAM is messed up so can't store but will be read in ERROR state
		}
		// else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
	}
	else {
		Log.info("Loading the sysStatus array");
		fram.get(FRAM::systemStatusAddr,sysStatus);                        // Loads the System Status array from FRAM
	}

	Log.info("Startup complete - battery %4.2f%%, clock is %s", System.batteryCharge(), (Time.isValid()) ? "valid" : "not valid");

	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	localTimeConvert_NOW.withCurrentTime().convert();  				        // Convert to local time for use later
	publishSchedule.withMinuteOfHour(sysStatus.frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("22:59:59")));	 // Publish every n minutes from 6am to 10pm

	// Attach the interrupt for the counter sensor and user button
	attachInterrupt(intPin, sensorISR, RISING);     // Pressure Sensor interrupt from low to high
	attachInterrupt(userSwitch,userSwitchISR,CHANGE); // We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;	// We got through setup without error
}


void loop()
{
	switch (state) {
		static system_tick_t lastAttempt = 0;
		case IDLE_STATE: {							// State we spend time in when there is nothing else to do
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			if (publishSchedule.isScheduledTime()) {	// Using the sendFrequency set above, we go to the reporing state - this only works if time is valid
				Log.info("Going to reporting based on schedule");
				sensorDetect = true;
				state = REPORTING_STATE;
			}

			if (!Time.isValid() && millis() - lastAttempt > 60000) {
				Log.info("Going to reporting on rescue mode");
				lastAttempt = millis();
				state = REPORTING_STATE;
			}
		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			int wakeInSeconds = secondsUntilNextEvent() - 10;   	   	       // Need to wake for the next event - the 10 seconds part ensures we are in IDLE when the next event occurs
			Log.info("Sleep for %i seconds", wakeInSeconds);
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(userSwitch,CHANGE)
				.duration(wakeInSeconds * 1000L);
			SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
			if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				// Will add functionality here
			}
			state = IDLE_STATE;
		} break;

		case REPORTING_STATE: {						// This is where we will send the message then go back to Idle
			int randomDelay = random(10);
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			Log.info("Adding a random delay of %i seconds", randomDelay);
			delay(randomDelay*1000);
			sendMessage();
			if (Time.isValid()) state = SLEEPING_STATE;							// We may be in rescue mode
			else state = IDLE_STATE;
		}
		break;
	}
	
	ab1805.loop();                                  // Keeps the RTC synchronized with the Boron's clock

	if (sensorDetect) recordCount();                // The ISR had raised the sensor flag - this will service interrupts regardless of state

	if (systemStatusWriteNeeded) {                                       // These flags get set when a value is changed
    	fram.put(FRAM::systemStatusAddr,sysStatus);
    	systemStatusWriteNeeded = false;
  	}
}

void sensorISR() {
	sensorDetect = true;
}

void userSwitchISR() {
  userSwitchDectected = true;                                            // The the flag for the user switch interrupt
}

void recordCount() {
	static time_t lastCount = Time.now();
	if (Time.now() - lastCount) {
		lastCount = Time.now();
		hourly++;
		daily++;
		sensorDetect = false;
	}
}

void sendMessage() {
	Log.info("Sending to Gateway");
	digitalWrite(blueLED,HIGH);

	const uint8_t temp = 85;
	uint8_t battChg = System.batteryCharge();
	uint8_t battState = System.batteryState();
    int16_t rssi = driver.lastRssi();
	static uint8_t msgCnt = 0;
	uint8_t payload[17];

	payload[0] = 0; 								// to be replaced/updated
	payload[1] = 0; 								// to be replaced/updated
	payload[2] = highByte(devID);					// Set for device
	payload[3] = lowByte(devID);
	payload[4] = firmwareMajor;						// Set for code release
	payload[5] = highByte(hourly);
	payload[6] = lowByte(hourly); 
	payload[7] = highByte(daily);
	payload[8] = lowByte(daily); 
	payload[9] = temp;
	payload[10] = battChg;
	payload[11] = battState;	
	payload[12] = resetCount;
	payload[13] = alerts;
	payload[14] = highByte(rssi);
	payload[15] = lowByte(rssi); 
	payload[16] = msgCnt++;

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	Log.info("sending message %d", payload[16]);
	if (manager.sendtoWait(payload, sizeof(payload), GATEWAY_ADDRESS) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server
		uint8_t len = sizeof(buf);
		uint8_t from;     
		Log.info("Message sent");
		if (manager.recvfromAckTimeout(buf, &len, 3000, &from)) {
			buf[len] = 0;
			char data[64];
			snprintf(data, sizeof(data),"Response: 0x%02x rssi=%d - delivery %s", from, driver.lastRssi(), (buf[2] == payload[16]) ? "successful":"unsuccessful");
			Log.info(data);
			uint32_t newTime = ((buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]);
			Log.info("Time is: %lu",newTime);
			Time.setTime(newTime);  // Set time based on response from gateway
			Log.info("Time set to %lu local time is %s", newTime, Time.timeStr(newTime).c_str());
			Log.info("Next report in %u minutes",((buf[7] << 8) | buf[8]));
			sysStatus.frequencyMinutes = ((buf[7] << 8) | buf[8]);
			publishSchedule.withMinuteOfHour(sysStatus.frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("22:59:59")));	 // Publish every as directed from 6am to 10pm
			systemStatusWriteNeeded = true;
 		}
		else {
			Log.info("No reply, are the gateways running?");
		}
	}
	else Log.info("sendtoWait failed. are the gateways running?");
	lastReportMillis = millis();
	digitalWrite(blueLED,LOW);
}

/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
	char stateTransitionString[40];
	snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	oldState = state;
	if (Particle.connected()) {
		static time_t lastPublish = 0;
		if (millis() - lastPublish > 1000) {
			lastPublish = millis();
			Particle.publish("State Transition",stateTransitionString, PRIVATE);
		}
	}
	Log.info(stateTransitionString);
}

/**
 * @brief Function to compute time to next scheduled event
 * 
 * @details - Computes seconds and returns 0 if no event is scheduled or time is invalid
 * 
 * 
 */
int secondsUntilNextEvent() {											// Time till next scheduled event
   if (Time.isValid()) {

        localTimeConvert_NOW.withCurrentTime().convert();
        Log.info("local time: %s", localTimeConvert_NOW.format(TIME_FORMAT_DEFAULT).c_str());

        LocalTimeConvert localTimeConvert_NEXT;
        localTimeConvert_NEXT.withCurrentTime().convert();

		if (publishSchedule.getNextScheduledTime(localTimeConvert_NEXT)) {
        	Log.info("time of next event is: %s which is %lu seconds away", localTimeConvert_NEXT.format(TIME_FORMAT_DEFAULT).c_str(), (long)(localTimeConvert_NEXT.time - localTimeConvert_NOW.time));
			return (long)(localTimeConvert_NEXT.time - localTimeConvert_NOW.time);
		}
		else return 0;
    }
	else return 0;
}