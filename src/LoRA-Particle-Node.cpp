/*
 * Project LoRA-Particle-Node - basic example of a remote counter
 * Description: Will send counts each our to server to be relayed to Particle / Ubidots
 * Author: Chip McClelland
 * Date: 7-25-22
 */

// Version list 
// v0.01 - initial attempt - no sleep
// v0.02 - Adding sleep and scheduling using localTimeRK
// v0.03 - Refactoring the code to break up the main file monolith

// Particle Libraries
#include <RHMesh.h>
#include <RH_RF95.h>						        // https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "Particle.h"                               // Because it is a CPP file not INO
// Application Files
#include "LoRA_Functions.h"							// Where we store all the information on our LoRA implementation - application specific not a general library
#include "device_pinout.h"							// Define pinouts and initialize them
#include "particle_fn.h"							// Particle specific functions
#include "storage_objects.h"						// Manage the initialization and storage of persistent objects
#include "take_measurements.h"						// Manages interactions with the sensors (default is temp for charging)

// Support for Particle Products (changes coming in 4.x - https://docs.particle.io/cards/firmware/macros/product_id/)
PRODUCT_ID(PLATFORM_ID);                            // Device needs to be added to product ahead of time.  Remove once we go to deviceOS@4.x
PRODUCT_VERSION(0);
char currentPointRelease[6] ="0.04";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();
void recordCount();

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;
bool rescueMode = false;
time_t lastPublish = Time.now();

void setup() {
	delay(5000);	// Wait for serial 

    initializePinModes();                           // Sets the pinModes

    initializePowerCfg();                           // Sets the power configuration for solar

    storageObjectStart();                           // Sets up the storage for system and current status in storage_objects.h

    // particleInitialize();                           // Sets up all the Particle functions and variables defined in particle_fn.h

    {                                               // Initialize AB1805 Watchdog and RTC                                 
        ab1805.withFOUT(D8).setup();                // The carrier board has D8 connected to FOUT for wake interrupts
        ab1805.resetConfig();                       // Reset the AB1805 configuration to default values
        ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);// Enable watchdog
    }

	initializeLoRA(false);								// Start the LoRA radio - Node

	// Local nodes don't need to know the actual time - their clocks will be set by the gateway
	if (!Time.isValid()) rescueMode = true;
  	Log.info("Startup complete with %s time and with battery %4.2f", (Time.isValid())? "valid" : "invalid", System.batteryCharge());

  	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); // We may need to monitor the user switch to change behaviours / modes

	if (!Time.isValid() || sysStatus.nodeNumber < 10) state = ERROR_STATE;

	if (state == INITIALIZATION_STATE) state = LoRA_STATE;  // This is not a bad way to start - could also go to the LoRA_STATE
}

void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			if (Time.now() - lastPublish > sysStatus.nextReportSeconds) state = LoRA_STATE;		   // If time is valid - wake on the right minute of the hour
		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			int wakeInSeconds = sysStatus.nextReportSeconds - (Time.now() - lastPublish);  // sleep till next event
			Log.info("Sleep for %i seconds", wakeInSeconds);
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.duration(wakeInSeconds * 1000L);
			SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
			if (result.wakeupPin() == BUTTON_PIN) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				setLowPowerMode("0");                                          // We are waking the device for a reason
				Log.info("Woke with user button - normal operations");
			}
			state = IDLE_STATE;
		} break;

		case LoRA_STATE: {
			if (state != oldState) {
				publishStateTransition();                   					// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				takeMeasurements();
				lastPublish = Time.now();
				if (!composeDataReportNode()) rescueMode = true;				// Initiate sending report
			} 
			// The big difference between a node and a gateway - the Node initiates a LoRA exchance by sending data
			if (receiveAcknowledmentDataReportNode()) {							// Listen for acknowledgement
				current.hourly = 0;												// Zero the hourly count
				if (!sysStatus.lowPowerMode && (sysStatus.nextReportSeconds / 60 > 12)) sysStatus.lowPowerMode = true;
				state = IDLE_STATE;
			}
			else rescueMode = true;
		} break;

		case ERROR_STATE: {														// Where we go if things are not quite right
			composeJoinRequesttNode();
			if (receiveAcknowledmentJoinRequestNode()) state = IDLE_STATE;
			else rescueMode = true;
		}
	}

	ab1805.loop();                                  							// Keeps the RTC synchronized with the Boron's clock

    storageObjectLoop();                            							// Compares current system and current objects and stores if the hash changes (once / second) in storage_objects.h

	if (rescueMode) {
		rescueMode = false;
		sysStatus.nextReportSeconds = 60;										// Rescue mode publish evert minute until we can connect
		sysStatus.lowPowerMode = false;
		Log.info("Send failed - going to send every minute");
		state = IDLE_STATE;
	}
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

void userSwitchISR() {
  userSwitchDectected = true;                                            // The the flag for the user switch interrupt
}