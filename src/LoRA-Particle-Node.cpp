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
// v0.04 - Tested with Join and Report - works!
// v0.05 - Added the Alert Report type 
// v0.06 - Added settings for selecting sensor type and recording counts

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
PRODUCT_VERSION(0);
char currentPointRelease[6] ="0.05";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();

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
  	Log.info("Startup complete with %s time and with battery %4.2f", (Time.isValid())? "valid" : "invalid", System.batteryCharge());


  	// Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  	if (Time.day() != Time.day(current.lastCountTime)) {                 // Check to see if the device was last on in a different day
    	resetEverything();                                               // Zero the counts for the new day
  	}

	if (sysStatus.nodeNumber < 10) {
		current.alertCodeNode = 1; // For testing
		sysStatus.nextReportSeconds = 10;
	}
	else if (!Time.isValid()) {
		current.alertCodeNode = 2;
		sysStatus.nextReportSeconds = 10;
	}

  	takeMeasurements();                                                  // Populates values so you can read them before the hour
  
	// Nodes don't worry about open and close they just folow orders from the gateway
    attachInterrupt(INT_PIN, sensorISR, RISING);                     	// Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); // We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;               // IDLE unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d", current.alertCodeNode);
  	digitalWrite(BLUE_LED,LOW);                                           // Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			if ((Time.now() - lastPublish) > sysStatus.nextReportSeconds) {
				if (current.alertCodeNode) state = ERROR_STATE;
				else state = LoRA_STATE;		   								// If time is valid - wake on the right minute of the hour
			}
			else state = SLEEPING_STATE;										// If we have time, let's take a nap
		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			int wakeInSeconds = sysStatus.nextReportSeconds - (Time.now() - lastPublish);  // sleep till next event
			Log.info("Sleep for %i seconds until next event at %s", wakeInSeconds, Time.timeStr(Time.now()+wakeInSeconds).c_str());
			delay(2000);									// Make sure message gets out
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
			if (result.wakeupPin() == BUTTON_PIN) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				setLowPowerMode("0");                                          // We are waking the device for a reason
				Log.info("Woke with user button - normal operations");
			}
			else if (result.wakeupPin() == INT_PIN) sensorDetect = true;
			state = IDLE_STATE;
			delay(2000);
			Log.info("Awoke at %s with %li free memory", Time.timeStr(Time.now()+wakeInSeconds).c_str(), System.freeMemory());

		} break;

		case LoRA_STATE: {
			if (state != oldState) {
				publishStateTransition();                   					// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				takeMeasurements();
				lastPublish = Time.now();
				if (!composeDataReportNode()) {
					rescueMode = true;											// Initiate sending report
					Log.info("Failed in send and rescue is %s", (rescueMode) ? "On" : "Off");
					break;
				}
			} 

			system_tick_t startListening = millis();

			while (millis() - startListening < 5000) {
				// The big difference between a node and a gateway - the Node initiates a LoRA exchange by sending data
				if (listenForLoRAMessageNode()) {									// Listen for acknowledgement
					current.hourlyCount = 0;										// Zero the hourly count
					rescueMode = false;
					sysStatus.lastConnection = Time.now();
					Log.info("Send and Ack succeeded and rescue is %s", (rescueMode) ? "On" : "Off");
					state = IDLE_STATE;
					break;
				}
				else {
					rescueMode = true;
					Log.info("Failed in ack and rescue is %s", (rescueMode) ? "On" : "Off");
				}
			}

		} break;

		case ERROR_STATE: {														// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (current.alertCodeNode)
			{
			case 1:	{															// Case 1 is an unconfigured node - needs to send join request
					if(composeJoinRequesttNode()) {
						system_tick_t startListening = millis();
						while (millis() - startListening < 3000) {
							if (listenForLoRAMessageNode()) {
								lastPublish = Time.now();
								rescueMode = false;
								sysStatus.lastConnection = Time.now();
								current.alertTimestampNode = Time.now();
								current.alertCodeNode = 0;
							}
						}
					}
					else rescueMode = true;
				} break;

			case 2:	{															// Case 2 is for Time not synced
					if(composeAlertReportNode()) {
						system_tick_t startListening = millis();
						while (millis() - startListening < 3000) {
							if (listenForLoRAMessageNode()) {
								lastPublish = Time.now();
								rescueMode = false;
								sysStatus.lastConnection = Time.now();
								current.alertTimestampNode = Time.now();
								current.alertCodeNode = 0;
							}
						}
					}
					else rescueMode = true;
				} break;

			default:

				break;
			}
			state = IDLE_STATE;
		}
	}

	ab1805.loop();                                  							// Keeps the RTC synchronized with the Boron's clock

	storageObjectLoop();   // Compares current system and current objects and stores if the hash changes (once / second) in storage_objects.h

	if (rescueMode) {
		rescueMode = false;
		sysStatus.nextReportSeconds = 60;										// Rescue mode publish evert minute until we can connect
		sysStatus.lowPowerMode = false;
		Log.info("Send failed - going to send every minute");
		state = IDLE_STATE;
	}

	if (sensorDetect) {															// Count the pulse and reset for next
		sensorDetect = false;
		recordCount();
	}
}

/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
	char stateTransitionString[256];
	if (state == IDLE_STATE) {
		if (!Time.isValid()) snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s with invalid time", stateNames[oldState],stateNames[state]);
		else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s for %lu seconds", stateNames[oldState],stateNames[state],(sysStatus.nextReportSeconds - (Time.now() - sysStatus.lastConnection)));
	}
	else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	oldState = state;
	Log.info(stateTransitionString);
}

void userSwitchISR() {
  	userSwitchDectected = true;                                            // The the flag for the user switch interrupt
}

void sensorISR()
{
  static bool frontTireFlag = false;
  if (frontTireFlag || sysStatus.sensorType == 1) {                   // Counts the rear tire for pressure sensors and once for PIR
    sensorDetect = true;                                              // sets the sensor flag for the main loop
    frontTireFlag = false;
  }
  else frontTireFlag = true;
}
