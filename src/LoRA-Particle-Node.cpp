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
// v0.06 - Added settings for selecting sensor type and recording counts - 
// v0.07 - StorageHelperRK support
// v0.08 - LoRA Functions moved to Class


// Particle Libraries
#include <RHMesh.h>
#include <RH_RF95.h>						        // https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
// #include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "Particle.h"                               // Because it is a CPP file not INO
// Application Files
#include "LoRA_Functions.h"							// Where we store all the information on our LoRA implementation - application specific not a general library
#include "device_pinout.h"							// Define pinouts and initialize them
#include "particle_fn.h"							// Particle specific functions
#include "take_measurements.h"						// Manages interactions with the sensors (default is temp for charging)
#include "MyPersistentData.h"						// Persistent Storage

// Support for Particle Products (changes coming in 4.x - https://docs.particle.io/cards/firmware/macros/product_id/)
PRODUCT_VERSION(0);
char currentPointRelease[6] ="0.08";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();
int secondsTillNextEvent();						// This is the node version

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;
bool rescueMode = false;
time_t lastPublish = Time.now();

void setup() {
	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes

    initializePowerCfg();                           // Sets the power configuration for solar

	current.setup();
	sysStatus.setup();								// Initialize persistent storage

    {                                               // Initialize AB1805 Watchdog and RTC                                 
        ab1805.withFOUT(D8).setup();                // The carrier board has D8 connected to FOUT for wake interrupts
        ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);// Enable watchdog
    }

  	// Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  	if (Time.day() != Time.day(current.get_lastCountTime())) {                 // Check to see if the device was last on in a different day
    	resetEverything();                                               // Zero the counts for the new day
  	}

	if (! LoRA_Functions::instance().setup(false)) 	{	// Start the LoRA radio - Node
		current.set_alertCodeNode(3);				// Initialization failure
		current.set_alertTimestampNode(Time.now());
		Log.info("LoRA Initialization failure alert code %d - power cycle in 30", current.get_alertCodeNode());
	}

	if ((current.get_alertCodeNode() == 0) && (sysStatus.get_nodeNumber() < 10)) {	// If there is already a hardware alert - deal with that first
		current.set_alertCodeNode(1); // For testing
		sysStatus.set_nextReportSeconds(10);
		Log.info("Node number indicated unconfigured node of %d setting alert code to  %d", current.get_nodeNumber(), current.get_alertCodeNode());
	}

	sysStatus.set_nextReportSeconds(10);

	/*
	else if (!Time.isValid()) {
		current.alertCodeNode = 2;
		sysStatus.nextReportSeconds = 10;
	}
	*/

  	takeMeasurements();                                                  // Populates values so you can read them before the hour
  
	// Nodes don't worry about open and close they just folow orders from the gateway
    attachInterrupt(INT_PIN, sensorISR, RISING);                     	// Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); // We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;               // IDLE unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d", current.get_alertCodeNode());
  	digitalWrite(BLUE_LED,LOW);                                           // Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			if ((Time.now() - lastPublish) > sysStatus.get_nextReportSeconds()) {
				Log.info("Time to publish with alert code %d", current.get_alertCodeNode());
				if (current.get_alertCodeNode() != 0) state = ERROR_STATE;
				else state = LoRA_STATE;		   								// If time is valid - wake on the right minute of the hour
			}
			else state = SLEEPING_STATE;										// If we have time, let's take a nap
		} break;

		case SLEEPING_STATE: {
			int wakeInSeconds = 0;
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			wakeInSeconds = secondsTillNextEvent();								// Figure out how long to sleep 
			Log.info("Sleep for %i seconds until next event %s", wakeInSeconds, (Time.isValid()) ? Time.timeStr(Time.now()+wakeInSeconds).c_str(): " ");
			delay(2000);									// Make sure message gets out
						state = IDLE_STATE;
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
			waitFor(Serial.isConnected, 10000);				// Wait for serial connection
			if (result.wakeupPin() == BUTTON_PIN) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				state = LoRA_STATE;
				Log.info("Woke with user button - LoRA State");
			}
			else if (result.wakeupPin() == INT_PIN) sensorDetect = true;
			Log.info("Awoke at %s with %li free memory", Time.timeStr(Time.now()+wakeInSeconds).c_str(), System.freeMemory());

		} break;

		case LoRA_STATE: {
			if (state != oldState) {
				publishStateTransition();                   					// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				takeMeasurements();
				lastPublish = Time.now();
				if (!LoRA_Functions::instance().composeDataReportNode()) {
					rescueMode = true;											// Initiate sending report
					Log.info("Failed in send and rescue is %s", (rescueMode) ? "On" : "Off");
					break;
				}
			} 

			system_tick_t startListening = millis();

			while (millis() - startListening < 10000) {
				// The big difference between a node and a gateway - the Node initiates a LoRA exchange by sending data
				if (LoRA_Functions::instance().listenForLoRAMessageNode()) {									// Listen for acknowledgement
					current.set_hourlyCount(0);										// Zero the hourly count
					rescueMode = false;
					sysStatus.set_lastConnection(Time.now());
					Log.info("Send and Ack succeeded and rescue is %s", (rescueMode) ? "On" : "Off");
					state = IDLE_STATE;
					break;
				}
				else {
					rescueMode = true;
					Log.info("Failed in ack and rescue is %s", (rescueMode) ? "On" : "Off");
				}
			}
			delay(1000);  // Temporary - prevents muliple sends

		} break;

		case ERROR_STATE: {														// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (current.get_alertCodeNode())
			{
			case 1:	{															// Case 1 is an unconfigured node - needs to send join request
				if(LoRA_Functions::instance().composeJoinRequesttNode()) {
					system_tick_t startListening = millis();
					while (millis() - startListening < 3000) {
						if (LoRA_Functions::instance().listenForLoRAMessageNode()) {
							lastPublish = Time.now();
							rescueMode = false;
							sysStatus.set_lastConnection(Time.now());
							current.set_alertTimestampNode(Time.now());
							current.set_alertCodeNode(0);
						}
					}
				}
				else rescueMode = true;
				state = IDLE_STATE;
			} break;

			case 2:	{															// Case 2 is for Time not synced
				if(LoRA_Functions::instance().composeAlertReportNode()) {
					system_tick_t startListening = millis();
					while (millis() - startListening < 3000) {
						if (LoRA_Functions::instance().listenForLoRAMessageNode()) {
							lastPublish = Time.now();
							rescueMode = false;
							sysStatus.set_lastConnection(Time.now());
							current.set_alertTimestampNode(Time.now());
							current.set_alertCodeNode(0);
						}
					}
				}
				else rescueMode = true;
				state = IDLE_STATE;
			} break;
			case 3: {
				static system_tick_t enteredState = millis();
				if (millis() - enteredState > 30000L) {
					Log.info("Resetting device");
					delay(2000);
					ab1805.deepPowerDown();
				}
			} break;

			default:

				break;
			}
		}
	}

	ab1805.loop();                                  							// Keeps the RTC synchronized with the Boron's clock

	current.loop();
	sysStatus.loop();

	if (rescueMode) {
		rescueMode = false;
		sysStatus.set_nextReportSeconds(10);										// Rescue mode publish evert minute until we can connect
		sysStatus.set_lowPowerMode(false);
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
		else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s for %u seconds", stateNames[oldState],stateNames[state],sysStatus.get_nextReportSeconds());
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
  if (frontTireFlag || sysStatus.get_sensorType() == 1) {                   // Counts the rear tire for pressure sensors and once for PIR
    sensorDetect = true;                                              // sets the sensor flag for the main loop
    frontTireFlag = false;
  }
  else frontTireFlag = true;
}

int secondsTillNextEvent() {										// This is the node version
	int returnSeconds = 60;

	if (Time.isValid()) {											// We may need to sleep when time is not valid
		if (sysStatus.get_nextReportSeconds() > (Time.now() - sysStatus.get_lastConnection())) {						// If this is false, we missed the last event.
			return (sysStatus.get_nextReportSeconds() - (Time.now() - sysStatus.get_lastConnection()) + 5);  			// sleep till next event - Add 5 seconds so it does not miss the gateway
		}
	}

	return returnSeconds;
}
