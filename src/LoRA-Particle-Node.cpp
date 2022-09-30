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
// v0.09 - Added LoRA Radio sleep / clear buffer
// v0.10 - Lots of little fixes minimally works now

// Particle Libraries
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "Particle.h"                               // Because it is a CPP file not INO
// Application Libraries / Class Files
#include "LoRA_Functions.h"							// Where we store all the information on our LoRA implementation - application specific not a general library
#include "device_pinout.h"							// Define pinouts and initialize them
#include "take_measurements.h"						// Manages interactions with the sensors (default is temp for charging)
#include "MyPersistentData.h"						// Persistent Storage

// Support for Particle Products (changes coming in 4.x - https://docs.particle.io/cards/firmware/macros/product_id/)
PRODUCT_VERSION(0);
char currentPointRelease[6] ="0.10";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();
int secondsTillNextEvent();							// This is the node version

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)


// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
void outOfMemoryHandler(system_event_t event, int param);

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;
time_t lastPublish = Time.now();

void setup() {
	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes

    initializePowerCfg();                           // Sets the power configuration for solar

	{
		current.setup();
		sysStatus.setup();								// Initialize persistent storage
	}

    {                                               // Initialize AB1805 Watchdog and RTC                                 
        ab1805.withFOUT(D8).setup();                // The carrier board has D8 connected to FOUT for wake interrupts
        ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);// Enable watchdog
    }

	System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	// In this section we test for issues and set alert codes as needed
	if (! LoRA_Functions::instance().setup(false)) 	{						// Start the LoRA radio - Node
		current.set_alertCodeNode(3);										// Initialization failure
		current.set_alertTimestampNode(Time.now());
		Log.info("LoRA Initialization failure alert code %d - power cycle in 30", current.get_alertCodeNode());
	}
	else if (sysStatus.get_nodeNumber() < 10) {			// If there is already a hardware alert - deal with that first
		current.set_alertCodeNode(1); // For testing
		Log.info("Node number indicated unconfigured node of %d setting alert code to %d", current.get_nodeNumber(), current.get_alertCodeNode());
	}

  	takeMeasurements();                                                  	// Populates values so you can read them before the hour
  
    attachInterrupt(INT_PIN, sensorISR, RISING);                     		// PIR or Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); 						// We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;               	// IDLE unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d", current.get_alertCodeNode());
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
  			if (Time.isValid() && Time.day() != Time.day(current.get_lastCountTime())) resetEverything();           // Check to see if the device was last on in a different day
			if (current.get_alertCodeNode() != 0) state = ERROR_STATE;
			else state = LoRA_STATE;		   								// If time is valid - wake on the right minute of the hour

		} break;

		case SLEEPING_STATE: {
			int wakeInSeconds = 0;
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												// No watchdogs interrupting our slumber
			wakeInSeconds = secondsUntilNextEvent();						// Figure out how long to sleep 
			Log.info("Sleep for %i seconds until next event %s", wakeInSeconds, (Time.isValid()) ? Time.timeStr(Time.now()+wakeInSeconds).c_str(): " ");
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			waitFor(Serial.isConnected, 10000);								// Wait for serial connection
			ab1805.resumeWDT();                                             // Wakey Wakey - WDT can resume
			if (result.wakeupPin() == BUTTON_PIN) {                         // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				Log.info("Woke with user button - LoRA State");
				state = LoRA_STATE;
			}
			else if (result.wakeupPin() == INT_PIN) {
				Log.info("Woke with sensor interrupt - Record count then sleep");
				if (recordCount()) sensorDetect = false;					// Record count and reset flag
				state = SLEEPING_STATE;
			}
			else {
				Log.info("Awoke at %s with %li free memory", Time.timeStr(Time.now()+wakeInSeconds).c_str(), System.freeMemory());
				state = IDLE_STATE;
			}
			

		} break;

		case LoRA_STATE: {
			if (state != oldState) {
				publishStateTransition();                   				// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				LoRA_Functions::instance().clearBuffer();
				takeMeasurements();
				lastPublish = Time.now();
				if (!LoRA_Functions::instance().composeDataReportNode()) {
					Log.info("Failed in data send");
					break;
				}
			} 

			system_tick_t startListening = millis();

			while (millis() - startListening < 5000) {
				// The big difference between a node and a gateway - the Node initiates a LoRA exchange by sending data
				if (LoRA_Functions::instance().listenForLoRAMessageNode()) {// Listen for acknowledgement
					current.set_hourlyCount(0);								// Zero the hourly count
					sysStatus.set_lastConnection(Time.now());
					break;
				}
			}

			LoRA_Functions::instance().sleepLoRaRadio();					// Done with LoRA - put radio to sleep
			state = SLEEPING_STATE;

		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (current.get_alertCodeNode())
			{
			case 1:	{														// Case 1 is an unconfigured node - needs to send join request
				if(LoRA_Functions::instance().composeJoinRequesttNode()) {
					system_tick_t startListening = millis();
					while (millis() - startListening < 3000) {
						if (LoRA_Functions::instance().listenForLoRAMessageNode()) {
							lastPublish = Time.now();
							sysStatus.set_lastConnection(Time.now());
							current.set_alertTimestampNode(Time.now());
							current.set_alertCodeNode(0);
						}
					}
				}
				LoRA_Functions::instance().sleepLoRaRadio();				// Done for now, put radio to sleep
				state = IDLE_STATE;
			} break;

			case 2:	{														// Case 2 is for Time not synced
				if(LoRA_Functions::instance().composeAlertReportNode()) {
					system_tick_t startListening = millis();
					while (millis() - startListening < 3000) {
						if (LoRA_Functions::instance().listenForLoRAMessageNode()) {
							lastPublish = Time.now();
							sysStatus.set_lastConnection(Time.now());
							current.set_alertTimestampNode(Time.now());
							current.set_alertCodeNode(0);
						}
					}
				}
				LoRA_Functions::instance().sleepLoRaRadio();				// Done for now, put radio to sleep
				state = IDLE_STATE;
			} break;
			case 3: {														// Case 3 is generic - power cycle device to recover from errors
				static system_tick_t enteredState = millis();
				if (millis() - enteredState > 30000L) {
					Log.info("Resetting device");
					delay(2000);
					ab1805.deepPowerDown();
				}
			} break;

			default:
				Log.info("Undefined Error State");
				current.set_alertCodeNode(0);
				state = IDLE_STATE;
			break;
			}
		}
	}

	// Housekeeping for each transit of the main loop
	ab1805.loop();                                  						// Keeps the RTC synchronized with the Boron's clock

	current.loop();
	sysStatus.loop();

	if (sensorDetect) {														// Count the pulse and reset for next
		if (recordCount()) sensorDetect = false;
	}

	if (outOfMemory >= 0) {                         // In this function we are going to reset the system if there is an out of memory error
		System.reset();
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
		else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	}
	else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	oldState = state;
	Log.info(stateTransitionString);
}

// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}

void userSwitchISR() {
  	userSwitchDectected = true;                                          	// The the flag for the user switch interrupt
}

void sensorISR()
{
  static bool frontTireFlag = false;
  if (frontTireFlag || sysStatus.get_sensorType() == 1) {               	// Counts the rear tire for pressure sensors and once for PIR
    sensorDetect = true;                                              		// sets the sensor flag for the main loop
    frontTireFlag = false;
  }
  else frontTireFlag = true;
}

/**
 * @brief Function to compute time to next scheduled event
 * 
 * @details - Computes seconds and returns 0 if no event is scheduled or time is invalid
 * 
 * 
 */
int secondsUntilNextEvent() {												// Time till next scheduled event
	unsigned long secondsToReturn = 0;
	unsigned long wakeBoundary = sysStatus.get_frequencyMinutes() * 60UL;
   	if (Time.isValid()) {
		secondsToReturn = constrain( wakeBoundary - Time.now() % wakeBoundary, 0UL, wakeBoundary);  // Adding one second to reduce prospect of round tripping to IDLE
        Log.info("Report frequency %d mins, next event in %lu seconds", sysStatus.get_frequencyMinutes(), secondsToReturn);
    }
	return secondsToReturn + 10;											// Add an off-set - need to refine this later.
}