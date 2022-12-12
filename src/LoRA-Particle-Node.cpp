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
// v0.11 - Big changes to messages and storage.  Works reliably now.
// v0.12 - Better management of node number at startup, also explicit set time for RTC
// v0.13 - Gateway sets open / close and alerts - sysStatus object and join ack updated
// v0.14 - Gateway can trigger join request to fix issues with deviceID and sensorType
// v0.15 - Implementing recovery steps for missed connections

#define NODENUMBEROFFSET 10UL						// By how much do we off set each node by node number

// Particle Libraries
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "Particle.h"                               // Because it is a CPP file not INO
#include <math.h>
// Application Libraries / Class Files
#include "LoRA_Functions.h"							// Where we store all the information on our LoRA implementation - application specific not a general library
#include "device_pinout.h"							// Define pinouts and initialize them
#include "take_measurements.h"						// Manages interactions with the sensors (default is temp for charging)
#include "MyPersistentData.h"						// Persistent Storage
#include "node_configuration.h"

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// For monitoring / debugging, you have some options on the next few lines - uncomment one
//SerialLogHandler logHandler(LOG_LEVEL_TRACE);
//SerialLogHandler logHandler(LOG_LEVEL_ALL);         // All the loggings 
SerialLogHandler logHandler(LOG_LEVEL_INFO);     // Easier to see the program flow
// Serial1LogHandler logHandler1(57600);            // This line is for when we are using the OTII ARC for power analysis

char currentPointRelease[6] ="0.15";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();
int secondsTillNextEvent();							// This is the node version

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)


// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_TRANSMISSION_STATE, LoRA_LISTENING_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA Transmit", "LoRA Listening", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
void outOfMemoryHandler(system_event_t event, int param);

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;
static int retryState = 0;										// We will attempt up to three retries - exponential backoff

void setup() {
	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes

    initializePowerCfg();                           // Sets the power configuration for solar

	{
		current.setup();
		sysStatus.setup();							// Initialize persistent storage
	}

	sysStatus.checkSystemValues();					// Make sure system values are in bounds for normal operation

	setNodeConfiguration();                         // This is a function for development - allows us to over-ride stored system values

    {                                               // Initialize AB1805 Watchdog and RTC                                 
        ab1805.withFOUT(D8).setup();                // The carrier board has D8 connected to FOUT for wake interrupts
        ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);// Enable watchdog
    }

	System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	// In this section we test for issues and set alert codes as needed
	if (! LoRA_Functions::instance().setup(false)) 	{						// Start the LoRA radio - Node
		sysStatus.set_alertCodeNode(3);										// Initialization failure
		sysStatus.set_alertTimestampNode(Time.now());
		Log.info("LoRA Initialization failure alert code %d - power cycle in 30", sysStatus.get_alertCodeNode());
	}
	else if (sysStatus.get_nodeNumber() > 10 || !Time.isValid()) {			// If the node number indicates this node is uninitialized or the clock needs to be set, initiate a join request
		sysStatus.set_alertCodeNode(1); 					// Will initiate a join request
		Log.info("Node number indicated unconfigured node of %d setting alert code to %d", sysStatus.get_nodeNumber(), sysStatus.get_alertCodeNode());
	}

  	takeMeasurements();                                                  	// Populates values so you can read them before the hour
  
    attachInterrupt(INT_PIN, sensorISR, RISING);                     		// PIR or Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); 						// We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = SLEEPING_STATE;               	// Sleep unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d", sysStatus.get_alertCodeNode());
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
  			if (Time.isValid() && Time.day() != Time.day(current.get_lastCountTime())) current.resetEverything();           // Check to see if the device was last on in a different day
			if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;
			else state = LoRA_TRANSMISSION_STATE;  // If time is valid - wake on the right minute of the hour

		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			int wakeInSeconds = secondsUntilNextEvent();					// Figure out how long to sleep 
			time_t time = Time.now() + wakeInSeconds;

			Log.info("Sleep for %i seconds until next event at %s with sensor %s", \
			wakeInSeconds, (Time.isValid()) ? Time.format(time, "%T").c_str():" ", (sysStatus.get_openHours()) ? "on" : "off");
			if (wakeInSeconds == 0) {										// While not likely, a zero result is possible
				state = IDLE_STATE;
				break;
			}

			if (!sysStatus.get_openHours()) digitalWrite(MODULE_POWER_PIN,HIGH);  // disable (HIGH) the sensor
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			ab1805.stopWDT();  												// No watchdogs interrupting our slumber
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			waitFor(Serial.isConnected, 10000);								// Wait for serial connection
			ab1805.resumeWDT();                                             // Wakey Wakey - WDT can resume
			digitalWrite(MODULE_POWER_PIN,LOW);             				// Enable (LOW) the sensor

			if (result.wakeupPin() == BUTTON_PIN) {                         // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				Log.info("Woke with user button - LoRA State");
				state = LoRA_TRANSMISSION_STATE;
			}
			else if (result.wakeupPin() == INT_PIN) {
				Log.info("Woke with sensor interrupt - Record count then sleep");
				if (recordCount()) sensorDetect = false;					// Record count and reset flag
				if (secondsUntilNextEvent() < 10) state=IDLE_STATE;
				else state = SLEEPING_STATE;
			}
			else {
				Log.info("Awoke at %s with %li free memory", Time.timeStr(Time.now()+wakeInSeconds).c_str(), System.freeMemory());
				state = IDLE_STATE;
			}
		} break;

		case LoRA_TRANSMISSION_STATE: {
			bool result = false;

			if (state != oldState) {
				publishStateTransition();                   				// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				LoRA_Functions::instance().clearBuffer();
				takeMeasurements();

				if (sysStatus.get_alertCodeNode() == 0) result = LoRA_Functions::instance().composeDataReportNode();
				else if (sysStatus.get_alertCodeNode() == 1) result = LoRA_Functions::instance().composeJoinRequesttNode();
				else result = LoRA_Functions::instance().composeAlertReportNode();

				if (!result) {
					Log.info("Failed in data send attempting retry");
					retryState++;
					state = SLEEPING_STATE;
				}
				else {
					retryState = 0;
					state = LoRA_LISTENING_STATE;
				}
			}
		} break;

		case LoRA_LISTENING_STATE: {
			static system_tick_t startListening = 0;

			if (state != oldState) {
				publishStateTransition();                   				// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				startListening = millis();
			}

			while (millis() - startListening < 5000) {
				// The big difference between a node and a gateway - the Node initiates a LoRA exchange by sending data
				if (LoRA_Functions::instance().listenForLoRAMessageNode()) {// Listen for acknowledgement
					sysStatus.set_lastConnection(Time.now());
					randomSeed(sysStatus.get_lastConnection());				// Done so we can genrate rando numbers later
					ab1805.setRtcFromTime(Time.now());
					if (sysStatus.get_alertCodeNode() > 0) {				// If we are reporting an alert we will also report data
						sysStatus.set_alertTimestampNode(Time.now());
						sysStatus.set_alertCodeNode(0);
						state = IDLE_STATE;									// This is enable us to send the data now that alert is reported
					}
					else {
						static int lastReportingHour = Time.hour();
						if (Time.hour() != lastReportingHour) {
							current.set_hourlyCount(0);					    // Zero the hourly count
							lastReportingHour = Time.hour();
						}
						LoRA_Functions::instance().sleepLoRaRadio();		// Done with LoRA - put radio to sleep
						state = SLEEPING_STATE;
					}
					break;
				}
			}
		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (sysStatus.get_alertCodeNode())
			{
			case 1:	{														// Case 1 is an unconfigured node - needs to send join request
				state = LoRA_TRANSMISSION_STATE;
			} break;

			case 2:	{														// Case 2 is for Time not synced
				state = LoRA_TRANSMISSION_STATE;
			} break;
			case 3: {														// Case 3 is generic - power cycle device to recover from errors
				static system_tick_t enteredState = millis();
				if (millis() - enteredState > 30000L) {
					Log.info("Resetting device");
					delay(2000);
					ab1805.deepPowerDown();
				}
			case 4: {
				// Reset LoRA radio
			}
			} break;

			default:
				Log.info("Undefined Error State");
				sysStatus.set_alertCodeNode(0);
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
	// First we will calculate the adjustment to the wakeboundary due to node number and retries
	if (retryState) {														// We will use an exponential back-off for 3 retries
		int fiftyFifty = random(2);											// Flip a coin - seed in Setup()
		unsigned long offset = NODENUMBEROFFSET * fiftyFifty * pow(2, retryState); // This is an exponential back-off
		Log.info("Node %d retry state %d dice of %d retry in %lu seconds", sysStatus.get_nodeNumber(), retryState, fiftyFifty, offset);
		if (retryState >= 3) retryState = 0;							    // Going to stop re-trying and wait until the next period
		return offset;
	}
	else if (Time.isValid()) {												// The general case - has to handle node-number specific timing
		unsigned long nodeSpecificOffset = NODENUMBEROFFSET * sysStatus.get_nodeNumber(); // Each node has an offset to avoid collisions
		unsigned long wakeBoundary = (sysStatus.get_frequencyMinutes() * 60UL);
		unsigned long secondsToReturn = constrain(wakeBoundary - Time.now() % wakeBoundary, 0UL, wakeBoundary);  // If Time is valid, we can compute time to the start of the next report window	
		if (sysStatus.get_nodeNumber() < 11)  {								// We need to test here as the off-set can cause missed reporting if awoken by the sensor at the period boundary
			if (secondsToReturn <= nodeSpecificOffset) secondsToReturn = nodeSpecificOffset - (nodeSpecificOffset - secondsToReturn);
			else secondsToReturn += nodeSpecificOffset;							// Off-set for configured nodes only
		}							
		return secondsToReturn;
    }
	else return 60UL;	// If time is not valid, we need to keep trying to catch the Gateway when it next wakes up.
}