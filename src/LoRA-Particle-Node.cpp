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
// v0.16 - More robust try and less missed reporting events
// v0.17 - Implemented full alert suite
// v1 - Release Candidate - sending to Pilot mountain
// v1.07 - Aligning numbers to Gateway - added nodeNumber validation, removed Alert message type

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


// Prototypes and System Mode calls
SYSTEM_MODE(MANUAL);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// For monitoring / debugging, you have some options on the next few lines - uncomment one
SerialLogHandler logHandler(LOG_LEVEL_INFO);     // Easier to see the program flow

char currentPointRelease[6] ="1.07";

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
int secondsUntilNextEvent(); 						// Time till next scheduled event
void sensorISR();

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
int retryState = 0;									// We will attempt up to three retries - exponential backoff

void setup() {

	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes

    initializePowerCfg();                           // Sets the power configuration for solar

	sysStatus.setup();								// Initialize persistent storage
	current.setup();
                              
    ab1805.withFOUT(D8).setup();                	// The carrier board has D8 connected to FOUT for wake interrupts
    ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);	// Enable watchdog

	System.on(out_of_memory, outOfMemoryHandler);   // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	// In this section we test for issues and set alert codes as needed
	if (! LoRA_Functions::instance().setup(false)) 	{						// Start the LoRA radio - Node
		sysStatus.set_alertCodeNode(3);										// Initialization failure
		sysStatus.set_alertTimestampNode(Time.now());
		Log.info("LoRA Initialization failure alert code %d - power cycle in 30", sysStatus.get_alertCodeNode());
	}
	else if (sysStatus.get_nodeNumber() > 10 || !Time.isValid()) {			// If the node number indicates this node is uninitialized or the clock needs to be set, initiate a join request
		sysStatus.set_alertCodeNode(1); 									// Will initiate a join request
		Log.info("Node number indicated unconfigured node of %d setting alert code to %d", sysStatus.get_nodeNumber(), sysStatus.get_alertCodeNode());
	}

  	takeMeasurements();                                                  	// Populates values so you can read them before the hour

	if (!digitalRead(BUTTON_PIN)) {
		Log.info("User button pressed, will force connection to Particle");
		Particle.connect();													// Will stay connected until reset
	}
  
    attachInterrupt(INT_PIN, sensorISR, RISING);                     		// PIR or Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); 						// We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = SLEEPING_STATE;               	// Sleep unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d and last connect %s", sysStatus.get_alertCodeNode(), Time.format(sysStatus.get_lastConnection(), "%T").c_str());
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {													// Unlike most sketches - nodes spend most time in sleep and only transit IDLE once or twice each period
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			// In this section, we will work through steps to recover from lost connectivity
			if (sysStatus.get_frequencyMinutes() == 1) Log.info("Trying to reconnect to gateway");  // Something is wrong - attempting every minute to connect
			else if ((Time.now() - sysStatus.get_lastConnection() > 2 * sysStatus.get_frequencyMinutes() * 60UL) && sysStatus.get_openHours()) { // Park is open but no connect for over two hours
				Log.info("Park is open but we have not connected for over two reporting periods - need to power cycle and go to 1 min frequency");
				sysStatus.set_alertCodeNode(3);								// This will trigger a power cycle reset
				sysStatus.set_alertTimestampNode(Time.now());		
				sysStatus.set_frequencyMinutes(1);							// Will wake every minute to send data - have to catch gateway when it is awake
				sysStatus.set_lastConnection(Time.now());					// Prevents cyclical resets
			}

			if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;
			else state = LoRA_TRANSMISSION_STATE;  							// No error - send data

		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			int wakeInSeconds = secondsUntilNextEvent();					// Figure out how long to sleep 
			time_t time = Time.now() + wakeInSeconds;

			Log.info("Sleep for %i seconds until next event at %s with sensor %s", \
			wakeInSeconds, (Time.isValid()) ? Time.format(time, "%T").c_str():"invalid time", (sysStatus.get_openHours()) ? "on" : "off");
			if (!sysStatus.get_openHours()) digitalWrite(MODULE_POWER_PIN,HIGH);  // disable (HIGH) the sensor
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			ab1805.stopWDT();  												// No watchdogs interrupting our slumber
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                             // Wakey Wakey - WDT can resume
			delay(2000);		//  Diagnostic code
			digitalWrite(MODULE_POWER_PIN,LOW);             				// Enable (LOW) the sensor
			if (result.wakeupPin() == BUTTON_PIN) {                         // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				Log.info("Woke with user button - LoRA State");
				state = LoRA_TRANSMISSION_STATE;
			}
			else if (result.wakeupPin() == INT_PIN) {
				Log.info("Woke with sensor interrupt");						// Will count at the bottom of the main loop
				if (secondsUntilNextEvent() <= 2 || \
					secondsUntilNextEvent() >= ((sysStatus.get_frequencyMinutes() * 60) - 2)) \
					state = IDLE_STATE;										// If more or less than 2 seconds we may miss reporting
				else state = SLEEPING_STATE;								// This is the normal behavioud
			}
			else {
				Log.info("Time is up at %s with %li free memory", Time.format((Time.now()+wakeInSeconds), "%T").c_str(), System.freeMemory());
				state = IDLE_STATE;
			}
		} break;

		case LoRA_TRANSMISSION_STATE: {
			bool result = false;

			if (state != oldState) {
				publishStateTransition();                   				// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				LoRA_Functions::instance().clearBuffer();
				takeMeasurements();

				// Based on Alert code, determine what message to send
				if (sysStatus.get_alertCodeNode() == 0) result = LoRA_Functions::instance().composeDataReportNode();
				else if (sysStatus.get_alertCodeNode() == 1) result = LoRA_Functions::instance().composeJoinRequesttNode();
				else Log.info("Alert code %d, will handle in ERROR state", sysStatus.get_alertCodeNode());

				if (!result) {
					retryState++;
					if (sysStatus.get_frequencyMinutes() == 1) retryState = 0;	// When we are in recovery mode, we don't need retries as we are sending every minute
					Log.info("Failed in data send, retryState = %d",retryState);
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
			bool messageReceived = false;

			if (state != oldState) {
				publishStateTransition();                   				// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				startListening = millis();
				state = SLEEPING_STATE;
			}

			while (millis() - startListening < 5000) {
				// The big difference between a node and a gateway - the Node initiates a LoRA exchange by sending data
				if (LoRA_Functions::instance().listenForLoRAMessageNode()) {// Listen for acknowledgement
					messageReceived = true;
					sysStatus.set_lastConnection(Time.now());
					randomSeed(sysStatus.get_lastConnection());				// Done so we can genrate rando numbers later
					ab1805.setRtcFromTime(Time.now());
					if (sysStatus.get_alertCodeNode() > 0) {				// If we are reporting an alert we will also report data
						state = IDLE_STATE;									// This is enable us to send the data now that alert is reported
					}
					else {
						static int lastReportingHour = Time.hour();
						if (Time.hour() != lastReportingHour) {
							current.set_hourlyCount(0);					    // Zero the hourly count
							lastReportingHour = Time.hour();
						}
					}
					LoRA_Functions::instance().sleepLoRaRadio();			// Done with LoRA - put radio to sleep
					break;
				}
			}
			if (!messageReceived) Log.info("Did not receive a response");
		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (sysStatus.get_alertCodeNode())
			{
			case 1:															// Case 1 is an unconfigured node - needs to send join request
				Log.info("Alert 1 - Join Request Required");
				state = LoRA_TRANSMISSION_STATE;							// Sends the alert and clears alert code
			break;
			case 2:															// Case 2 is for Time not synced
				Log.info("Alert 2- New Day Alert");
				state = LoRA_TRANSMISSION_STATE;							// Sends the alert and clears alert code
			break;
			case 3: {														// Case 3 is generic - power cycle device to recover from errors
				static system_tick_t enteredState = millis();
				if (millis() - enteredState > 30000L) {
					Log.info("Alert 3 - Resetting device");
					sysStatus.set_alertCodeNode(0);							// Need to clear so we don't get in a retry cycle
					sysStatus.set_alertTimestampNode(Time.now());
					sysStatus.flush(true);									// All this is required as we are done trainsiting loop
					delay(2000);
					ab1805.deepPowerDown();
				}
			} break;
			case 4: 														// In this state, we have retried sending - time to reinitilize the modem
				Log.info("Initialize LoRA radio");
				if(LoRA_Functions::instance().initializeRadio()) {
					Log.info("Initialization successful");	
					state = LoRA_TRANSMISSION_STATE;						// Sends the alert and clears alert code
				}
				else {
					Log.info(("Initialization not successful - power cycle"));
					sysStatus.set_alertCodeNode(3);							// Next time through - will transition to power cycle
					sysStatus.set_alertTimestampNode(Time.now());
					state = IDLE_STATE;
				}
			break;
			case 5:															// In this case, we will reset all data
				sysStatus.initialize();										// Resets the sysStatus values to factory default
				current.resetEverything();									// Resets the node counts
				sysStatus.set_alertCodeNode(1);								// Resetting system values requires we re-join the network
				sysStatus.set_alertTimestampNode(Time.now());			
				state = IDLE_STATE;	
			break;
			case 6: 														// In this state system data is retained but current data is reset
				current.resetEverything();
				sysStatus.set_alertCodeNode(0);
				state = SLEEPING_STATE;										// Once we clear the counts - go to sleep
			break;
			case 7:
				// This alert code is handled in the Data response function - on Data response Gateway type overwrites node type
				sysStatus.set_alertCodeNode(0);
				state = SLEEPING_STATE;										// Once we clear the counts - go to sleep
			break;
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
		Log.info("Resetting due to low memory");
		delay(2000);
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
	static time_t nextPeriodBegins = 0;
	// First we will calculate the adjustment to the wakeboundary due to node number and retries
	if (retryState) {														// We will use an exponential back-off for 3 retries
		int fiftyFifty = random(2);											// Flip a coin - seed in Setup()
		unsigned long offset = 10UL + NODENUMBEROFFSET * fiftyFifty * pow(2, retryState); // This is an exponential back-off - adding 10 seconds to increase odds
		Log.info("Node %d retry state %d dice of %d retry in %lu seconds", sysStatus.get_nodeNumber(), retryState, fiftyFifty, offset);
		if (retryState >= 3) {
			retryState = -1;							    				// Going to stop re-trying and wait until the next period
			sysStatus.set_alertCodeNode(4);									// This will trigger a reinitialization of the radio
			sysStatus.set_alertTimestampNode(Time.now());					// Alert Time stamp
		}
		return offset;
	}
	else if (Time.isValid()) {												// The general case - has to handle node-number specific timing
		unsigned long nodeSpecificOffset = NODENUMBEROFFSET * sysStatus.get_nodeNumber(); // Each node has an offset to avoid collisions
		unsigned long wakeBoundary = (sysStatus.get_frequencyMinutes() * 60UL);
		unsigned long secondsToReturn = constrain(wakeBoundary - Time.now() % wakeBoundary, 0UL, wakeBoundary);  // If Time is valid, we can compute time to the start of the next report window	
		if (sysStatus.get_nodeNumber() >= 11)  {							// We need to test here as the off-set can cause missed reporting if awoken by the sensor at the period boundary
			Log.info("Unconfigured node - no offset and %lu seconds till next period",secondsToReturn);
			return secondsToReturn;
		}	

		if (Time.now() - nextPeriodBegins >= 0) {							// We are in a new period
			secondsToReturn += nodeSpecificOffset;							// Off-set for configured nodes only
			nextPeriodBegins = Time.now() + secondsToReturn;	
			Log.info("Starting a new period %lu sec till %s", secondsToReturn, Time.format(nextPeriodBegins, "%T").c_str());
		}
		else if (nextPeriodBegins - Time.now() <= nodeSpecificOffset) {		// Not at new period yet but within the node offset
			secondsToReturn = nextPeriodBegins - Time.now();
			Log.info("In terminal phase %lu seconds till %s",secondsToReturn, Time.format(nextPeriodBegins, "%T").c_str() );
		}
		else {
			secondsToReturn += nodeSpecificOffset;							// Off-set for configured nodes only
			// Log.info("In current period %lu sec till %s", secondsToReturn, Time.format(nextPeriodBegins, "%T").c_str());
		}
		return secondsToReturn;
    }
	else return 60UL;	// If time is not valid, we need to keep trying to catch the Gateway when it next wakes up.
}