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
// v1.08 - Added logic for pressure and PIR sensor light control
// v1.09 - Changed startup behaviour - pressing the user button puts device into 60 second connect mode
// v3.00 - Updated to reset the device after connecting so there is not an attempt to reconnect on each wake. (Gateway v2)
// v4.00 - Makig the disconnect process cleaner and disabling watchdog during update
// v5.00 - Updates to improve reliability
// v8.00 - Fixed issue with unconstrained blinking - need to get to Pilot mountain - changed battery montiroing / center freq for stick antenna (v8 Node / v7 Gateway)
// v10.00 - Breaking Change - v9 Gateway required - Node reports RSSI / SNR to Gateway - Listening / repeating mode, Simplified error handling
// v12.00 - Debounce for button press-to transmit
// v13.00 - Adding code to ensure the device charges reliably - Removed PMIC - Fixed the internal temp to int8_t


#define NODENUMBEROFFSET 10000UL					// By how much do we off set each node by node number

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

PRODUCT_VERSION(13);									// For now, we are putting nodes and gateways in the same product group - need to deconflict #

// Prototype functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
void sensorISR();
bool disconnectFromParticle();						// Makes sure we are disconnected from Particle

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_TRANSMISSION_STATE, LoRA_LISTENING_STATE, LoRA_RETRY_WAIT_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[10][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA Transmit", "LoRA Listening", "LoRA Retry Wait", "Connecting", "Disconnecting", "Reporting"};
volatile State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
void outOfMemoryHandler(system_event_t event, int param);
void transmitDelayTimerISR();
void listeningDurationTimerISR();

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;

Timer transmitDelayTimer(10000,transmitDelayTimerISR,true);
Timer listeningDurationTimer(300000,listeningDurationTimerISR,true);

void setup() {

	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes

	sysStatus.setup();								// Initialize persistent storage
	current.setup();

	takeMeasurements();                             // Populates values so you can read them before the hour

	if (!digitalRead(BUTTON_PIN)) {											// We will use this to connect in order to get an update only
		Log.info("User button pressed at startup - attempt to connect");
		Particle.connect();													// Connects to Particle and stays connected until reset
		if (!waitFor(Particle.connected,600000)) {
			Log.info("Connection timeout - disconnect and reset");
			disconnectFromParticle();										// Times out after 10 minutes - then disconnects and continues
			System.reset();
		}
		else {
			Log.info("Connected - staying online for update");
 	    	unsigned long start = millis();
			while (millis() - start < (120 * 1000)) {							// Stay on-line for two minutes
				Particle.process();
			}
			disconnectFromParticle();
			System.reset();        				// You won't reach this point if there is an update but we need to reset to take the device back off-line
		}
	}
                              
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
  
    attachInterrupt(INT_PIN, sensorISR, RISING);                     		// PIR or Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,FALLING); 						// We may need to monitor the user switch to change behaviours / modes

	if (sysStatus.get_openHours()) sensorControl(sysStatus.get_sensorType(),true); // Turn the sensor on during open hours

	if (state == INITIALIZATION_STATE) state = SLEEPING_STATE;               	// Sleep unless otherwise from above code
  	Log.info("Startup complete for the Node with alert code %d and last connect %s", sysStatus.get_alertCodeNode(), Time.format(sysStatus.get_lastConnection(), "%T").c_str());
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup
}

void loop() {
	switch (state) {
		case IDLE_STATE: {													// Unlike most sketches - nodes spend most time in sleep and only transit IDLE once or twice each period
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;	// Error - let's handle this first
			else state = LoRA_LISTENING_STATE;  							// No error - Enter the LoRA state
		} break;

		case SLEEPING_STATE: {
			unsigned long wakeInSeconds, wakeBoundary;
			time_t time;

			publishStateTransition();              							// Publish state transition
			// How long to sleep
			if (Time.isValid()) {
				wakeBoundary = (sysStatus.get_frequencyMinutes() * 60UL);
				wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 0UL, wakeBoundary);  // If Time is valid, we can compute time to the start of the next report window	
				time = Time.now() + wakeInSeconds;
				Log.info("Sleep for %lu seconds until next event at %s with sensor %s", wakeInSeconds, Time.format(time, "%T").c_str(), (sysStatus.get_openHours()) ? "on" : "off");
			}
			else {
				wakeInSeconds = 60UL;
				Log.info("Time not valid, sleeping for 60 seconds");
			}
			// Turn things off to save power
			if (!sysStatus.get_openHours()) if (sysStatus.get_openHours()) sensorControl(sysStatus.get_sensorType(),false);
			// Configure Sleep
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration((wakeInSeconds) * 1000L);							// Configuring sleep
			ab1805.stopWDT();  												// No watchdogs interrupting our slumber
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                             // Wakey Wakey - WDT can resume
			sensorControl(sysStatus.get_sensorType(),true);					// Enable the sensor
			if (result.wakeupPin() == BUTTON_PIN) {                         // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				waitFor(Serial.isConnected, 10000);							// Wait for serial connection if we are using the button - we may want to monito serial 
				Log.info("Woke with user button");
				state = IDLE_STATE;
			}
			else if (result.wakeupPin() == INT_PIN) {
				Log.info("Woke with sensor interrupt");						// Will count at the bottom of the main loop
				state = SLEEPING_STATE;										// This is the normal behaviour
			}
			else {
				Log.info("Time is up at %s with %li free memory", Time.format((Time.now()+wakeInSeconds), "%T").c_str(), System.freeMemory());
				state = IDLE_STATE;
			}
		} break;

		case LoRA_LISTENING_STATE: {																// Timers will take us to transmit and back to sleep
			static int lastReportingHour = Time.hour();

			if (state != oldState) {
				if (oldState != LoRA_TRANSMISSION_STATE) {
					if (!listeningDurationTimer.isActive()) listeningDurationTimer.start();				// Don't reset timer if it is already running
					if (sysStatus.get_nodeNumber() < 11) transmitDelayTimer.changePeriod(sysStatus.get_nodeNumber()*NODENUMBEROFFSET);		// Wait a beat before transmitting
					else state = LoRA_TRANSMISSION_STATE;
				}
				publishStateTransition();                   										// Publish state transition
			}

			if (LoRA_Functions::instance().listenForLoRAMessageNode()) {							// Listen for LoRA signals - could be an acknowledgement or a message to relay to another node
				sysStatus.set_lastConnection(Time.now());											// Came back as true - message was for our node
				randomSeed(sysStatus.get_lastConnection() * sysStatus.get_nodeNumber());			// Done so we can genrate rando numbers later
				ab1805.setRtcFromTime(Time.now());
				if (Time.hour() != lastReportingHour) {
					current.set_hourlyCount(0);					    								// Zero the hourly count
					lastReportingHour = Time.hour();
				}
				else if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;					// Need to resolve alert before listening for others
			}
		} break;

		case LoRA_TRANSMISSION_STATE: {
			bool result = false;
			static int retryCount = 0;

			publishStateTransition();                   					// Let everyone know we are changing state
			takeMeasurements();												// Taking measurements now should allow for accurate battery measurements
			LoRA_Functions::instance().clearBuffer();
			// Based on Alert code, determine what message to send
			if (sysStatus.get_alertCodeNode() == 0) result = LoRA_Functions::instance().composeDataReportNode();
			else if (sysStatus.get_alertCodeNode() == 1 || sysStatus.get_alertCodeNode() == 2) result = LoRA_Functions::instance().composeJoinRequesttNode();
			else {
				Log.info("Alert code = %d",sysStatus.get_alertCodeNode());
				state = ERROR_STATE;
				break;														// Resolve the alert code in ERROR_STATE
			}		

			if (result) {
				retryCount = 0;												// Successful transmission - go listen for response
				state = LoRA_LISTENING_STATE;
			}
			else if (retryCount >= 3) {
				Log.info("Too many retries - giving up for this period");
				retryCount = 0;
				if ((Time.now() - sysStatus.get_lastConnection() > 2 * sysStatus.get_frequencyMinutes() * 60UL)) { 	// Device has not connected for two reporting periods
					Log.info("Nothing for two reporting periods - power cycle after current cycle");
					sysStatus.set_alertCodeNode(3);							// This will trigger a power cycle reset
					sysStatus.set_alertTimestampNode(Time.now());		
					sysStatus.set_lastConnection(Time.now());				// Prevents cyclical resets
					state = ERROR_STATE;									// Likely radio is locked up - reset the device and radio
					break;
				}
				state = LoRA_LISTENING_STATE;
			}
			else {
				Log.info("Transmission failed - retry number %d",retryCount++);
				state = LoRA_RETRY_WAIT_STATE;
			}
		} break;

		case LoRA_RETRY_WAIT_STATE: {										// In this state we introduce a random delay and then retransmit
			static unsigned long variableDelay = 0;
			static unsigned long startDelay = 0;

			if (state != oldState) {
				publishStateTransition();                   				// Publish state transition
				variableDelay = random(20000);								// a random delay up to 20 seconds
				startDelay = millis();
				Log.info("Going to retry in %lu seconds", variableDelay/1000UL);
			}

			if (millis() >= startDelay + variableDelay) state = LoRA_TRANSMISSION_STATE;

		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			if (state != oldState) publishStateTransition();                // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			switch (sysStatus.get_alertCodeNode()) {
			case 1:															// Case 1 is an unconfigured node - needs to send join request
				sysStatus.set_nodeNumber(11);
				Log.info("LoRA Radio initialized as an unconfigured node %i and a deviceID of %s", sysStatus.get_nodeNumber(), System.deviceID().c_str());
				state = LoRA_LISTENING_STATE;							// Sends the alert and clears alert code
			break;
			case 2:															// Case 2 is for Time not synced
				Log.info("Alert 2- Time is not valid going to join again");
				state = LoRA_LISTENING_STATE;							// Sends the alert and clears alert code
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
					sysStatus.set_alertCodeNode(0);							// Modem reinitialized successfully, going back to retransmit
					state = LoRA_LISTENING_STATE;							// Sends the alert and clears alert code
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
				Log.info("Full Reset and Re-Join Network");
				state = LoRA_LISTENING_STATE;							// Sends the alert and clears alert code

			break;
			case 6: 														// In this state system data is retained but current data is reset
				current.resetEverything();
				sysStatus.set_alertCodeNode(0);
				state = LoRA_LISTENING_STATE;								// Once we clear the counts we can go back to listening
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

	if (userSwitchDectected) {
		delay(100);									// Debounce the button press
		userSwitchDectected = false;				// Clear the interrupt flag
		if (!listeningDurationTimer.isActive()) listeningDurationTimer.start();				// Don't reset timer if it is already running
		Log.info("Detected button press");
		state = LoRA_TRANSMISSION_STATE;
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

void transmitDelayTimerISR() {
	state = LoRA_TRANSMISSION_STATE;										// Time for our node to transmit
}

void listeningDurationTimerISR() {
	LoRA_Functions::instance().sleepLoRaRadio();							// Done with the radio - shut it off
	state = SLEEPING_STATE;													// Go back to sleep
}

void userSwitchISR() {
	userSwitchDectected = true;
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

bool disconnectFromParticle()                      							// Ensures we disconnect cleanly from Particle
                                                                       		// Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  time_t startTime = Time.now();
  Log.info("In the disconnect from Particle function");
  Particle.disconnect();                                               		// Disconnect from Particle
  waitForNot(Particle.connected, 15000);                               		// Up to a 15 second delay() 
  Particle.process();
  if (Particle.connected()) {                      							// As this disconnect from Particle thing can be a·syn·chro·nous, we need to take an extra step to wait, 
    Log.info("Failed to disconnect from Particle");
    return(false);
  }
  else Log.info("Disconnected from Particle in %i seconds", (int)(Time.now() - startTime));
  // Then we need to disconnect from Cellular and power down the cellular modem
  startTime = Time.now();
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  Particle.process();
  if (Cellular.isOn()) {                                               // At this point, if cellular is not off, we have a problem
    Log.info("Failed to turn off the Cellular modem");
    return(false);                                                     // Let the calling function know that we were not able to turn off the cellular modem
  }
  else {
    Log.info("Turned off the cellular modem in %i seconds", (int)(Time.now() - startTime));
    return true;
  }
}
