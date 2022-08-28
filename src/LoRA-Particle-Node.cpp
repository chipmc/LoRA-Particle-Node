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
#include "LocalTimeRK.h"					        // https://rickkas7.github.io/LocalTimeRK/
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
LocalTimeSchedule publishSchedule;					// These allow us to enable a schedule and to use local time
LocalTimeConvert localTimeConvert_NOW;

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;

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

	initializeLoRA();								// Start the LoRA radio

	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	localTimeConvert_NOW.withCurrentTime().convert();  				        // Convert to local time for use later
  	publishSchedule.withMinuteOfHour(sysStatus.frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("22:59:59")));	 // Publish every 15 minutes from 6am to 10pm

  	Log.info("Startup complete at %s with battery %4.2f", localTimeConvert_NOW.format(TIME_FORMAT_ISO8601_FULL).c_str(), System.batteryCharge());

  	attachInterrupt(BUTTON_PIN,userSwitchISR,CHANGE); // We may need to monitor the user switch to change behaviours / modes

	if (state == INITIALIZATION_STATE) state = LoRA_STATE;  // This is not a bad way to start - could also go to the LoRA_STATE
}


void loop() {
	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			if (publishSchedule.isScheduledTime()) state = LoRA_STATE;		   // See Time section in setup for schedule
		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			int wakeInSeconds = secondsUntilNextEvent()-10;  		   		   // Subtracting ten seconds to reduce prospect of round tripping to IDLE
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
			static system_tick_t startLoRAWindow = 0;

			if (state != oldState) {
				publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				takeMeasurements();
				startLoRAWindow = millis();              	// Mark when we enter this state - for timeouts
				int randomDelay = random(10);
				Log.info("Node is preparing to send report with a randomDelay of %i", randomDelay);
				delay(randomDelay *1000);
				if (!composeDataReportNode())	{						// Initiate send data report
					sysStatus.frequencyMinutes = 1;						// Rescue mode
					Log.info("Send failed - going to send every minute");
					publishSchedule.withMinuteOfHour(sysStatus.frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("21:59:59")));	 // Publish every 15 minutes from 6am to 10pm
					state = IDLE_STATE;
				}
			} 
			// The big difference between a node and a gateway - the Node initiates a LoRA exchance by sending data
			if (receiveAcknowledmentDataReportNode()) {					// Listen for acknowledgement
				publishSchedule.withMinuteOfHour(sysStatus.frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("21:59:59")));	 // Publish every 15 minutes from 6am to 10pm
				current.hourly = 0;							// Zero the hourly count
				state = IDLE_STATE;
			}

			if ((millis() - startLoRAWindow) > 300000L) state = IDLE_STATE;	// This is a fail safe to make sure an off-line client won't prevent gatewat from checking in - and setting its clock
		} break;
	}

	ab1805.loop();                                  // Keeps the RTC synchronized with the Boron's clock

    storageObjectLoop();                            // Compares current system and current objects and stores if the hash changes (once / second) in storage_objects.h
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
			long unsigned secondsToReturn = constrain(localTimeConvert_NEXT.time - localTimeConvert_NOW.time, 0L, 86400L);	// Constrain to positive seconds less than or equal to a day.
        	Log.info("time of next event is: %s which is %lu seconds away", localTimeConvert_NEXT.format(TIME_FORMAT_DEFAULT).c_str(), secondsToReturn);
			return secondsToReturn;
		}
		else return 0;
    }
	else return 0;
}