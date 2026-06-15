/*
 * Project LoRA-Particle-Node - basic example of a remote counter
 * Description: Will send counts each our to server to be relayed to Particle / Ubidots
 * Author: Chip McClelland
 * Date: 7-25-22
 */

// Current release: v25.00
// Historical release notes live in CHANGELOG.md.


#define NODENUMBEROFFSET 10000UL					// By how much do we off set each node by node number

// Particle Libraries
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "Particle.h"                               // Because it is a CPP file not INO
#include <math.h>
// Application Libraries / Class Files
#include "config.h"
#include "LoRA_Functions.h"							// Where we store all the information on our LoRA implementation - application specific not a general library
#include "device_pinout.h"							// Define pinouts and initialize them
#include "energy_trend.h"
#include "node_time.h"
#include "power_management.h"
#include "take_measurements.h"						// Manages interactions with the sensors (default is temp for charging)
#include "MyPersistentData.h"						// Persistent Storage

// Prototypes and System Mode calls
SYSTEM_MODE(MANUAL);                        // This will enable user code to start executing automatically.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// For monitoring / debugging, you have some options on the next few lines - uncomment one
SerialLogHandler logHandler(LOG_LEVEL_INFO);     // Easier to see the program flow

PRODUCT_VERSION(NODE_FIRMWARE_RELEASE);					// For now, we are putting nodes and gateways in the same product group - need to deconflict #

#ifndef ENABLE_MESH_RELAY_LISTEN_WINDOW
#define ENABLE_MESH_RELAY_LISTEN_WINDOW 0
#endif

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

bool isSupportedSensorType(uint8_t sensorType);
bool shouldSensorBePowered();
bool isLoRaWindowState(State candidateState);
void updateSensorPowerState(bool enableSensor, const char *reason);
void stopLoRaWindowTimers();
void prepareForGatewayAckSleep();
const char *sleepTimeSourceLabel();
time_t nextScheduledWakeEpoch(time_t nowEpoch, time_t anchorEpoch, unsigned long wakeBoundary);
void logSleepCalculation(time_t nowEpoch, time_t targetEpoch, unsigned long wakeInSeconds);
bool requestLoRaTransmit(const char *reason);
String formatUtcDateTime(time_t epochUtc);
void logBlockedTransmitReject(const char *reason);

// Initialize Functions
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
void outOfMemoryHandler(system_event_t event, int param);
void transmitDelayTimerISR();
void listeningDurationTimerISR();

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;
volatile bool transmitDelayPending = false;
volatile bool listeningWindowExpired = false;
bool sensorPowerEnabled = false;
bool sensorPowerStateKnown = false;
uint8_t consecutiveFailedReportCycles = 0;
bool discoveryModeActive = false;
uint16_t discoverySleepCycles = 0;
bool sustainedFailureAlert3Pending = false;
bool deepPowerDownAttemptedThisBoot = false;
system_tick_t errorStateEnteredAt = 0;
uint8_t errorStateAlertCode = 0;
bool ackSyncOccurredThisCycle = false;
volatile bool loraTransactionActive = false;
volatile uint32_t loraTransactionGuardRejectCount = 0;
volatile uint32_t loraTransactionDeadlockRecoveryCount = 0;
volatile uint32_t sameStateTransitionCount = 0;
State lastBlockedTransmitState = INITIALIZATION_STATE;
bool lastBlockedTransmitStateKnown = false;

const unsigned long DISCOVERY_SLEEP_SECONDS = 7UL * 60UL;
const unsigned long DISCOVERY_STALE_ACK_SECONDS = 3UL * 60UL * 60UL;
const uint8_t LORA_REINIT_AFTER_FAILED_CYCLES = 2;
const uint8_t LORA_DEEP_POWERDOWN_AFTER_FAILED_CYCLES = 4;

Timer transmitDelayTimer(10000,transmitDelayTimerISR,true);
Timer listeningDurationTimer(300000,listeningDurationTimerISR,true);

String formatUtcDateTime(time_t epochUtc) {
	if (epochUtc <= 0) {
		return "invalid";
	}
	struct tm utcTm;
	gmtime_r(&epochUtc, &utcTm);
	char buffer[32];
	if (strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &utcTm) == 0) {
		return "invalid";
	}
	return String(buffer);
}

bool requestLoRaTransmit(const char *reason) {
	if (state == LoRA_TRANSMISSION_STATE) {
		sameStateTransitionCount++;
		Log.warn("Illegal same-state transition request: %s reason=%s count=%lu", stateNames[state], reason ? reason : "unknown", (unsigned long)sameStateTransitionCount);
		return false;
	}
	const bool internalRetryAdvance = loraTransactionActive && state == LoRA_RETRY_WAIT_STATE;
	if (loraTransactionActive && !internalRetryAdvance) {
		loraTransactionGuardRejectCount++;
		logBlockedTransmitReject(reason);
		return false;
	}
	state = LoRA_TRANSMISSION_STATE;
	return true;
}

void logBlockedTransmitReject(const char *reason) {
	const bool isFirstReject = (loraTransactionGuardRejectCount == 1UL);
	const bool isThousandReject = ((loraTransactionGuardRejectCount % 1000UL) == 0UL);
	const bool isStateChange = (!lastBlockedTransmitStateKnown || lastBlockedTransmitState != state);

	if (isFirstReject || isThousandReject || isStateChange) {
		Log.warn("Blocked transmit storm: state=%s reason=%s rejects=%lu", stateNames[state], reason ? reason : "unknown", (unsigned long)loraTransactionGuardRejectCount);
	}

	lastBlockedTransmitState = state;
	lastBlockedTransmitStateKnown = true;
}

bool isSupportedSensorType(uint8_t sensorType) {
	return sensorType == 0 || sensorType == 1;
}

bool shouldSensorBePowered() {
	return sysStatus.get_openHours() && isSupportedSensorType(sysStatus.get_sensorType());
}

bool isLoRaWindowState(State candidateState) {
	return candidateState == LoRA_LISTENING_STATE || candidateState == LoRA_TRANSMISSION_STATE || candidateState == LoRA_RETRY_WAIT_STATE;
}

void updateSensorPowerState(bool enableSensor, const char *reason) {
	if (!sensorPowerStateKnown || sensorPowerEnabled != enableSensor) {
		Log.info(
			"Sensor power %s (%s): openHours=%s sensorType=%u",
			enableSensor ? "enabled" : "disabled",
			reason,
			sysStatus.get_openHours() ? "true" : "false",
			sysStatus.get_sensorType());
		sensorPowerEnabled = enableSensor;
		sensorPowerStateKnown = true;
	}
	sensorControl(sysStatus.get_sensorType(), enableSensor);
}

void stopLoRaWindowTimers() {
	transmitDelayTimer.stop();
	listeningDurationTimer.stop();
	transmitDelayPending = false;
	listeningWindowExpired = false;
	loraTransactionActive = false;
}

void stopListeningDurationTimer() {
	listeningDurationTimer.stop();
	listeningWindowExpired = false;
}

void prepareForGatewayAckSleep() {
	current.flush(true);
	sysStatus.flush(true);
	LoRA_Functions::instance().sleepLoRaRadio();
	stopLoRaWindowTimers();
	state = SLEEPING_STATE;
}

const char *sleepTimeSourceLabel() {
	if (!Time.isValid()) {
		return "invalid";
	}
	if (ackSyncOccurredThisCycle) {
		return "ack";
	}
	if (nodeTimeIsProvisional()) {
		return "rtc-provisional";
	}
	if (nodeTimeHasAuthoritativeAck()) {
		return "authoritative";
	}
	return "system";
}

time_t nextScheduledWakeEpoch(time_t nowEpoch, time_t anchorEpoch, unsigned long wakeBoundary) {
	(void)anchorEpoch;  // Unused - retained for API compatibility
	
	if (wakeBoundary == 0) {
		return nowEpoch;
	}

	// Fixed wall-clock boundary alignment based on frequencyMinutes
	// Always aligns to the next boundary (e.g., :00:00 for hourly schedules)
	// to prevent drift accumulation from ACK processing delays.
	// 
	// This implementation ONLY supports fixed-period schedules where the wake
	// boundary is a simple modulo of epoch time (e.g., hourly, every N minutes).
	// It does NOT support arbitrary anchor times or offset schedules.
	//
	// Example: nowEpoch = 07:15:42, wakeBoundary = 3600 (60 min)
	//   => nextWake = 08:00:00 (strips seconds, aligns to hour)
	return nowEpoch + (time_t)(wakeBoundary - ((unsigned long)nowEpoch % wakeBoundary));
}

void logSleepCalculation(time_t nowEpoch, time_t targetEpoch, unsigned long wakeInSeconds) {
	String nowUtc = formatUtcDateTime(nowEpoch);
	String nextWakeUtc = formatUtcDateTime(targetEpoch);

	Log.info(
		"SleepCalc: nowUtc=%s nextWakeUtc=%s sleepSeconds=%lu",
		nowUtc.c_str(),
		nextWakeUtc.c_str(),
		wakeInSeconds);
}

void setup() {
	waitFor(Serial.isConnected, 10000);				// Wait for serial connection

    initializePinModes();                           // Sets the pinModes
	ab1805.withFOUT(D8).setup();                      // Restore system time from the RTC before any time-based logic runs
	nodeTimeNoteBootRtcState(Time.isValid() && ab1805.isRTCSet());
 	if (Time.isValid()) {
		Log.info("TimeSync: bootUtc=%s", formatUtcDateTime(Time.now()).c_str());
	}

	sysStatus.setup();								// Initialize persistent storage
	current.setup();
	EnergyTrend::instance().setup();
	PowerManager::instance().setup();
	PowerManager::instance().beginWakeCycle(false);
	EnergyTrend::instance().noteWake();
	ackSyncOccurredThisCycle = false;

	takeMeasurements();                             // Populates values so you can read them before the hour

	if (!digitalRead(BUTTON_PIN)) {											// We will use this to connect in order to get an update only
		Log.info("User button pressed at startup - attempt to connect");
		Particle.connect();													// Connects to Particle and stays connected until reset
		if (!waitFor(Particle.connected,600000)) {
			Log.info("Connection timeout - disconnect and reset");
			EnergyTrend::instance().noteConnectionFailure();
			disconnectFromParticle();										// Times out after 10 minutes - then disconnects and continues
			EnergyTrend::instance().noteResetImminent();
			System.reset();
		}
		else {
			Log.info("Connected - staying online for update");
	    	uint32_t connectedAtMs = millis();
 	    	unsigned long start = millis();
			while (millis() - start < (120 * 1000)) {							// Stay on-line for two minutes
				Particle.process();
			}
			EnergyTrend::instance().noteConnection(millis() - connectedAtMs);
			disconnectFromParticle();
			EnergyTrend::instance().noteResetImminent();
			System.reset();        				// You won't reach this point if there is an update but we need to reset to take the device back off-line
		}
	}
                              
    ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);	// Enable watchdog

	System.on(out_of_memory, outOfMemoryHandler);   // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	// In this section we test for issues and set alert codes as needed
	if (! LoRA_Functions::instance().setup(false)) 	{						// Start the LoRA radio - Node
		sysStatus.set_alertCodeNode(3);									// Initialization failure
		sysStatus.set_alertTimestampNode(Time.now());
		Log.info("LoRA Initialization failure alert code %d - power cycle in 30", sysStatus.get_alertCodeNode());
	}
	else if (sysStatus.get_nodeNumber() > 10 || !Time.isValid()) {			// If the node number indicates this node is uninitialized or the clock needs to be set, initiate a join request
		sysStatus.set_alertCodeNode(1); 								// Will initiate a join request
		if (!Time.isValid()) Log.info("Startup detected invalid time - setting alert code to %d for join request", sysStatus.get_alertCodeNode());
		if (sysStatus.get_nodeNumber() > 10) Log.info("Startup detected unconfigured node number %d - setting alert code to %d", sysStatus.get_nodeNumber(), sysStatus.get_alertCodeNode());
	}

  	takeMeasurements();                                                  	// Populates values so you can read them before the hour
 
    attachInterrupt(INT_PIN, sensorISR, RISING);                     		// PIR or Pressure Sensor interrupt from low to high
	attachInterrupt(BUTTON_PIN,userSwitchISR,FALLING); 						// We may need to monitor the user switch to change behaviours / modes

	updateSensorPowerState(shouldSensorBePowered(), "startup");

	if (state == INITIALIZATION_STATE) state = SLEEPING_STATE;               	// Sleep unless otherwise from above code
	Log.info("Startup %s complete: alert=%d lastConnect=%s", NODE_FIRMWARE_RELEASE_LABEL, sysStatus.get_alertCodeNode(), Time.format(sysStatus.get_lastConnection(), "%T").c_str());
	Log.info("Startup diagnostics: transactionRecoveries=%lu", (unsigned long)loraTransactionDeadlockRecoveryCount);
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup
}

void loop() {
	static State timerManagedState = INITIALIZATION_STATE;
	static bool loggedStaleListeningWindowExpired = false;
	static bool previousTimeValid = false;

	if (loraTransactionActive && !isLoRaWindowState(state)) {
		loraTransactionActive = false;
		Log.warn("Cleared stale LoRa transaction guard outside LoRa window (state=%s)", stateNames[state]);
	}

	bool timeValid = Time.isValid();
	if (timeValid != previousTimeValid) {
		if (timeValid) {
			Log.info("TimeSync: valid utc=%s", formatUtcDateTime(Time.now()).c_str());
		}
		else {
			Log.warn("TimeSync: invalid");
		}
		previousTimeValid = timeValid;
	}

	if (state != timerManagedState) {
		if (timerManagedState == LoRA_LISTENING_STATE && state != LoRA_LISTENING_STATE) {
			stopListeningDurationTimer();
		}
		if (isLoRaWindowState(timerManagedState) && !isLoRaWindowState(state)) {
			stopLoRaWindowTimers();
		}
		timerManagedState = state;
	}

	if (listeningWindowExpired && state == LoRA_LISTENING_STATE) {
		LoRA_Functions::instance().sleepLoRaRadio();
		stopLoRaWindowTimers();
		state = SLEEPING_STATE;
	}
	else if (listeningWindowExpired) {
		if (!loggedStaleListeningWindowExpired) {
			Log.warn("Ignoring stale listening window expiry while in %s", stateNames[state]);
			loggedStaleListeningWindowExpired = true;
		}
		listeningWindowExpired = false;
	}
	else if (transmitDelayPending && state == LoRA_LISTENING_STATE) {
		transmitDelayPending = false;
		stopListeningDurationTimer();
		requestLoRaTransmit("transmit-delay-expired");
	}
	else if (transmitDelayPending) {
		transmitDelayPending = false;
	}

	if (!listeningWindowExpired) {
		loggedStaleListeningWindowExpired = false;
	}

	switch (state) {
		case IDLE_STATE: {													// Unlike most sketches - nodes spend most time in sleep and only transit IDLE once or twice each period
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;	// Error - let's handle this first
			else state = LoRA_LISTENING_STATE;  							// No error - Enter the LoRA state
		} break;

		case SLEEPING_STATE: {
			unsigned long wakeInSeconds, wakeBoundary;
			time_t targetEpoch = 0;
			time_t nowEpoch = Time.isValid() ? Time.now() : 0;
			bool usingDiscoverySleep = false;
			const char *discoveryReason = nullptr;
			time_t lastConnection = sysStatus.get_lastConnection();
			time_t ackAgeSeconds = 0;

			if (state != oldState) publishStateTransition();              							// Publish state transition
			if (!Time.isValid()) discoveryReason = "time invalid";
			else if (lastConnection <= 0) discoveryReason = "no successful gateway ACK";
			else {
				ackAgeSeconds = nowEpoch - lastConnection;
				if (ackAgeSeconds >= (time_t) DISCOVERY_STALE_ACK_SECONDS) discoveryReason = "stale gateway ACK";
			}

			if (discoveryReason) {
				const PowerReport &powerReport = PowerManager::instance().lastReport();
				double stateOfCharge = current.get_stateOfCharge();
				if (!powerReport.policy.shouldAllowDiscoveryMode) {
					discoveryModeActive = false;
					discoverySleepCycles = 0;
					if (powerReport.reading.socStatus != PowerAvailability::Valid) Log.info("Discovery mode skipped - SOC %s, using normal sleep", PowerManager::availabilityLabel(powerReport.reading.socStatus));
					else Log.info("Discovery mode skipped - SOC %.2f below policy threshold, using normal sleep", stateOfCharge);
				}
				else {
					unsigned long ackAgeMinutes = (unsigned long)(ackAgeSeconds / 60);
					if (!discoveryModeActive) {
						if (ackAgeSeconds > 0) Log.info("Discovery mode entered - %s (%lu minutes since ACK), SOC %.2f", discoveryReason, ackAgeMinutes, stateOfCharge);
						else Log.info("Discovery mode entered - %s, SOC %.2f", discoveryReason, stateOfCharge);
					}
					discoveryModeActive = true;
					discoverySleepCycles++;
					wakeInSeconds = DISCOVERY_SLEEP_SECONDS;
					targetEpoch = nowEpoch + (time_t)wakeInSeconds;
					usingDiscoverySleep = true;
					Log.info("Discovery sleep interval %u - sleeping for %lu seconds", discoverySleepCycles, wakeInSeconds);
				}
			}
			// How long to sleep
			if (!usingDiscoverySleep && Time.isValid()) {
				wakeBoundary = (sysStatus.get_frequencyMinutes() * 60UL);
				if (sysStatus.get_openHours()) {
					targetEpoch = nextScheduledWakeEpoch(nowEpoch, lastConnection, wakeBoundary);
					wakeInSeconds = (unsigned long)(targetEpoch - nowEpoch);
				}
				else {
					wakeInSeconds = wakeBoundary;
					targetEpoch = nowEpoch + (time_t)wakeInSeconds;
				}
				logSleepCalculation(nowEpoch, targetEpoch, wakeInSeconds);
				if (sysStatus.get_openHours()) {
					Log.info("Sleep for %lu seconds until next boundary-aligned event at %s with sensor %s", wakeInSeconds, Time.format(targetEpoch, "%T").c_str(), "on");
				}
				else {
					Log.info("Closed-hours relative sleep for %u minutes until %s", sysStatus.get_frequencyMinutes(), Time.format(targetEpoch, "%T").c_str());
				}
			}
			else if (!usingDiscoverySleep) {
				wakeInSeconds = 60UL;
				logSleepCalculation(0, 0, wakeInSeconds);
				Log.info("Time not valid, sleeping for 60 seconds");
			}
			else {
				logSleepCalculation(nowEpoch, targetEpoch, wakeInSeconds);
			}
			// Turn things off to save power
			updateSensorPowerState(shouldSensorBePowered(), "pre-sleep");
			// Configure Sleep
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration((wakeInSeconds) * 1000L);							// Configuring sleep
			EnergyTrend::instance().noteSleepEntry();
			ab1805.stopWDT();  												// No watchdogs interrupting our slumber
			// Drain serial buffer if USB connected to prevent split log messages
			if (Serial.isConnected()) {
				Serial.flush();
				delay(75);  												// Allow USB CDC transmission to complete
			}
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                             // Wakey Wakey - WDT can resume
			PowerManager::instance().beginWakeCycle(true);
			EnergyTrend::instance().noteWake();
			ackSyncOccurredThisCycle = false;
			updateSensorPowerState(shouldSensorBePowered(), "post-wake");
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
				LoRA_Functions::instance().logListeningStateEntry();
				if (!listeningDurationTimer.isActive()) listeningDurationTimer.start();				// Bound the direct-to-gateway ACK wait window
				if (oldState != LoRA_TRANSMISSION_STATE) {
					if (sysStatus.get_nodeNumber() < 11) transmitDelayTimer.changePeriod(sysStatus.get_nodeNumber()*NODENUMBEROFFSET);		// Wait a beat before transmitting
					else requestLoRaTransmit("listening-unconfigured-node");
				}
				publishStateTransition();                   										// Publish state transition
			}

			if (LoRA_Functions::instance().listenForLoRAMessageNode()) {							// Listen for LoRA signals - could be an acknowledgement or a message to relay to another node
				ackSyncOccurredThisCycle = true;
				if (sysStatus.get_lastConnection() > 0) {
					Log.info(
					"ACK TimeSync: gatewayUtc=%s nodeUtcBefore=%s driftSeconds=%+ld scheduleIntervalMinutes=%u",
						formatUtcDateTime(nodeTimeLastGatewayAckUtc()).c_str(),
						formatUtcDateTime(nodeTimeLastNodeUtcBeforeAck()).c_str(),
						(long)nodeTimeLastAckDriftSeconds(),
						sysStatus.get_frequencyMinutes());
				}
				Log.info(
					"ACK Diagnostics: transactionActive=%s blockedRejects=%lu deadlockRecoveries=%lu",
					loraTransactionActive ? "true" : "false",
					(unsigned long)loraTransactionGuardRejectCount,
					(unsigned long)loraTransactionDeadlockRecoveryCount);
				if (discoveryModeActive || discoverySleepCycles != 0) Log.info("Discovery mode exit after ACK after %u discovery sleeps", discoverySleepCycles);
				discoveryModeActive = false;
				discoverySleepCycles = 0;
				consecutiveFailedReportCycles = 0;
				sustainedFailureAlert3Pending = false;
				randomSeed(sysStatus.get_lastConnection() * sysStatus.get_nodeNumber());			// Done so we can genrate rando numbers later
				if (Time.hour() != lastReportingHour) {
					current.set_hourlyCount(0);					    								// Zero the hourly count
					lastReportingHour = Time.hour();
				}
				if (sysStatus.get_alertCodeNode() != 0) state = ERROR_STATE;						// Need to resolve alert before sleeping or listening for others
				#if !ENABLE_MESH_RELAY_LISTEN_WINDOW
				else prepareForGatewayAckSleep();
				#endif
			}
		} break;

		case LoRA_TRANSMISSION_STATE: {
			bool result = false;
			static int retryCount = 0;

			if (state != oldState) publishStateTransition();                   					// Let everyone know we are changing state
			if (!loraTransactionActive) {
				loraTransactionActive = true;
			}
			takeMeasurements();												// Taking measurements now should allow for accurate battery measurements
			LoRA_Functions::instance().clearBuffer();
			// Based on Alert code, determine what message to send
			if (sysStatus.get_alertCodeNode() == 0) result = LoRA_Functions::instance().composeDataReportNode();
			else if (sysStatus.get_alertCodeNode() == 1 || sysStatus.get_alertCodeNode() == 2) result = LoRA_Functions::instance().composeJoinRequesttNode();
			else {
				Log.info("Alert code = %d",sysStatus.get_alertCodeNode());
				loraTransactionActive = false;
				state = ERROR_STATE;
				break;														// Resolve the alert code in ERROR_STATE
			}		

			if (result) {
				retryCount = 0;												// Successful transmission - go listen for response
				state = LoRA_LISTENING_STATE;
			}
			else if (retryCount >= 3) {
				Log.info("Too many retries - gateway unavailable, sleeping until next window");
				retryCount = 0;
				consecutiveFailedReportCycles++;
				Log.info("Sustained failure counter %u, reinit at %u, recovery powerdown at %u", consecutiveFailedReportCycles, LORA_REINIT_AFTER_FAILED_CYCLES, LORA_DEEP_POWERDOWN_AFTER_FAILED_CYCLES);
				if (consecutiveFailedReportCycles >= LORA_DEEP_POWERDOWN_AFTER_FAILED_CYCLES) {
					if (deepPowerDownAttemptedThisBoot) {
						Log.info("Sustained failure threshold reached again, but recovery powerdown already attempted this boot");
					}
					else {
						deepPowerDownAttemptedThisBoot = true;
						sustainedFailureAlert3Pending = true;
						Log.info("Sustained failure reached %u cycles - escalating to Alert 3 recovery", consecutiveFailedReportCycles);
						sysStatus.set_alertCodeNode(3);
						sysStatus.set_alertTimestampNode(Time.now());
						state = ERROR_STATE;
						Log.info("ACK Timeout Recovery: msg=%u txActive=%s nextState=%s",
							current.get_messageCount(),
							loraTransactionActive ? "true" : "false",
							stateNames[state]);
						break;
					}
				}
				else if (consecutiveFailedReportCycles >= LORA_REINIT_AFTER_FAILED_CYCLES) {
					Log.info("Sustained failure reached %u cycles - reinitializing LoRa radio before sleep", consecutiveFailedReportCycles);
					if (LoRA_Functions::instance().initializeRadio()) Log.info("LoRa radio reinitialized after sustained failure");
					else Log.info("LoRa radio reinitialization failed after sustained failure");
				}
				current.flush(true);
				sysStatus.flush(true);
				LoRA_Functions::instance().sleepLoRaRadio();
				stopLoRaWindowTimers();									// NOTE: This clears loraTransactionActive
				state = SLEEPING_STATE;
				Log.info("ACK Timeout Recovery: msg=%u txActive=%s nextState=%s",
					current.get_messageCount(),
					loraTransactionActive ? "true" : "false",
					stateNames[state]);
			}
			else {
				Log.info("Transmission failed - retry number %d",retryCount++);
				state = LoRA_RETRY_WAIT_STATE;
				// NOTE: loraTransactionActive remains true during retry - will be cleared if transition out of LoRA window states
				Log.info("ACK Timeout Recovery: msg=%u txActive=%s nextState=%s",
					current.get_messageCount(),
					loraTransactionActive ? "true" : "false",
					stateNames[state]);
			}
		} break;

		case LoRA_RETRY_WAIT_STATE: {										// In this state we introduce a random delay and then retransmit
			static unsigned long variableDelay = 0;
			static unsigned long startDelay = 0;

			if (state != oldState) {
				publishStateTransition();                   				// Publish state transition
				variableDelay = 2000 + random(18000);						// a random delay from 2 to 20 seconds
				startDelay = millis();
				Log.info("Going to retry in %lu seconds", variableDelay/1000UL);
			}

			if (loraTransactionActive) {
				const unsigned long elapsedMs = millis() - startDelay;
				const unsigned long recoveryThresholdMs = variableDelay + 10000UL;
				if (elapsedMs > recoveryThresholdMs) {
					loraTransactionDeadlockRecoveryCount++;
					loraTransactionActive = false;
					Log.warn(
						"RetryWait deadlock recovery triggered: elapsed=%lu retryDelay=%lu recoveries=%lu",
						elapsedMs / 1000UL,
						variableDelay / 1000UL,
						(unsigned long)loraTransactionDeadlockRecoveryCount);
					state = LoRA_TRANSMISSION_STATE;
					break;
				}
			}

			if (millis() >= startDelay + variableDelay) requestLoRaTransmit("retry-delay-expired");

		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			if (state != oldState) {
				publishStateTransition();                // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				errorStateEnteredAt = millis();
				errorStateAlertCode = sysStatus.get_alertCodeNode();
			}
			else if (errorStateAlertCode != sysStatus.get_alertCodeNode()) {
				errorStateEnteredAt = millis();
				errorStateAlertCode = sysStatus.get_alertCodeNode();
			}

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
				if (millis() - errorStateEnteredAt > 30000L) {
					if (sustainedFailureAlert3Pending) {
						const PowerReport &powerReport = PowerManager::instance().lastReport();
						double stateOfCharge = current.get_stateOfCharge();
						if (!powerReport.policy.shouldAllowRecoveryPowerdown) {
							if (powerReport.reading.socStatus != PowerAvailability::Valid) Log.info("Sustained failure recovery powerdown skipped - SOC %s", PowerManager::availabilityLabel(powerReport.reading.socStatus));
							else Log.info("Sustained failure recovery powerdown skipped - SOC %.2f below policy threshold", stateOfCharge);
							sustainedFailureAlert3Pending = false;
							sysStatus.set_alertCodeNode(0);
							sysStatus.set_alertTimestampNode(Time.now());
							current.flush(true);
							sysStatus.flush(true);
							LoRA_Functions::instance().sleepLoRaRadio();
							stopLoRaWindowTimers();
							state = SLEEPING_STATE;
							break;
						}
						Log.info("Alert 3 - Sustained LoRa failure recovery powerdown, SOC %.2f", stateOfCharge);
						sustainedFailureAlert3Pending = false;
					}
					else Log.info("Alert 3 - Resetting device");
					sysStatus.set_alertCodeNode(0);							// Need to clear so we don't get in a retry cycle
					sysStatus.set_alertTimestampNode(Time.now());
					sysStatus.flush(true);									// All this is required as we are done trainsiting loop
					EnergyTrend::instance().noteResetImminent("alert3");
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
		EnergyTrend::instance().noteResetImminent("oom");
		delay(2000);
		System.reset();
  	}

	if (userSwitchDectected) {
		delay(100);									// Debounce the button press
		userSwitchDectected = false;				// Clear the interrupt flag
		if (loraTransactionActive) {
			loraTransactionGuardRejectCount++;
			logBlockedTransmitReject("user-button");
		}
		else {
			if (!listeningDurationTimer.isActive()) listeningDurationTimer.start();				// Don't reset timer if it is already running
			Log.info("Detected button press");
			requestLoRaTransmit("user-button");
		}
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
	if (state == oldState) {
		sameStateTransitionCount++;
		Log.warn("Illegal same-state transition: %s count=%lu", stateNames[state], (unsigned long)sameStateTransitionCount);
		return;
	}
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
	if (state == LoRA_LISTENING_STATE) {
		transmitDelayPending = true;
	}
}

void listeningDurationTimerISR() {
	if (isLoRaWindowState(state)) {
		listeningWindowExpired = true;
	}
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
