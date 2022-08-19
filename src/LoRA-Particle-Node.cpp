/*
 * Project LoRA-Particle-Node - basic example of a remote counter
 * Description: Will send counts each our to server to be relayed to Particle / Ubidots
 * Author: Chip McClelland
 * Date: 7-25-22
 */

// Version list 
// v1 - initial attempt - no sleep


// Included libraries
#include <RHMesh.h>
#include <RH_RF95.h>								// https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "LocalTimeRK.h"					        // https://rickkas7.github.io/LocalTimeRK/
#include "Particle.h"

// Particle Product definitions
const uint8_t firmVersion = 1;

// System Mode Calls
SYSTEM_MODE(MANUAL);								// For testing - allows us to monitor the device
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
retained uint8_t resetCount;


SerialLogHandler logHandler(LOG_LEVEL_INFO);		// For monitoring / debugging, you have some options on the next few lines

// Prototype functions
void sensorISR();
void recordCount();
void sendMessage();
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void stayAwakeTimerISR(void);                       // interrupt service routine for sleep control timer
int setLowPowerMode(String command);                // Enable sleep
void flashTheLEDs();                                // Indication that nodes are heard
void userSwitchISR();                               // interrupt service routime for the user switch

// Initialize functions here
LocalTimeSchedule publishSchedule;							// These allow us to enable a schedule and to use local time
LocalTimeConvert conv;


// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Set up pins
const pin_t intPin = A0;
const pin_t blueLED = 		D7;
//Define pins for the RFM9x on my Particle carrier board
const pin_t RFM95_CS =      A5;                     // SPI Chip select pin - Standard SPI pins otherwise
const pin_t RFM95_RST =     D3;                     // Radio module reset
const pin_t RFM95_INT =     D2;                     // Interrupt from radio

//  ***************  LoRA Setup Section ********************
// In this implementation - we have one gateway and two nodes (will generalize later) - nodes are numbered 2 to ...
#define GATEWAY_ADDRESS 1					        // The gateway for each mesh is #1
#define NODE2_ADDRESS 2					            // First Node
#define NODE3_ADDRESS 3					           	// Second Node

// Singleton instance of the radio driver
#define RF95_FREQ 915.0
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE2_ADDRESS);

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#ifndef RH_MAX_MESSAGE_LEN
#define RH_MAX_MESSAGE_LEN 255
#endif

uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
//********************************************************

// Program variables

volatile bool sensorDetect = false;
uint16_t hourly = 0;
uint16_t daily = 0;
const uint16_t devID = 65534;
uint8_t alerts = 0;
uint16_t nextReportSeconds = 10;					// Default send every 1 minute until configured by the gateway
time_t lastReportSeconds = 0;						// Ensures we send right away

void setup() 
{
	// Wait for a USB serial connection for up to 15 seconds
	waitFor(Serial.isConnected, 15000);

	pinMode(blueLED,OUTPUT);						// Blue led signals sends
	pinMode(intPin, INPUT_PULLDOWN);				// Initialize sensor interrupt pin

	// Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  	if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    	resetCount++;
    	if (resetCount > 6) alerts = 13;            // Excessive resets
  	}

	if (!manager.init()) Log.info("init failed"); // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
	
	driver.setFrequency(RF95_FREQ);					// Setup ISM frequency - typically 868.0 or 915.0 in the Americas, or 433.0 in the EU

	driver.setTxPower(23, false);                     // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm (13dBm default):

	attachInterrupt(intPin, sensorISR, RISING);     // Pressure Sensor interrupt from low to high

	Log.info("Startup complete - battery %4.2f%%, reporting every %u seconds clock is %s", System.batteryCharge(), nextReportSeconds, (Time.isValid()) ? "valid" : "not valid");

	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	conv.withCurrentTime().convert();  				        // Convert to local time for use later

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;	// We got through setup without error
}


void loop()
{

	switch (state) {
		case IDLE_STATE: {							// State we spend time in when there is nothing else to do
			if (Time.now() - lastReportSeconds > nextReportSeconds) {	// Using the sendFrequency set above, we go to the reporing state
				sensorDetect = true;
				state = REPORTING_STATE;
			}
		}
		break;

		case REPORTING_STATE: {						// This is where we will send the message then go back to Idle
			sendMessage();
			state = IDLE_STATE;
		}
		break;

	}

	if (sensorDetect) recordCount();                // The ISR had raised the sensor flag - this will service interrupts regardless of state
}

void sensorISR() {
	sensorDetect = true;
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
	Log.info("Sending to manager_mesh_server1");
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
	payload[4] = firmVersion;						// Set for code release
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
	if (Particle.connected()) Particle.publish("sending","payload to server1",PRIVATE);
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
			Log.info("Next report in %u seconds",((buf[7] << 8) | buf[8]));
			nextReportSeconds = ((buf[7] << 8) | buf[8]);
			if (Particle.connected()) Particle.publish("Update",data,PRIVATE);
 		}
		else {
			Log.info("No reply, is rf95_mesh_server1, rf95_mesh_server2 and rf95_mesh_server3 running?");
		}
	}
	else Log.info("sendtoWait failed. Are the intermediate mesh servers running?");
	lastReportSeconds = Time.now();
	digitalWrite(blueLED,LOW);
}

