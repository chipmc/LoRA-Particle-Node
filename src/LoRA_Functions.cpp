#include "LoRA_Functions.h"
#include <RHMesh.h>
#include <RH_RF95.h>						        // https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "device_pinout.h"
#include "MyPersistentData.h"


// Singleton instantiation - from template
LoRA_Functions *LoRA_Functions::_instance;

// [static]
LoRA_Functions &LoRA_Functions::instance() {
    if (!_instance) {
        _instance = new LoRA_Functions();
    }
    return *_instance;
}

LoRA_Functions::LoRA_Functions() {
}

LoRA_Functions::~LoRA_Functions() {
}


// ************************************************************************
// *****                      LoRA Setup                              *****
// ************************************************************************
// In this implementation - we have one gateway numde number 0 and up to 10 nodes with node numbers 1-10
// Node numbers greater than 10 initiate a join request
const uint8_t GATEWAY_ADDRESS = 0;
const double RF95_FREQ = 915.0;				 // Frequency - ISM

// Define the message flags
typedef enum { NULL_STATE, JOIN_REQ, JOIN_ACK, DATA_RPT, DATA_ACK, ALERT_RPT, ALERT_ACK} LoRA_State;
char loraStateNames[7][16] = {"Null", "Join Req", "Join Ack", "Data Report", "Data Ack", "Alert Rpt", "Alert Ack"};
static LoRA_State lora_state = NULL_STATE;

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, GATEWAY_ADDRESS);

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#ifndef RH_MAX_MESSAGE_LEN
#define RH_MAX_MESSAGE_LEN 255
#endif

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
// #define RH_MESH_MAX_MESSAGE_LEN 50
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];               // Related to max message size - RadioHead example note: dont put this on the stack:


bool LoRA_Functions::setup(bool gatewayID) {
    // Set up the Radio Module
	if (!manager.init()) {
		Log.info("init failed");					// Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
		return false;
	}

	driver.setFrequency(RF95_FREQ);					// Frequency is typically 868.0 or 915.0 in the Americas, or 433.0 in the EU - Are there more settings possible here?
	driver.setTxPower(23, false);                   // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm (13dBm default).  PA_BOOST?
	
	Log.info("in LoRA setup - node number %d",sysStatus.get_nodeNumber());

	if (gatewayID == true) {
		sysStatus.set_nodeNumber(GATEWAY_ADDRESS);							// Gateway - Manager is initialized by default with GATEWAY_ADDRESS - make sure it is stored in FRAM
		Log.info("LoRA Radio initialized as a gateway with a deviceID of %s", System.deviceID().c_str());
	}
	else if (sysStatus.get_nodeNumber() > 0 && sysStatus.get_nodeNumber() <= 10) {
		manager.setThisAddress(sysStatus.get_nodeNumber());// Node - use the Node address in valid range from memory
		Log.info("LoRA Radio initialized as node %i and a deviceID of %s", manager.thisAddress(), System.deviceID().c_str());
	}
	else {																						// Else, we will set as an unconfigured node
		sysStatus.set_nodeNumber(11);
		manager.setThisAddress(11);
		Log.info("LoRA Radio initialized as an unconfigured node %i and a deviceID of %s", manager.thisAddress(), System.deviceID().c_str());
	}

	return true;
}

void LoRA_Functions::loop() {
    // Put your code to run during the application thread loop here
}


// ************************************************************************
// *****					Common LoRA Functions					*******
// ************************************************************************


void LoRA_Functions::clearBuffer() {
	uint8_t bufT[RH_RF95_MAX_MESSAGE_LEN];
	uint8_t lenT;

	while(driver.recv(bufT, &lenT)) {};
}

void LoRA_Functions::sleepLoRaRadio() {
	driver.sleep();                             	// Here is where we will power down the LoRA radio module
}

bool  LoRA_Functions::initializeRadio() {  			// Set up the Radio Module
	digitalWrite(RFM95_RST,LOW);					// Reset the radio module before setup
	delay(10);
	digitalWrite(RFM95_RST,HIGH);
	delay(10);

	if (!manager.init()) {
		Log.info("init failed");					// Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
		return false;
	}
	driver.setFrequency(RF95_FREQ);					// Frequency is typically 868.0 or 915.0 in the Americas, or 433.0 in the EU - Are there more settings possible here?
	driver.setTxPower(23, false);                   // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm (13dBm default).  PA_BOOST?

return true;
}


// ************************************************************************
// *****                         Node Functions                       *****
// ************************************************************************
bool LoRA_Functions::listenForLoRAMessageNode() {
	uint8_t len = sizeof(buf);
	uint8_t from;  
	uint8_t dest;
	uint8_t id;
	uint8_t messageFlag;
	uint8_t hops;
	if (manager.recvfromAck(buf, &len, &from, &dest, &id, &messageFlag, &hops))	{	// We have received a message
		buf[len] = 0;
		if ((buf[0] << 8 | buf[1]) != sysStatus.get_magicNumber()) {
			Log.info("Magic Number mismatch - ignoring message");
			return false;
		} 
		lora_state = (LoRA_State)messageFlag;
		Log.info("Received from node %d with rssi=%d - a %s message", from, driver.lastRssi(), loraStateNames[lora_state]);

		Time.setTime(((buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5]));  // Set time based on response from gateway
		sysStatus.set_frequencyMinutes((buf[6] << 8 | buf[7]));			// Frequency of reporting set by Gateway

		// The gateway may set an alert code for the node
		sysStatus.set_alertCodeNode(buf[8]);
		sysStatus.set_alertTimestampNode(Time.now());

		Log.info("Set clock to %s and report frequency to %d minutes", Time.timeStr().c_str(),sysStatus.get_frequencyMinutes());

		if (lora_state == DATA_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentDataReportNode()) return true;}
		if (lora_state == JOIN_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentJoinRequestNode()) return true;}
		if (lora_state == ALERT_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentAlertReportNode()) return true;}
	}
	return false;
}


bool LoRA_Functions::composeDataReportNode() {

	float percentSuccess = ((current.get_successCount() * 1.0)/ current.get_messageCount())*100.0;

	digitalWrite(BLUE_LED,HIGH);
	if (current.get_messageCount() == 255) {		// This should not happen in a day unless we pick a very small reporting freq
		current.set_messageCount(0);				// Prevent divide by zero
		current.set_successCount(0);				// Zero as well
	}
	else current.set_messageCount(current.get_messageCount()+1);
	Log.info("Sending data report number %d",current.get_messageCount());

	buf[0] = highByte(sysStatus.get_magicNumber());
	buf[1] = lowByte(sysStatus.get_magicNumber());			
	buf[2] = 1;						// Set for code release - fix later
	buf[3] = highByte(current.get_hourlyCount());
	buf[4] = lowByte(current.get_hourlyCount()); 
	buf[5] = highByte(current.get_dailyCount());
	buf[6] = lowByte(current.get_dailyCount()); 
	buf[7] = sysStatus.get_sensorType();
	buf[8] = current.get_internalTempC();
	buf[9] = current.get_stateOfCharge();
	buf[10] = current.get_batteryState();	
	buf[11] = sysStatus.get_resetCount();
	buf[12] = current.get_messageCount();
	buf[13] = current.get_successCount();

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	unsigned char result = manager.sendtoWait(buf, 14, GATEWAY_ADDRESS, DATA_RPT);
	
	if ( result == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		current.set_successCount(current.get_successCount()+1);
		percentSuccess = ((current.get_successCount() * 1.0)/ current.get_messageCount())*100.0;
		Log.info("Data report delivered - success rate %4.2f",percentSuccess);
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else if (result == RH_ROUTER_ERROR_NO_ROUTE) {
        Log.info("Node %d - Data report send to gateway %d failed - No Route - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, percentSuccess);
    }
    else if (result == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
        Log.info("Node %d - Data report send to gateway %d failed - Unable to Deliver - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,percentSuccess);
	}
	else  {
		Log.info("Node %d - Data report send to gateway %d failed  - Unknown - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,percentSuccess);
	}
	digitalWrite(BLUE_LED, LOW);
	return false;
}

bool LoRA_Functions::receiveAcknowledmentDataReportNode() {
	LEDStatus blinkBlue(RGB_COLOR_BLUE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

	// contents of response for 1-7 handled in common function above
	byte alertSetByGateway = buf[8];
	if (alertSetByGateway == 1) {								// Gateway did not recognize our node number, need to re-join
		sysStatus.set_nodeNumber(11);
		manager.setThisAddress(11);
		sysStatus.set_alertCodeNode(alertSetByGateway);	
		sysStatus.set_alertTimestampNode(Time.now());	
		Log.info("LoRA Radio initialized as an unconfigured node %i and a deviceID of %s", manager.thisAddress(), System.deviceID().c_str());
	}
	else if (buf[8] > 0) {							// the Gateway set an alert
		Log.info("The gateway set an alert %d", alertSetByGateway);
		sysStatus.set_alertCodeNode(alertSetByGateway);	
		sysStatus.set_alertTimestampNode(Time.now());			
	}
	sysStatus.set_sensorType(buf[9]); 				// In data response gateway overwrites node

	if (buf[10] == 0) {								// Open Hours Processing
		sysStatus.set_openHours(false);				// Open hours or not - impacts whether we power down the sensor for sleep
		Log.info("Park is closed - reset everything");
	}
	else sysStatus.set_openHours(true);

	Log.info("Data report acknowledged %s alert for message %d park is %s and alert code is %d", (alertSetByGateway > 0) ? "with":"without", buf[11], (buf[10] ==1) ? "open":"closed", sysStatus.get_alertCodeNode());
	
	blinkBlue.setActive(true);
	unsigned long strength = (unsigned long)(map(driver.lastRssi(),-30,-120,2000,0));
    delay(strength);
    blinkBlue.setActive(false);

	return true;
}

bool LoRA_Functions::composeJoinRequesttNode() {

	digitalWrite(BLUE_LED,HIGH);

	char deviceID[25];
	System.deviceID().toCharArray(deviceID, 25);					// the deviceID is 24 charcters long

	buf[0] = highByte(sysStatus.get_magicNumber());					// Needs to equal 128
	buf[1] = lowByte(sysStatus.get_magicNumber());					// Needs to equal 128
	for (uint8_t i=0; i < sizeof(deviceID); i++) {
		buf[i+2] = deviceID[i];
	}
	buf[27] = sysStatus.get_sensorType();

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	Log.info("Sending join request because %s",(sysStatus.get_nodeNumber() > 10) ? "a NodeNumber is needed" : "the clock is not set");
	if (manager.sendtoWait(buf, 28, GATEWAY_ADDRESS, JOIN_REQ) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		Log.info("Join request sent to gateway successfully");
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else {
		Log.info("Join request to Gateway failed");
		digitalWrite(BLUE_LED, LOW);
		return false;
	}
}

bool LoRA_Functions::receiveAcknowledmentJoinRequestNode() {
	LEDStatus blinkOrange(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

	if (sysStatus.get_nodeNumber() > 10) sysStatus.set_nodeNumber(buf[9]);
	sysStatus.set_sensorType(buf[10]);
	Log.info("Node %d Join request acknowledged and sensor set to %d", sysStatus.get_nodeNumber(), sysStatus.get_sensorType());
	manager.setThisAddress(sysStatus.get_nodeNumber());

    blinkOrange.setActive(true);
	unsigned long strength = (unsigned long)(map(driver.lastRssi(),-30,-120,2000,0));
    delay(strength);
    blinkOrange.setActive(false);

	return true;
}

bool LoRA_Functions::composeAlertReportNode() {
	digitalWrite(BLUE_LED,HIGH);

	buf[0] = highByte(sysStatus.get_magicNumber());					// Magic Number
	buf[1] = lowByte(sysStatus.get_magicNumber());					
	buf[2] = sysStatus.get_alertCodeNode();   						// Node's Alert Code


	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	if (manager.sendtoWait(buf, 3, GATEWAY_ADDRESS, ALERT_RPT) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		Log.info("Success sending Alert Report number %d to gateway at %d", sysStatus.get_alertCodeNode(), GATEWAY_ADDRESS);
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else {
		Log.info("Node - Alert Report send to Gateway failed");
		digitalWrite(BLUE_LED, LOW);
		return false;
	}
}

bool LoRA_Functions::receiveAcknowledmentAlertReportNode() {

	Log.info("Alert report acknowledged with alert code %d", sysStatus.get_alertCodeNode());
	return true;
}


