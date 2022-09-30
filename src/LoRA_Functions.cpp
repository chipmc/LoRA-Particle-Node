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
// In this implementation - we have one gateway and two nodes (will generalize later) - nodes are numbered 2 to ...
// 2- 9 for new nodes making a join request
// 10 - 255 nodes assigned to the mesh
const uint8_t GATEWAY_ADDRESS = 0;			 // Gateway addess is always zero
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

	if (!(sysStatus.get_structuresVersion() == 128)) {    	// This will be our indication that the deviceID and nodeID has not yet been set
		randomSeed(sysStatus.get_lastConnection());			// 32-bit number for seed
		sysStatus.set_deviceID(random(1,65535));			// 16-bit number for deviceID
		if (!gatewayID) sysStatus.set_nodeNumber(random(10,255));		// Random number in - unconfigured - range will trigger a Join request
		else sysStatus.set_nodeNumber(0);
		sysStatus.set_structuresVersion(128);			// Set the structure to the magic number so we can have a stable deviceID
	}

	manager.setThisAddress(sysStatus.get_nodeNumber());	// Assign the NodeNumber to this node
	
	if (manager.thisAddress() > 0) Log.info("LoRA Radio initialized as node %i and with a DeviceID of %i", manager.thisAddress(), sysStatus.get_deviceID());
	else Log.info("LoRA Radio initialized as a gateway with a deviceID of %i", sysStatus.get_deviceID());
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
	driver.sleep();                             // Here is where we will power down the LoRA radio module
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
		lora_state = (LoRA_State)messageFlag;
		Log.info("Received from node %d with rssi=%d - a %s message", from, driver.lastRssi(), loraStateNames[lora_state]);

		Time.setTime(((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]));  // Set time based on response from gateway
		sysStatus.set_frequencyMinutes((buf[5] << 8 | buf[6]));			// Frequency of reporting set by Gateway
		Log.info("Set clock to %s and report frequency to %d minutes", Time.timeStr().c_str(),sysStatus.get_frequencyMinutes());

		if (lora_state == DATA_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentDataReportNode()) return true;}
		if (lora_state == JOIN_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentJoinRequestNode()) return true;}
		if (lora_state == ALERT_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentAlertReportNode()) return true;}
	}
	return false;
}


bool LoRA_Functions::composeDataReportNode() {
	static int attempts = 0;
	static int success = 0;
	static uint8_t msgCnt = 0;


	digitalWrite(BLUE_LED,HIGH);
	attempts++;
	msgCnt++;
	Log.info("Sending data report number %d",msgCnt);

	buf[0] = highByte(sysStatus.get_deviceID());					// Set for device
	buf[1] = lowByte(sysStatus.get_deviceID());
	buf[2] = highByte(sysStatus.get_nodeNumber());				// NodeID for verification
	buf[3] = lowByte(sysStatus.get_nodeNumber());				
	buf[4] = 1;						// Set for code release - fix later
	buf[5] = highByte(current.get_hourlyCount());
	buf[6] = lowByte(current.get_hourlyCount()); 
	buf[7] = highByte(current.get_dailyCount());
	buf[8] = lowByte(current.get_dailyCount()); 
	buf[9] = current.get_internalTempC();
	buf[10] = current.get_stateOfCharge();
	buf[11] = current.get_batteryState();	
	buf[12] = sysStatus.get_resetCount();
	buf[13] = 1;				// reserved for later
	buf[14] = highByte(driver.lastRssi());
	buf[15] = lowByte(driver.lastRssi()); 
	buf[16] = msgCnt;

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	unsigned char result = manager.sendtoWait(buf, 17, GATEWAY_ADDRESS, DATA_RPT);
	
	if ( result == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		success++;
		Log.info("Data report delivered - success rate %4.2f",((success * 1.0)/ attempts)*100.0);
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else if (result == RH_ROUTER_ERROR_NO_ROUTE) {
        Log.info("Node %d - Data report send to gateway %d failed - No Route - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, ((success * 1.0)/ attempts)*100.0);
    }
    else if (result == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
        Log.info("Node %d - Data report send to gateway %d failed - Unable to Deliver - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, ((success * 1.0)/ attempts)*100.0);
	}
	else  {
		Log.info("Node %d - Data report send to gateway %d failed  - Unknown - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, ((success * 1.0)/ attempts)*100.0);
	}
	digitalWrite(BLUE_LED, LOW);
	return false;
}

bool LoRA_Functions::receiveAcknowledmentDataReportNode() {
		
	Log.info("Data report acknowledged");
	return true;
}

bool LoRA_Functions::composeJoinRequesttNode() {
	digitalWrite(BLUE_LED,HIGH);

	buf[0] = highByte(sysStatus.get_deviceID());                      // deviceID is unique to the device
	buf[1] = lowByte(sysStatus.get_deviceID());
	buf[2] = highByte(sysStatus.get_nodeNumber());                  			// Node Number
	buf[3] = lowByte(sysStatus.get_nodeNumber());
	buf[4] = sysStatus.get_structuresVersion();						// Needs to equal 128
	buf[5] = highByte(driver.lastRssi());				        // Signal strength
	buf[6] = lowByte(driver.lastRssi()); 

	
	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	Log.info("Sending join request because %s",(sysStatus.get_nodeNumber() < 10) ? "a NodeNumber is needed" : "the clock is not set");
	if (manager.sendtoWait(buf, 7, GATEWAY_ADDRESS, JOIN_REQ) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		Log.info("Data report send to gateway successfully");
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else {
		Log.info("Data report send to Gateway failed");
		digitalWrite(BLUE_LED, LOW);
		return false;
	}
}

bool LoRA_Functions::receiveAcknowledmentJoinRequestNode() {

	if (sysStatus.get_nodeNumber() < 10 && buf[0] == 128) sysStatus.set_nodeNumber((buf[7] << 8 | buf[8]));
	Log.info("Join request acknowledged and node ID set to %d", sysStatus.get_nodeNumber());
	return true;
}

bool LoRA_Functions::composeAlertReportNode() {
	digitalWrite(BLUE_LED,HIGH);

	buf[0] = highByte(sysStatus.get_deviceID());       // deviceID is unique to the device
	buf[1] = lowByte(sysStatus.get_deviceID());
	buf[2] = highByte(sysStatus.get_nodeNumber());     // Node Number
	buf[3] = lowByte(sysStatus.get_nodeNumber());
	buf[4] = highByte(current.get_alertCodeNode());   // Node's Alert Code
	buf[5] = ((uint8_t) ((Time.now()) >> 24));  // Fourth byte - current time
	buf[6] = ((uint8_t) ((Time.now()) >> 16));	// Third byte
	buf[7] = ((uint8_t) ((Time.now()) >> 8));	// Second byte
	buf[8] = ((uint8_t) (Time.now()));		    // First byte			
	buf[9] = highByte(driver.lastRssi());		// Signal strength
	buf[10] = lowByte(driver.lastRssi()); 

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	if (manager.sendtoWait(buf, 11, GATEWAY_ADDRESS, ALERT_RPT) == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		Log.info("Success sending Alert Report number %d to gateway at %d", current.get_alertCodeNode(), GATEWAY_ADDRESS);
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

	Log.info("Alert report acknowledged");
	return true;
}


