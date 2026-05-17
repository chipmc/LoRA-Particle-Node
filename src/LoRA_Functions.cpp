#include "LoRA_Functions.h"
#include <RHMesh.h>
#include <RH_RF95.h>						        // https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "config.h"
#include "device_pinout.h"
#include "MyPersistentData.h"
#include "node_time.h"

extern AB1805 ab1805;

#ifndef LORA_VERBOSE_DIAGNOSTICS
#define LORA_VERBOSE_DIAGNOSTICS 0
#endif


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
// const double RF95_FREQ = 915.0;				 	// Frequency - ISM
const double RF95_FREQ = 926.84;				// Center frequency for the omni-directional antenna I am using
const RH_RF95::ModemConfigChoice RF95_MODEM_CONFIG = RH_RF95::Bw125Cr45Sf2048;
const uint16_t LORA_MANAGER_TIMEOUT_MS = 2000;
const int8_t RF95_TX_POWER_DBM = 23;
const bool RF95_USE_RFO = false;
const uint8_t MAX_GATEWAY_ALERT_CODE = 7;
const uint8_t MIN_ASSIGNED_NODE_ADDRESS = 1;
const uint8_t MAX_ASSIGNED_NODE_ADDRESS = 10;

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

namespace {

const char *radioModeName(RHGenericDriver::RHMode mode) {
	switch (mode) {
		case RHGenericDriver::RHModeInitialising: return "Initialising";
		case RHGenericDriver::RHModeSleep: return "Sleep";
		case RHGenericDriver::RHModeIdle: return "Idle";
		case RHGenericDriver::RHModeTx: return "Tx";
		case RHGenericDriver::RHModeRx: return "Rx";
		case RHGenericDriver::RHModeCad: return "Cad";
		default: return "Unknown";
	}
}

const char *modemConfigName(RH_RF95::ModemConfigChoice config) {
	switch (config) {
		case RH_RF95::Bw125Cr45Sf128: return "Bw125Cr45Sf128";
		case RH_RF95::Bw500Cr45Sf128: return "Bw500Cr45Sf128";
		case RH_RF95::Bw31_25Cr48Sf512: return "Bw31_25Cr48Sf512";
		case RH_RF95::Bw125Cr48Sf4096: return "Bw125Cr48Sf4096";
		case RH_RF95::Bw125Cr45Sf2048: return "Bw125Cr45Sf2048";
		default: return "Unknown";
	}
}

void logLoRaSetupDiagnostics() {
	Log.info(
		"LoRa setup diag: RH=%d.%d libStatus=runtime-unavailable gateway=%u freq=%.2f modem=%d/%s timeout=%u txPower=%d useRFO=%s pins(cs=%d irq=%d rst=%d) regVersion=0x%02x",
		RH_VERSION_MAJOR,
		RH_VERSION_MINOR,
		GATEWAY_ADDRESS,
		RF95_FREQ,
		(int) RF95_MODEM_CONFIG,
		modemConfigName(RF95_MODEM_CONFIG),
		LORA_MANAGER_TIMEOUT_MS,
		RF95_TX_POWER_DBM,
		RF95_USE_RFO ? "true" : "false",
		RFM95_CS,
		RFM95_INT,
		RFM95_RST,
		driver.getDeviceVersion());
}

bool isValidGatewayAlertCode(uint8_t alertCode) {
	return alertCode <= MAX_GATEWAY_ALERT_CODE;
}

bool isValidAssignedNodeNumber(uint8_t nodeNumber) {
	return nodeNumber >= MIN_ASSIGNED_NODE_ADDRESS && nodeNumber <= MAX_ASSIGNED_NODE_ADDRESS;
}

void logRejectedGatewayValue(const char *fieldName, uint8_t value, const char *context) {
	Log.warn("Rejecting invalid gateway %s %u in %s", fieldName, value, context);
}

time_t gatewayAckEpoch() {
	return (time_t)((buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5]);
}

bool applyGatewayAckTiming(const char *context) {
	if (!nodeTimeApplyGatewayAck(gatewayAckEpoch(), ab1805, context)) {
		return false;
	}
	sysStatus.set_frequencyMinutes((buf[6] << 8 | buf[7]));
	return true;
}


float computeSuccessPercent(uint8_t attemptCount, uint8_t successCount) {
	const float safeAttemptCount = (attemptCount == 0) ? 1.0f : (float)attemptCount;
	const float rawSuccessPercent = ((float)successCount / safeAttemptCount) * 100.0f;
	float clampedSuccessPercent = rawSuccessPercent;

	if (clampedSuccessPercent < 0.0f) {
		clampedSuccessPercent = 0.0f;
	}
	else if (clampedSuccessPercent > 100.0f) {
		clampedSuccessPercent = 100.0f;
	}

	#if VERBOSE_SYSTEM_LOGS
	if (clampedSuccessPercent != rawSuccessPercent) {
		Log.info("Success rate corrected: raw=%.2f clamped=%.2f", rawSuccessPercent, clampedSuccessPercent);
	}
	#endif

	return clampedSuccessPercent;
}
}


bool LoRA_Functions::setup(bool gatewayID) {
    // Set up the Radio Module
	LoRA_Functions::initializeRadio();
	logLoRaSetupDiagnostics();

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
		sysStatus.set_alertCodeNode(1);															// Join request required
		Log.info("LoRA Radio initialized as an unconfigured node %i and a deviceID of %s and alert code %d", manager.thisAddress(), System.deviceID().c_str(), sysStatus.get_alertCodeNode());
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

	while (true) {
		lenT = sizeof(bufT);
		if (!driver.recv(bufT, &lenT)) {
			break;
		}
	}
}

void LoRA_Functions::sleepLoRaRadio() {
	driver.sleep();                             	// Here is where we will power down the LoRA radio module
}

void LoRA_Functions::logListeningStateEntry() {
	const RHGenericDriver::RHMode mode = driver.mode();
	Log.info(
		"LoRa listen diag: rxArmed=%s mode=%s gateway=%u freq=%.2f modem=%d/%s timeout=%u",
		(mode == RHGenericDriver::RHModeRx) ? "true" : "false",
		radioModeName(mode),
		GATEWAY_ADDRESS,
		RF95_FREQ,
		(int) RF95_MODEM_CONFIG,
		modemConfigName(RF95_MODEM_CONFIG),
		LORA_MANAGER_TIMEOUT_MS);
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
	driver.setTxPower(RF95_TX_POWER_DBM, RF95_USE_RFO);                   // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm (13dBm default).  PA_BOOST?

	driver.setModemConfig(RF95_MODEM_CONFIG);
	// driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);	// This optimized the radio for long range - https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
	driver.setLowDatarate();						// https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a8e2df6a6d2cb192b13bd572a7005da67
	manager.setTimeout(LORA_MANAGER_TIMEOUT_MS);						// 200mSec is the default - may need to extend once we play with other settings on the modem - https://www.airspayce.com/mikem/arduino/RadioHead/classRHReliableDatagram.html
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
		(void) len;
		#if LORA_VERBOSE_DIAGNOSTICS
		Log.info("recvfromAck ok: from=%u dest=%u id=%u flag=%u/%s hops=%u RSSI/SNR=%d/%d", from, dest, id, messageFlag, loraStateNames[(messageFlag <= ALERT_ACK) ? messageFlag : 0], hops, driver.lastRssi(), driver.lastSNR());
		#endif
		if ((buf[0] << 8 | buf[1]) != sysStatus.get_magicNumber()) {
			#if LORA_VERBOSE_DIAGNOSTICS
			Log.info("Magic Number mismatch - ignoring message from=%u dest=%u id=%u RSSI/SNR=%d/%d", from, dest, id, driver.lastRssi(), driver.lastSNR());
			#endif
			return false;
		} 
		lora_state = (LoRA_State)messageFlag;
		#if LORA_VERBOSE_DIAGNOSTICS
		Log.info("Received from node %d with RSSI / SNR of %d / %d - a %s message with %d hops", from, driver.lastRssi(), driver.lastSNR(), loraStateNames[lora_state], hops);
		#endif

		if (lora_state == DATA_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentDataReportNode()) return true;}
		else if (lora_state == JOIN_ACK) { if(LoRA_Functions::instance().receiveAcknowledmentJoinRequestNode()) return true;}
		else {Log.info("Invaled LoRA message flag"); return false;}

	}
	else {
		#if LORA_VERBOSE_DIAGNOSTICS
		Log.info("recvfromAck false: mode=%s rxGood=%u rxBad=%u RSSI/SNR=%d/%d", radioModeName(driver.mode()), driver.rxGood(), driver.rxBad(), driver.lastRssi(), driver.lastSNR());
		#endif
		LoRA_Functions::clearBuffer();
	}
	return false;
}


bool LoRA_Functions::composeDataReportNode() {
	float successPercent = 0.0f;

	if (current.get_messageCount()==0) {		// 8-bit number so need to protect against divide by zero on reset or wrap around
		current.set_messageCount(0);
		current.set_successCount(0);
	}
	current.set_messageCount(current.get_messageCount()+1);

	digitalWrite(BLUE_LED,HIGH);

	int deviceIDCheckSum = stringCheckSum(System.deviceID());

	buf[0] = highByte(sysStatus.get_magicNumber());
	buf[1] = lowByte(sysStatus.get_magicNumber());			
	buf[2] = highByte(deviceIDCheckSum);
	buf[3] = lowByte(deviceIDCheckSum);
	buf[4] = highByte(current.get_hourlyCount());
	buf[5] = lowByte(current.get_hourlyCount()); 
	buf[6] = highByte(current.get_dailyCount());
	buf[7] = lowByte(current.get_dailyCount()); 
	buf[8] = sysStatus.get_sensorType();
	buf[9] = current.get_internalTempC();
	buf[10] = current.get_stateOfCharge();
	buf[11] = current.get_batteryState();	
	buf[12] = sysStatus.get_resetCount();
	buf[13] = current.get_messageCount();
	buf[14] = current.get_successCount();
	buf[15] = highByte(current.get_RSSI());
	buf[16] = lowByte(current.get_RSSI());
	buf[17] = highByte(current.get_SNR());
	buf[18] = lowByte(current.get_SNR());

	// Send a message to manager_server
  	// A route to the destination will be automatically discovered.
	unsigned char result = manager.sendtoWait(buf, 19, GATEWAY_ADDRESS, DATA_RPT);
	
	if ( result == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		current.set_successCount(current.get_successCount()+1);
		successPercent = computeSuccessPercent(current.get_messageCount(), current.get_successCount());
		current.set_RSSI(driver.lastRssi());				// Set these here - will send on next data report
		current.set_SNR(driver.lastSNR());
		Log.info("Node %d data report delivered - success rate %4.2f and  RSSI/SNR of %d / %d ",sysStatus.get_nodeNumber(),successPercent,current.get_RSSI(), current.get_SNR());
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else if (result == RH_ROUTER_ERROR_NO_ROUTE) {
		successPercent = computeSuccessPercent(current.get_messageCount(), current.get_successCount());
        Log.info("Node %d - Data report send to gateway %d failed - No Route - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, successPercent);
    }
    else if (result == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
		successPercent = computeSuccessPercent(current.get_messageCount(), current.get_successCount());
        Log.info("Node %d - Data report send to gateway %d failed - Unable to Deliver - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,successPercent);
	}
	else  {
		successPercent = computeSuccessPercent(current.get_messageCount(), current.get_successCount());
		Log.info("Node %d - Data report send to gateway %d failed  - Unknown - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,successPercent);
	}
	digitalWrite(BLUE_LED, LOW);
	return false;
}

bool LoRA_Functions::receiveAcknowledmentDataReportNode() {
	LEDStatus blinkBlue(RGB_COLOR_BLUE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

	if (!isValidGatewayAlertCode(buf[8])) {
		logRejectedGatewayValue("alert code", buf[8], "DATA_ACK");
		return false;
	}

	if (!applyGatewayAckTiming("DATA_ACK")) {
		return false;
	}

	// contents of response for 1-7 handled in common function above
	sysStatus.set_alertCodeNode(buf[8]);

	if (sysStatus.get_alertCodeNode() == 7) {		// This alert triggers an update to the sensor type on the node - handle it here
		Log.info("The gatway is updating sensor type from %d to %d", sysStatus.get_sensorType(), buf[9]);
		sysStatus.set_sensorType(buf[9]);
		sysStatus.set_alertCodeNode(0);				// Sensor updated - clear alert
	}
	else if (sysStatus.get_alertCodeNode()) {
		Log.info("The gateway set an alert %d", sysStatus.get_alertCodeNode());
		sysStatus.set_alertTimestampNode(Time.now());	
	}

	sysStatus.set_openHours(buf[10]);				// The Gateway tells us whether the park is open or closed

	if (sysStatus.get_openHours() == 0) {			// Open Hours Processing
		current.resetEverything();
		Log.info("Park is closed - reset everything");
	}
	else sysStatus.set_openHours(true);

	Log.info("Data report acknowledged %s alert for message %d park is %s and alert code is %d", (sysStatus.get_alertCodeNode()) ? "with":"without", buf[11], (buf[10] ==1) ? "open":"closed", sysStatus.get_alertCodeNode());
	
	blinkBlue.setActive(true);
	unsigned long strength = (unsigned long)(map(current.get_RSSI(),-10,-140,3000,100));
	strength = constrain(strength,100UL,3000UL);
    delay(strength);
    blinkBlue.setActive(false);

	return true;
}

bool LoRA_Functions::composeJoinRequesttNode() {
	char deviceID[25];
	System.deviceID().toCharArray(deviceID, 25);					// the deviceID is 24 charcters long
	int deviceIDCheckSum = stringCheckSum(System.deviceID());

	manager.setThisAddress(sysStatus.get_nodeNumber());				// Join with the right node number

	buf[0] = highByte(sysStatus.get_magicNumber());					// Needs to equal 128
	buf[1] = lowByte(sysStatus.get_magicNumber());					// Needs to equal 128
	buf[2] = highByte(deviceIDCheckSum);
	buf[3] = lowByte(deviceIDCheckSum);
	for (uint8_t i=0; i < sizeof(deviceID); i++) {
		buf[i+4] = deviceID[i];
	}
	buf[29] = sysStatus.get_sensorType();

	digitalWrite(BLUE_LED,HIGH);
	unsigned char result = manager.sendtoWait(buf, 30, GATEWAY_ADDRESS, JOIN_REQ);
	digitalWrite(BLUE_LED, LOW);

	if (result == RH_ROUTER_ERROR_NONE) {					// It has been reliably delivered to the next node.
		current.set_RSSI(driver.lastRssi());				// Set these here - will send on next data report
		current.set_SNR(driver.lastSNR());
		Log.info("Join request sent to gateway successfully RSSI/SNR of %d / %d ",current.get_RSSI(), current.get_SNR());
		return true;
	}
	else {
		Log.info("Join request to Gateway failed");
		return false;
	}
}

bool LoRA_Functions::receiveAcknowledmentJoinRequestNode() {
	LEDStatus blinkOrange(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

	if (!isValidGatewayAlertCode(buf[8])) {
		logRejectedGatewayValue("alert code", buf[8], "JOIN_ACK");
		return false;
	}
	if (sysStatus.get_nodeNumber() > 10 && !isValidAssignedNodeNumber(buf[9])) {
		logRejectedGatewayValue("node number", buf[9], "JOIN_ACK");
		return false;
	}

	if (!applyGatewayAckTiming("JOIN_ACK")) {
		return false;
	}
	sysStatus.set_alertCodeNode(buf[8]);
	if (sysStatus.get_alertCodeNode() != 0) {
		sysStatus.set_alertTimestampNode(Time.now());
	}

	if (sysStatus.get_nodeNumber() > 10) sysStatus.set_nodeNumber(buf[9]);
	sysStatus.set_sensorType(buf[10]);
	Log.info("Node %d Join request acknowledged and sensor set to %d", sysStatus.get_nodeNumber(), sysStatus.get_sensorType());
	manager.setThisAddress(sysStatus.get_nodeNumber());

    blinkOrange.setActive(true);
	unsigned long strength = (unsigned long)(map(current.get_RSSI(),-10,-140,3000,100));
	strength = constrain(strength,100UL,3000UL);
    delay(strength);
    blinkOrange.setActive(false);

	return true;
}


int LoRA_Functions::stringCheckSum(String str){												// This function is made for the Particle DeviceID
    int result = 0;
    for(unsigned int i = 0; i < str.length(); i++){
      int asciiCode = (int)str[i];

      if (asciiCode >=48 && asciiCode <58) {              // 0-9
        result += asciiCode - 48;
      } 
      else if (asciiCode >=65 && asciiCode < 71) {        // A-F
        result += 10 + asciiCode -65;
      }
      else if (asciiCode >=97 && asciiCode < 103) {       // a - f
        result += 10 + asciiCode -97;
      }
    }
    return result;
}

