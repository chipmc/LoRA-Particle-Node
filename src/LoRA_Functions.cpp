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
// const double RF95_FREQ = 915.0;				 	// Frequency - ISM
const double RF95_FREQ = 926.84;				// Center frequency for the omni-directional antenna I am using

// Define the message flags
typedef enum { NULL_STATE, JOIN_REQ, JOIN_ACK, DATA_RPT, DATA_ACK, ALERT_RPT, ALERT_ACK} LoRA_State;
char loraStateNames[7][16] = {"Null", "Join Req", "Join Ack", "Data Report", "Data Ack", "Alert Rpt", "Alert Ack"};
static LoRA_State lora_state = NULL_STATE;

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, GATEWAY_ADDRESS);

namespace {
const RH_RF95::ModemConfigChoice RF95_MODEM_CONFIG = RH_RF95::Bw125Cr48Sf4096;
const uint16_t RH_MESH_TIMEOUT_MS = 2000;
const uint8_t RH_MESH_RETRIES = RH_DEFAULT_RETRIES;
const bool RH_LOW_DATARATE_ENABLED = true;
const int8_t RH_TX_POWER_REQUEST_DBM = 20;
const uint16_t RH_PREAMBLE_BYTES = 8;
const bool RH_PAYLOAD_CRC_ENABLED = true;
const char RAW_TEST_PAYLOAD[] = "RAW_NODE_1_TEST";
const char *const RH_SPI_INSTANCE_NAME = "SPI";
const char *const RH_SYNC_WORD_STATE = "not-used-lora-default";

const char *radioModeName(RHGenericDriver::RHMode mode) {
	switch(mode) {
	case RHGenericDriver::RHModeInitialising:
		return "init";
	case RHGenericDriver::RHModeSleep:
		return "sleep";
	case RHGenericDriver::RHModeIdle:
		return "idle";
	case RHGenericDriver::RHModeTx:
		return "tx";
	case RHGenericDriver::RHModeRx:
		return "rx";
	case RHGenericDriver::RHModeCad:
		return "cad";
	default:
		return "unknown";
	}
}

const char *loraPacketTypeName(uint8_t packetType) {
	switch(packetType) {
	case JOIN_REQ:
		return "JOIN_REQ";
	case JOIN_ACK:
		return "JOIN_ACK";
	case DATA_RPT:
		return "DATA_RPT";
	case DATA_ACK:
		return "DATA_ACK";
	case ALERT_RPT:
		return "ALERT_RPT";
	case ALERT_ACK:
		return "ALERT_ACK";
	default:
		return "UNKNOWN";
	}
}

const char *meshResultName(uint8_t result) {
	switch(result) {
	case RH_ROUTER_ERROR_NONE:
		return "success";
	case RH_ROUTER_ERROR_INVALID_LENGTH:
		return "invalid length";
	case RH_ROUTER_ERROR_NO_ROUTE:
		return "no route";
	case RH_ROUTER_ERROR_TIMEOUT:
		return "timeout";
	case RH_ROUTER_ERROR_NO_REPLY:
		return "no reply";
	case RH_ROUTER_ERROR_UNABLE_TO_DELIVER:
		return "unable to deliver";
	default:
		return "other";
	}
}

const char *modemConfigName(RH_RF95::ModemConfigChoice config) {
	switch(config) {
	case RH_RF95::Bw125Cr45Sf128:
		return "Bw125Cr45Sf128";
	case RH_RF95::Bw500Cr45Sf128:
		return "Bw500Cr45Sf128";
	case RH_RF95::Bw31_25Cr48Sf512:
		return "Bw31_25Cr48Sf512";
	case RH_RF95::Bw125Cr48Sf4096:
		return "Bw125Cr48Sf4096";
	case RH_RF95::Bw125Cr45Sf2048:
		return "Bw125Cr45Sf2048";
	default:
		return "unknown";
	}
}

void modemConfigDetails(RH_RF95::ModemConfigChoice config, const char *&bandwidth, uint8_t &spreadingFactor, const char *&codingRate) {
	switch(config) {
	case RH_RF95::Bw125Cr45Sf128:
		bandwidth = "125kHz";
		spreadingFactor = 7;
		codingRate = "4/5";
		break;
	case RH_RF95::Bw500Cr45Sf128:
		bandwidth = "500kHz";
		spreadingFactor = 7;
		codingRate = "4/5";
		break;
	case RH_RF95::Bw31_25Cr48Sf512:
		bandwidth = "31.25kHz";
		spreadingFactor = 9;
		codingRate = "4/8";
		break;
	case RH_RF95::Bw125Cr48Sf4096:
		bandwidth = "125kHz";
		spreadingFactor = 12;
		codingRate = "4/8";
		break;
	case RH_RF95::Bw125Cr45Sf2048:
		bandwidth = "125kHz";
		spreadingFactor = 11;
		codingRate = "4/5";
		break;
	default:
		bandwidth = "unknown";
		spreadingFactor = 0;
		codingRate = "unknown";
		break;
	}
}

void ensureRadioReadyForTransmit() {
	RHGenericDriver::RHMode modeBefore = driver.mode();
	driver.setModeIdle();
	manager.setThisAddress(sysStatus.get_nodeNumber());
	Log.info("LoRa TX prep: mode=%s node=%u dest=%u", radioModeName(modeBefore), manager.thisAddress(), GATEWAY_ADDRESS);
}

int8_t appliedTxPowerDbm() {
	if (RH_TX_POWER_REQUEST_DBM < 2) {
		return 2;
	}
	if (RH_TX_POWER_REQUEST_DBM > 20) {
		return 20;
	}
	return RH_TX_POWER_REQUEST_DBM;
}

void logRadioInitConfig() {
	Log.info("LoRa init rf: freq=%.2f modem=%d/%s timeout=%u",
		RF95_FREQ,
		(int)RF95_MODEM_CONFIG,
		modemConfigName(RF95_MODEM_CONFIG),
		RH_MESH_TIMEOUT_MS,
		RH_MESH_TIMEOUT_MS);
	Log.info("LoRa init net: node=%u dest=%u txPower=%ddBm",
		manager.thisAddress(),
		GATEWAY_ADDRESS,
		appliedTxPowerDbm());
	#if FIELD_DEBUG_BUILD
	Log.info("LoRa pin map: cs=%u rst=%u irq=%u spi=%s",
		(unsigned)RFM95_CS,
		(unsigned)RFM95_RST,
		(unsigned)RFM95_INT,
		RH_SPI_INSTANCE_NAME);
	#endif
}

void logNodeSendStart(const char *packetType, uint8_t payloadLen, uint8_t attemptNumber) {
	RHRouter::RoutingTableEntry* route = manager.getRouteTo(GATEWAY_ADDRESS);
	const char *bandwidth;
	const char *codingRate;
	uint8_t spreadingFactor;
	modemConfigDetails(RF95_MODEM_CONFIG, bandwidth, spreadingFactor, codingRate);
	Log.info("LoRa TX start: type=%s dest=%u len=%u node=%u attempt=%u",
		packetType,
		GATEWAY_ADDRESS,
		payloadLen,
		manager.thisAddress(),
		attemptNumber);
	Log.info("LoRa TX rf: freq=%.2f modem=%d/%s bw=%s sf=%u cr=%s txPower=%ddBm",
		RF95_FREQ,
		(int)RF95_MODEM_CONFIG,
		modemConfigName(RF95_MODEM_CONFIG),
		bandwidth,
		spreadingFactor,
		codingRate,
		RH_TX_POWER_REQUEST_DBM);
	Log.info("LoRa TX route: route=%s nextHop=%u cs=%u int=%u rst=%u",
		route ? "known" : "missing",
		route ? route->next_hop : RH_BROADCAST_ADDRESS,
		(unsigned)RFM95_CS,
		(unsigned)RFM95_INT,
		(unsigned)RFM95_RST);
}

void logNodeSendResult(const char *packetType, uint8_t result) {
	Log.info("LoRa TX result: type=%s dest=%u rc=%u (%s)",
		packetType,
		GATEWAY_ADDRESS,
		result,
		meshResultName(result));
}
}

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
	if (!LoRA_Functions::initializeRadio()) {
		return false;
	}

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

	logRadioInitConfig();

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
	uint8_t lenT = sizeof(bufT);

	while(driver.recv(bufT, &lenT)) {
		lenT = sizeof(bufT);
	}
}

void LoRA_Functions::sleepLoRaRadio() {
	driver.sleep();                             	// Here is where we will power down the LoRA radio module
}

bool  LoRA_Functions::initializeRadio() {  			// Set up the Radio Module
	digitalWrite(RFM95_RST,LOW);					// Reset the radio module before setup
	delay(10);
	digitalWrite(RFM95_RST,HIGH);
	delay(10);

	bool managerInitResult = manager.init();
	#if FIELD_DEBUG_BUILD
	Log.info("LoRa radio init: manager.init()=%s regVersion=0x%02x", managerInitResult ? "true" : "false", driver.getDeviceVersion());
	#endif
	if (!managerInitResult) {
		Log.info("init failed");					// Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
		return false;
	}
	bool setFrequencyResult = driver.setFrequency(RF95_FREQ);			// Frequency is typically 868.0 or 915.0 in the Americas, or 433.0 in the EU - Are there more settings possible here?
	driver.setTxPower(RH_TX_POWER_REQUEST_DBM, false); // PA_BOOST path on the RFM95.

	bool setModemConfigResult = driver.setModemConfig(RF95_MODEM_CONFIG);
	// driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);	// This optimized the radio for long range - https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
	driver.setPreambleLength(RH_PREAMBLE_BYTES);
	driver.setPayloadCRC(RH_PAYLOAD_CRC_ENABLED);
	driver.setLowDatarate();						// https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a8e2df6a6d2cb192b13bd572a7005da67
	manager.setTimeout(RH_MESH_TIMEOUT_MS);			// 200mSec is the default - may need to extend once we play with other settings on the modem - https://www.airspayce.com/mikem/arduino/RadioHead/classRHReliableDatagram.html
	manager.setRetries(RH_MESH_RETRIES);
	#if FIELD_DEBUG_BUILD
	Log.info("LoRa radio cfg: init=%s ver=0x%02x freq=%s %.2f modem=%s %d/%s",
		managerInitResult ? "ok" : "fail",
		driver.getDeviceVersion(),
		setFrequencyResult ? "ok" : "fail",
		RF95_FREQ,
		setModemConfigResult ? "ok" : "fail",
		(int)RF95_MODEM_CONFIG,
		modemConfigName(RF95_MODEM_CONFIG));
	Log.info("LoRa radio tx: req=%ddBm applied=%ddBm preamble=%u crc=%s sync=%s",
		RH_TX_POWER_REQUEST_DBM,
		appliedTxPowerDbm(),
		RH_PREAMBLE_BYTES,
		RH_PAYLOAD_CRC_ENABLED ? "on" : "off",
		RH_SYNC_WORD_STATE);
	#endif
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
		Log.info("LoRa RX node: type=%s src=%u dest=%u id=%u flags=%u hops=%u len=%u",
			loraPacketTypeName(messageFlag), from, dest, id, messageFlag, hops, len);
		if (len < 9) {
			Log.info("LoRA message too short - length %u", len);
			return false;
		}
		if ((buf[0] << 8 | buf[1]) != sysStatus.get_magicNumber()) {
			Log.info("Magic Number mismatch - ignoring message");
			return false;
		} 
		lora_state = (LoRA_State)messageFlag;
		Log.info("Received from node %d with RSSI / SNR of %d / %d - a %s message with %d hops", from, driver.lastRssi(), driver.lastSNR(), loraStateNames[lora_state], hops);

		Time.setTime(((buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5]));  // Set time based on response from gateway
		sysStatus.set_frequencyMinutes((buf[6] << 8 | buf[7]));			// Frequency of reporting set by Gateway

		// The gateway may set an alert code for the node
		sysStatus.set_alertCodeNode(buf[8]);
		sysStatus.set_alertTimestampNode(Time.now());

		Log.info("Set clock to %s and report frequency to %d minutes", Time.timeStr().c_str(),sysStatus.get_frequencyMinutes());

		if (lora_state == DATA_ACK) {
			if (len < 12) {
				Log.info("Data acknowledgement too short - length %u", len);
				return false;
			}
			if(LoRA_Functions::instance().receiveAcknowledmentDataReportNode()) return true;
		}
		else if (lora_state == JOIN_ACK) {
			if (len < 11) {
				Log.info("Join acknowledgement too short - length %u", len);
				return false;
			}
			if(LoRA_Functions::instance().receiveAcknowledmentJoinRequestNode()) return true;
		}
		else {Log.info("Invaled LoRA message flag"); return false;}

	}
	else LoRA_Functions::clearBuffer();
	return false;
}


bool LoRA_Functions::composeDataReportNode(uint8_t attemptNumber) {
	float successPercent;

	if (current.get_messageCount()==0) {		// 8-bit number so need to protect against divide by zero on reset or wrap around
		successPercent = 0.0;	
		current.set_messageCount(0);
		current.set_successCount(0);
	}
	else successPercent = ((current.get_successCount()+1.0)/(float)current.get_messageCount()) * 100.0;  // Add one to success because we are receving the message
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
	ensureRadioReadyForTransmit();
	logNodeSendStart("DATA_RPT", 19, attemptNumber);
	unsigned char result = manager.sendtoWait(buf, 19, GATEWAY_ADDRESS, DATA_RPT);
	logNodeSendResult("DATA_RPT", result);
	
	if ( result == RH_ROUTER_ERROR_NONE) {
		// It has been reliably delivered to the next node.
		// Now wait for a reply from the ultimate server 
		current.set_successCount(current.get_successCount()+1);
		current.set_RSSI(driver.lastRssi());				// Set these here - will send on next data report
		current.set_SNR(driver.lastSNR());
		Log.info("Node %d data report delivered - success rate %4.2f and  RSSI/SNR of %d / %d ",sysStatus.get_nodeNumber(),successPercent,current.get_RSSI(), current.get_SNR());
		digitalWrite(BLUE_LED, LOW);
		return true;
	}
	else if (result == RH_ROUTER_ERROR_NO_ROUTE) {
        Log.info("Node %d - Data report send to gateway %d failed - No Route - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS, successPercent);
    }
    else if (result == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
        Log.info("Node %d - Data report send to gateway %d failed - Unable to Deliver - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,successPercent);
	}
	else  {
		Log.info("Node %d - Data report send to gateway %d failed  - Unknown - success rate %4.2f", sysStatus.get_nodeNumber(), GATEWAY_ADDRESS,successPercent);
	}
	digitalWrite(BLUE_LED, LOW);
	return false;
}

bool LoRA_Functions::receiveAcknowledmentDataReportNode() {
	LEDStatus blinkBlue(RGB_COLOR_BLUE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

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

bool LoRA_Functions::composeJoinRequesttNode(uint8_t attemptNumber) {
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
	ensureRadioReadyForTransmit();
	logNodeSendStart("JOIN_REQ", 30, attemptNumber);
	unsigned char result = manager.sendtoWait(buf, 30, GATEWAY_ADDRESS, JOIN_REQ);
	logNodeSendResult("JOIN_REQ", result);
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

bool LoRA_Functions::sendRawTestNode() {
	ensureRadioReadyForTransmit();
	driver.setHeaderTo(GATEWAY_ADDRESS);
	driver.setHeaderFrom(sysStatus.get_nodeNumber());
	driver.setHeaderId(0);
	driver.setHeaderFlags(0, 0xff);

	uint16_t txGoodBefore = driver.txGood();
	Log.info("LoRa raw TX start: node=%u dest=%u payload=%s txGoodBefore=%u",
		sysStatus.get_nodeNumber(),
		GATEWAY_ADDRESS,
		RAW_TEST_PAYLOAD,
		txGoodBefore);

	bool sendResult = driver.send((const uint8_t *)RAW_TEST_PAYLOAD, sizeof(RAW_TEST_PAYLOAD) - 1);
	Log.info("LoRa raw TX send()=%s", sendResult ? "true" : "false");

	bool waitResult = false;
	if (sendResult) {
		waitResult = driver.waitPacketSent();
	}

	Log.info("LoRa raw TX waitPacketSent()=%s txGoodAfter=%u",
		waitResult ? "true" : "false",
		driver.txGood());

	return sendResult && waitResult;
}

bool LoRA_Functions::receiveAcknowledmentJoinRequestNode() {
	LEDStatus blinkOrange(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

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

