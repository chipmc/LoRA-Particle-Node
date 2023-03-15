/**
 * @file LoRA-Functions.h - Singleton approach
 * @author Chip McClelland (chip@seeinisghts.com)
 * @brief This file allows me to move all the LoRA functionality out of the main program - application implementation / not general
 * @version 0.1
 * @date 2022-09-14
 * 
 */

// Data exchange formats
// Format of a data report
/*
buf[0 - 1] magicNumber                      // Magic number for devices
buf[2 - 3] nodeID                           // nodeID for verification
buf[4 - 5] hourly                           // Hourly count
buf[6 - 7] daily                            // Daily Count
buf[8] sensorType                           // What sensor type is it
buf[9] temp;                                // Enclosure temp
buf[10] battChg;                            // State of charge
buf[11] battState;                          // Battery State
buf[12] resets                              // Reset count
buf[13] messageCount;                       // Sequential message number
buf[14] successCount;                       // How Many successful sends
buf[15-16] RSSI                             // From the Node's perspective
buf[17-18] SNR                              // From the Node's perspective
*/

// Format of a data acknowledgement
/*    
    buf[0 - 1 ] magicNumber                 // Magic Number
    buf[2 - 5 ] Time.now()                  // Set the time 
    buf[6 - 7] frequencyMinutes             // For the Gateway minutes on the hour
    buf[8] alertCode                        // This lets the Gateway trigger an alert on the node - typically a join request
    buf[9] sensorType                       // Let's the Gateway reset the sensor if needed 
    buf[10] openHours                       // From the Gateway to the node - is the park open?
    buf[11] message number                  // Parrot this back to see if it matches
*/

// Format of a join request
/*
buf[0-1] magicNumber;                       // Magic Number
buf[2 - 3] nodeID                            // nodeID for verification
buf[4- 28] Particle deviceID;               // deviceID is unique to the device
buf[29] sensorType				            // Identifies sensor type to Gateway
*/

// Format for a join acknowledgement
/*
    buf[0 - 1 ]  magicNumber                // Magic Number
    buf[2 - 5 ] Time.now()                  // Set the time 
    buf[6 - 7] frequencyMinutes             // For the Gateway minutes on the hour  
    buf[8] alertCodeNode                    // Gateway can set an alert code here
    buf[9]  newNodeNumber                   // New Node Number for device
    buf[10]  sensorType				        // Gateway confirms sensor type
*/

#ifndef __LORA_FUNCTIONS_H
#define __LORA_FUNCTIONS_H

#include "Particle.h"

extern uint16_t __system_product_version;

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * LoRA_Functions::instance().setup(gateway - true or false);
 * 
 * From global application loop you must call:
 * LoRA_Functions::instance().loop();
 */
class LoRA_Functions {
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use LoRA_Functions::instance() to instantiate the singleton.
     */
    static LoRA_Functions &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use LoRA_Functions::instance().setup();
     */
    bool setup(bool gatewayID);

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use LoRA_Functions::instance().loop();
     */
    void loop();


    // Common Functions
    /**
     * @brief Clear whatever message is in the buffer - good for waking
     * 
     * @details calls the driver and iterates through the message queue
     * 
    */
    void clearBuffer();

    /**
     * @brief Class to put the LoRA Radio to sleep when we exit the LoRa state
     * 
     * @details May help prevent the radio locking up based on local LoRA traffic interference
     * 
    */
    void sleepLoRaRadio();

    /**
     * @brief Initialize the LoRA radio
     * 
     */
   bool initializeRadio();

 
    // Node Functions
    /**
     * @brief Listens for the gateway to respond to the node - takes note of message flag
     * 
     * @return true 
     * @return false 
     */
    bool listenForLoRAMessageNode();                // Node - sent a message - awiting reply
    /**
     * @brief Composes a Data Report and sends to the Gateway
     * 
     * @return true 
     * @return false 
     */
    bool composeDataReportNode();                  // Node - Composes data report
    /**
     * @brief Acknowledges the response from the Gateway that acknowledges receipt of a data report
     * 
     * @return true 
     * @return false 
     */
    bool receiveAcknowledmentDataReportNode();     // Node - receives acknolwedgement
    /**
     * @brief Composes a Join Request and sends to the Gateway
     * 
     * @return true 
     * @return false 
     */
    bool composeJoinRequesttNode();                // Node - Composes Join request
    /**
     * @brief Acknowledges the response from the Gateway that acknowledges receipt of a Join Request
     * 
     * @return true 
     * @return false 
     */
    bool receiveAcknowledmentJoinRequestNode();    // Node - received join request asknowledgement
    /**
     * @brief computes a two digit checksum based on the Particle deviceID
     * 
     * @param str - a 24 character hex number string
     * @return int - a value from 0 to 360 based on the character string
     */
    int stringCheckSum(String str);


protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use LoRA_Functions::instance() to instantiate the singleton.
     */
    LoRA_Functions();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~LoRA_Functions();

    /**
     * This class is a singleton and cannot be copied
     */
    LoRA_Functions(const LoRA_Functions&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    LoRA_Functions& operator=(const LoRA_Functions&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static LoRA_Functions *_instance;

};
#endif  /* __LORA_FUNCTIONS_H */
