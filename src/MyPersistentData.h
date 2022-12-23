#ifndef __MYPERSISTENTDATA_H
#define __MYPERSISTENTDATA_H

#include "Particle.h"
#include "MB85RC256V-FRAM-RK.h"
#include "StorageHelperRK.h"

//Define external class instances. These are typically declared public in the main .CPP. I wonder if we can only declare it here?
extern MB85RC64 fram;

//Macros(#define) to swap out during pre-processing (use sparingly). This is typically used outside of this .H and .CPP file within the main .CPP file or other .CPP files that reference this header file. 
// This way you can do "data.setup()" instead of "MyPersistentData::instance().setup()" as an example
#define current currentStatusData::instance()
#define sysStatus sysStatusData::instance()

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * MyPersistentData::instance().setup();
 * 
 * From global application loop you must call:
 * MyPersistentData::instance().loop();
 */

// *******************  SysStatus Storage Object **********************
//
// ********************************************************************

class sysStatusData : public StorageHelperRK::PersistentDataFRAM {
public:

    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use MyPersistentData::instance() to instantiate the singleton.
     */
    static sysStatusData &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use MyPersistentData::instance().setup();
     */
    void setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use MyPersistentData::instance().loop();
     */
    void loop();

	/**
	 * @brief Load the appropriate system defaults - good ot initialize a system to "factory settings"
	 * 
	 */
	void loadSystemDefaults(); 

	/**
	 * @brief Checks to make sure the system values are in-order
	 * 
	 */
	void checkSystemValues();	


	class SysData {
	public:
		// This structure must always begin with the header (16 bytes)
		StorageHelperRK::PersistentDataBase::SavedDataHeader sysHeader;
		// Your fields go here. Once you've added a field you cannot add fields
		// (except at the end), insert fields, remove fields, change size of a field.
		// Doing so will cause the data to be corrupted!
		uint16_t nodeNumber;                              // Assigned by the gateway on joining the network
		uint8_t structuresVersion;                        // Version of the data structures (system and data)
		uint16_t magicNumber;							  // A way to identify nodes and gateways so they can trust each other
		uint8_t firmwareRelease;                          // Version of the device firmware (integer - aligned to particle product firmware)
		uint8_t resetCount;                               // reset count of device (0-256)
		time_t lastConnection;                     		  // Last time we successfully connected to Particle
		uint16_t frequencyMinutes;                        // When we are reporing at minute increments - what are they - for Gateways
		uint8_t alertCodeNode;                            // Alert code from node
		time_t alertTimestampNode;                 	      // Timestamp of alert
		uint8_t sensorType;                               // PIR sensor, car counter, others - this value is changed by the Gateway
		bool openHours;									  // Are we collecting data or is it outside open hours?
	};

	SysData sysData;

	// 	******************* Get and Set Functions for each variable in the storage object ***********
    
	/**
	 * @brief For the Get functions, used to retrieve the value of the variable
	 * 
	 * @details Specific to the location in the object and the type of the variable
	 * 
	 * @param Nothing needed
	 * 
	 * @returns The value of the variable in the corret type
	 * 
	 */

	/**
	 * @brief For the Set functions, used to set the value of the variable
	 * 
	 * @details Specific to the location in the object and the type of the variable
	 * 
	 * @param Value to set the variable - correct type - for Strings they are truncated if too long
	 * 
	 * @returns None needed
	 * 
	 */


	uint8_t get_nodeNumber() const;
	void set_nodeNumber(uint8_t value);

	uint8_t get_structuresVersion() const ;
	void set_structuresVersion(uint8_t value);

	uint16_t get_magicNumber() const ;
	void set_magicNumber(uint16_t value);

	uint8_t get_firmwareRelease() const;
	void set_firmwareRelease(uint8_t value);

	uint8_t get_resetCount() const;
	void set_resetCount(uint8_t value);

	time_t get_lastConnection() const;
	void set_lastConnection(time_t value);

	uint16_t get_frequencyMinutes() const;
	void set_frequencyMinutes(uint16_t value);

	uint8_t get_alertCodeNode() const;
	void set_alertCodeNode(uint8_t value);

	time_t get_alertTimestampNode() const;
	void set_alertTimestampNode(time_t value);

	uint8_t get_sensorType() const;
	void set_sensorType(uint8_t value);

	bool get_openHours() const;
	void set_openHours(bool value);

	//Members here are internal only and therefore protected
protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use MyPersistentData::instance() to instantiate the singleton.
     */
    sysStatusData();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~sysStatusData();

    /**
     * This class is a singleton and cannot be copied
     */
    sysStatusData(const sysStatusData&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    sysStatusData& operator=(const sysStatusData&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static sysStatusData *_instance;

    //Since these variables are only used internally - They can be private. 
	static const uint32_t SYS_DATA_MAGIC = 0x20a99e75;
	static const uint16_t SYS_DATA_VERSION = 2;

};




// *****************  Current Status Storage Object *******************
//
// ********************************************************************

class currentStatusData : public StorageHelperRK::PersistentDataFRAM {
public:

    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use MyPersistentData::instance() to instantiate the singleton.
     */
    static currentStatusData &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use MyPersistentData::instance().setup();
     */
    void setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use MyPersistentData::instance().loop();
     */
    void loop();

	/**
	 * @brief Load the appropriate system defaults - good ot initialize a system to "factory settings"
	 * 
	 */
	void loadCurrentDefaults();                          // Initilize the object values for new deployments

	/**
	 * @brief Resets the current and hourly counts
	 * 
	 */
	void resetEverything();  

	class CurrentData {
	public:
		// This structure must always begin with the header (16 bytes)
		StorageHelperRK::PersistentDataBase::SavedDataHeader currentHeader;
		// Your fields go here. Once you've added a field you cannot add fields
		// (except at the end), insert fields, remove fields, change size of a field.
		// Doing so will cause the data to be corrupted!
		// You may want to keep a version number in your data.
		uint8_t internalTempC;                            // Enclosure temperature in degrees C
		double stateOfCharge;                                // Battery charge level
		uint8_t batteryState;                             // Stores the current battery state (charging, discharging, etc)
		time_t lastSampleTime;                            // Timestamp of last data collection
		uint16_t RSSI;                                    // Latest signal strength value
		uint8_t messageCount;                            // What message are we on
		uint8_t successCount;							  // How many messages are delivered successfully
		time_t lastCountTime;                             // When did we last record a count
		uint16_t hourlyCount;                             // Current Hourly Count
		uint16_t dailyCount;                              // Current Daily Count
		// OK to add more fields here 
	};
	CurrentData currentData;

	// 	******************* Get and Set Functions for each variable in the storage object ***********
    
	/**
	 * @brief For the Get functions, used to retrieve the value of the variable
	 * 
	 * @details Specific to the location in the object and the type of the variable
	 * 
	 * @param Nothing needed
	 * 
	 * @returns The value of the variable in the corret type
	 * 
	 */

	/**
	 * @brief For the Set functions, used to set the value of the variable
	 * 
	 * @details Specific to the location in the object and the type of the variable
	 * 
	 * @param Value to set the variable - correct type - for Strings they are truncated if too long
	 * 
	 * @returns None needed
	 * 
	 */


	uint8_t get_internalTempC() const ;
	void set_internalTempC(uint8_t value);

	double get_stateOfCharge() const;
	void set_stateOfCharge(double value);

	uint8_t get_batteryState() const;
	void set_batteryState(uint8_t value);

	time_t get_lastSampleTime() const;
	void set_lastSampleTime(time_t value);

	uint16_t get_RSSI() const;
	void set_RSSI(uint16_t value);

	uint8_t get_messageCount() const;
	void set_messageCount(uint8_t value);

	uint8_t get_successCount() const;
	void set_successCount(uint8_t value);

	time_t get_lastCountTime() const;
	void set_lastCountTime(time_t value);

	uint16_t get_hourlyCount() const;
	void set_hourlyCount(uint16_t value);

	uint16_t get_dailyCount() const;
	void set_dailyCount(uint16_t value);

		//Members here are internal only and therefore protected
protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use MyPersistentData::instance() to instantiate the singleton.
     */
    currentStatusData();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~currentStatusData();

    /**
     * This class is a singleton and cannot be copied
     */
    currentStatusData(const currentStatusData&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    currentStatusData& operator=(const currentStatusData&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static currentStatusData *_instance;

    //Since these variables are only used internally - They can be private. 
	static const uint32_t CURRENT_DATA_MAGIC = 0x20a99e74;
	static const uint16_t CURRENT_DATA_VERSION = 2;
};


#endif  /* __MYPERSISTENTDATA_H */
